import json
import math
from collections import deque
import socket
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Optional

from fleet_features import (
    TaskDatabase,
    choose_priority_robot,
    query_robots,
    simulated_battery_percent,
    summarize_robot,
)
from gui import TelemetryGUI

HOST = "0.0.0.0"
PORT = 9000
MAX_CLIENTS = 10
GRID_CELL_CM = 10.0
GRID_DIM_CELLS = 40
ARENA_SIZE_CM = GRID_CELL_CM * GRID_DIM_CELLS
OBSTACLE_MEMORY_S = 20.0
# Firmware accepts up to MAX_WAYPOINTS (12), but its serial line buffer is only
# 512 bytes (ATmega328P SRAM is tight). A 12-waypoint message with a motion
# block is ~528 bytes and gets dropped; 8 waypoints is ~402 bytes, leaving
# comfortable headroom. Batching still avoids a serial round-trip per 10 cm,
# which was the main source of click-to-motion latency; the rest of the path
# flows on each path_complete.
MAX_WAYPOINTS_PER_ASSIGNMENT = 8
MIN_OBSTACLE_SENSOR_CM = 2.0
MAX_OBSTACLE_SENSOR_CM = 120.0
# The robot body is wider than one 10 cm cell (forward sensors sit ~14 cm apart
# and the chassis is wider still), so the planner inflates obstacles to keep the
# whole body clear. With radius 1 the path center stays only 10 cm from a wall
# cell center -- ~5 cm from the wall edge -- and the body clips it, so the robot
# appears to drive through walls. Permanent walls are known hazards and get the
# full body clearance (radius 2 ~= 20 cm). Transient sensor blips are noisy and
# numerous, so they get a lighter radius 1 to avoid walling off the whole arena.
PERMANENT_INFLATE_CELLS = 2
TRANSIENT_INFLATE_CELLS = 1
# A robot with an active goal is never abandoned: the watchdog keeps replanning
# at this cadence until it arrives or the operator presses Stop.
REPLAN_WATCHDOG_PERIOD_S = 2.0
# If a dispatch has been waiting this long for path_complete with no progress,
# assume the message/round-trip was lost and recover so the robot isn't wedged.
DISPATCH_STALL_TIMEOUT_S = 8.0
PLAN_MARGIN_CELLS = 40
PERMANENT_OBSTACLES_PATH = Path(__file__).with_name("permanent_obstacles.json")


@dataclass
class ClientSession:
    client_id: int
    conn: socket.socket
    addr: tuple
    name: str = ""
    robot_id: Optional[str] = None
    state: str = "connected"
    last_heartbeat: float = field(default_factory=time.time)
    last_telemetry: Optional[dict] = None
    last_status: Optional[dict] = None
    current_path_id: Optional[str] = None
    current_waypoint_index: Optional[int] = None
    send_lock: threading.Lock = field(default_factory=threading.Lock)
    pending_waypoints: list[dict] = field(default_factory=list)
    pending_motion: Optional[dict] = None
    sequence_path_id: Optional[int] = None
    active_subpath_id: Optional[int] = None
    awaiting_path_ack: bool = False
    awaiting_path_complete: bool = False
    awaiting_since: float = 0.0


clients_lock = threading.Lock()

# keyed by client_id
client_sessions: Dict[int, ClientSession] = {}

# keyed by robot_id
robots_by_id: Dict[str, int] = {}

next_client_id = 1
next_robot_path_id = 1000
task_db = TaskDatabase()
battery_by_robot: dict[str, float] = {}
obstacle_cells: dict[tuple[int, int], float] = {}
permanent_obstacle_cells: set[tuple[int, int]] = set()
goal_by_robot: dict[str, tuple[int, int]] = {}


def load_permanent_obstacle_cells() -> set[tuple[int, int]]:
    try:
        raw_cells = json.loads(PERMANENT_OBSTACLES_PATH.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return set()

    cells = set()
    for item in raw_cells:
        try:
            row, col = item
        except (TypeError, ValueError):
            continue
        cells.add((int(row), int(col)))
    return cells


def save_permanent_obstacle_cells() -> None:
    cells = sorted([list(cell) for cell in permanent_obstacle_cells])
    PERMANENT_OBSTACLES_PATH.write_text(json.dumps(cells, indent=2), encoding="utf-8")


permanent_obstacle_cells = load_permanent_obstacle_cells()


# ----------------------------
# Networking helpers
# ----------------------------
def send_json(conn: socket.socket, message: dict) -> None:
    data = json.dumps(message) + "\n"
    conn.sendall(data.encode("utf-8"))


def recv_lines(conn: socket.socket):
    buffer = ""
    while True:
        data = conn.recv(4096)
        if not data:
            break

        buffer += data.decode("utf-8", errors="replace")

        while "\n" in buffer:
            line, buffer = buffer.split("\n", 1)
            line = line.strip()
            if line:
                yield line


def get_session(client_id: int) -> Optional[ClientSession]:
    with clients_lock:
        return client_sessions.get(client_id)


def get_robot_snapshot() -> dict:
    with clients_lock:
        snapshot = {}
        for robot_id, client_id in robots_by_id.items():
            session = client_sessions.get(client_id)
            if session is None:
                continue

            snapshot[robot_id] = {
                "client_id": session.client_id,
                "name": session.name,
                "state": session.state,
                "path_id": session.current_path_id,
                "waypoint_index": session.current_waypoint_index,
                "last_heartbeat": session.last_heartbeat,
                "last_telemetry": session.last_telemetry,
                "last_status": session.last_status,
                "awaiting_path_complete": session.awaiting_path_complete,
                "awaiting_path_ack": session.awaiting_path_ack,
            }
        return snapshot


def print_robot_table() -> None:
    with clients_lock:
        print("\n=== Robot Table ===")
        if not client_sessions:
            print("(no clients connected)")
        for client_id, session in client_sessions.items():
            print(
                f"client_id={client_id} "
                f"robot_id={session.robot_id} "
                f"name={session.name!r} "
                f"state={session.state!r} "
                f"path_id={session.current_path_id!r} "
                f"waypoint_index={session.current_waypoint_index!r} "
                f"last_heartbeat={session.last_heartbeat:.1f}"
            )
        print("===================\n")


def print_fleet_summary() -> None:
    snapshot = get_robot_snapshot()
    blocked = sorted(obstacle_cells_snapshot())
    print("\n=== Fleet Summary ===")
    if not snapshot:
        print("(no robots available)")
    for robot_id, robot in snapshot.items():
        print(summarize_robot(robot_id, robot))
    idle = query_robots(snapshot, state="idle")
    healthy = query_robots(snapshot, min_battery=30.0)
    print(f"idle={idle}")
    print(f"battery>=30={healthy}")
    print(f"remembered_obstacles={blocked[:20]} count={len(blocked)}")
    print("=====================\n")


def send_to_robot(robot_id: str, message: dict) -> bool:
    with clients_lock:
        client_id = robots_by_id.get(robot_id)
        if client_id is None:
            print(f"[SEND] No connected session for robot_id={robot_id}")
            return False

        session = client_sessions.get(client_id)
        if session is None:
            print(f"[SEND] Session disappeared for robot_id={robot_id}")
            return False

    try:
        with session.send_lock:
            send_json(session.conn, message)

        print(f"[SEND] -> {robot_id}: {message}")
        return True

    except Exception as exc:
        print(f"[!] Failed to send to robot {robot_id}: {exc}")
        return False


def next_subpath_id() -> int:
    global next_robot_path_id
    next_robot_path_id += 1
    if next_robot_path_id > 30000:
        next_robot_path_id = 1000
    return next_robot_path_id


def clear_robot_sequence(session: ClientSession) -> None:
    session.pending_waypoints.clear()
    session.pending_motion = None
    session.sequence_path_id = None
    session.active_subpath_id = None
    session.awaiting_path_ack = False
    session.awaiting_path_complete = False
    session.awaiting_since = 0.0


def recover_dispatch_flags(robot_id: str, state: str | None = None) -> None:
    """Release the awaiting-path flags so a robot is never permanently wedged
    when a path_complete is lost or a replan can't find a route yet."""
    with clients_lock:
        client_id = robots_by_id.get(robot_id)
        session = client_sessions.get(client_id) if client_id is not None else None
        if session is None:
            return
        session.awaiting_path_ack = False
        session.awaiting_path_complete = False
        session.awaiting_since = 0.0
        session.active_subpath_id = None
        if state is not None:
            session.state = state


def clear_active_goal(robot_id: str) -> None:
    """Forget a robot's goal so the watchdog stops chasing it (arrival or Stop)."""
    goal_by_robot.pop(robot_id, None)


def any_other_robot_busy_unlocked(robot_id: str) -> bool:
    for session in client_sessions.values():
        if session.robot_id and session.robot_id != robot_id and session.awaiting_path_complete:
            return True
    return False


def maybe_dispatch_waiting_sequences() -> bool:
    with clients_lock:
        robot_ids = [
            session.robot_id
            for session in client_sessions.values()
            if session.robot_id and session.pending_waypoints and not session.awaiting_path_complete
        ]

    dispatched = False
    for robot_id in robot_ids:
        if dispatch_next_waypoint(robot_id):
            dispatched = True
            break

    return dispatched


def dispatch_next_waypoint(robot_id: str) -> bool:
    with clients_lock:
        client_id = robots_by_id.get(robot_id)
        if client_id is None:
            print(f"[SEQUENCE] No connected session for robot_id={robot_id}")
            return False

        session = client_sessions.get(client_id)
        if session is None:
            print(f"[SEQUENCE] Session disappeared for robot_id={robot_id}")
            return False

        if any_other_robot_busy_unlocked(robot_id):
            return False

        if session.awaiting_path_ack or session.awaiting_path_complete:
            return False

        if not session.pending_waypoints:
            session.pending_motion = None
            session.sequence_path_id = None
            session.active_subpath_id = None
            return False

        batch = session.pending_waypoints[:MAX_WAYPOINTS_PER_ASSIGNMENT]
        del session.pending_waypoints[:MAX_WAYPOINTS_PER_ASSIGNMENT]
        subpath_id = next_subpath_id()
        message = {
            "type": "path_assignment",
            "robot_id": robot_id,
            "path_id": subpath_id,
            "replace_existing": True,
            "waypoints": batch,
        }
        if session.pending_motion is not None:
            message["motion"] = dict(session.pending_motion)

        session.active_subpath_id = subpath_id
        session.current_path_id = str(subpath_id)
        session.current_waypoint_index = 0
        session.awaiting_path_ack = True
        session.awaiting_path_complete = True
        session.awaiting_since = time.time()

    ok = send_to_robot(robot_id, message)
    if not ok:
        with clients_lock:
            client_id = robots_by_id.get(robot_id)
            session = client_sessions.get(client_id) if client_id is not None else None
            if session is not None:
                session.pending_waypoints[:0] = batch
                session.active_subpath_id = None
                session.awaiting_path_ack = False
                session.awaiting_path_complete = False
        return False

    print(f"[SEQUENCE] dispatched subpath {subpath_id} to {robot_id} -> {len(batch)} waypoints")
    return True


def clamp_cell(value: int) -> int:
    return int(value)


def cm_to_cell(x_cm: float, y_cm: float) -> tuple[int, int] | None:
    return math.floor(y_cm / GRID_CELL_CM), math.floor(x_cm / GRID_CELL_CM)


def pose_to_cell(telemetry: dict) -> tuple[int, int] | None:
    try:
        x_cm = float(telemetry["x_cm"])
        y_cm = float(telemetry["y_cm"])
    except (KeyError, TypeError, ValueError):
        return None

    return cm_to_cell(x_cm, y_cm)


def cell_center_waypoint(cell: tuple[int, int]) -> dict:
    row, col = cell
    return {
        "x_cm": col * GRID_CELL_CM + GRID_CELL_CM / 2.0,
        "y_cm": row * GRID_CELL_CM + GRID_CELL_CM / 2.0,
    }


def prune_obstacle_cells() -> None:
    now = time.time()
    expired = [
        cell for cell, seen_at in obstacle_cells.items()
        if now - seen_at > OBSTACLE_MEMORY_S
    ]
    for cell in expired:
        obstacle_cells.pop(cell, None)


def remember_obstacle_cell(cell: tuple[int, int] | None) -> None:
    if cell is not None:
        obstacle_cells[cell] = time.time()


def front_cell_from_pose(telemetry: dict, lookahead_cm: float = 15.0) -> tuple[int, int] | None:
    """The grid cell directly ahead of the robot, used to block the wall it
    just bumped into so the planner is forced to route around it."""
    try:
        x = float(telemetry["x_cm"])
        y = float(telemetry["y_cm"])
        theta_rad = math.radians(float(telemetry["theta_deg"]))
    except (KeyError, TypeError, ValueError):
        return None

    fx = x + lookahead_cm * math.cos(theta_rad)
    fy = y + lookahead_cm * math.sin(theta_rad)
    return cm_to_cell(fx, fy)


def obstacle_cells_snapshot() -> set[tuple[int, int]]:
    prune_obstacle_cells()
    return set(obstacle_cells) | set(permanent_obstacle_cells)


def update_permanent_obstacle_cell(action: str, row: int | None = None, col: int | None = None) -> bool:
    action = str(action).lower()
    cell = None
    if row is not None and col is not None:
        cell = (clamp_cell(int(row)), clamp_cell(int(col)))

    if action == "clear":
        permanent_obstacle_cells.clear()
        save_permanent_obstacle_cells()
        print("[MAP] cleared permanent obstacles")
        return True
    if cell is None:
        return False
    if action == "remove":
        permanent_obstacle_cells.discard(cell)
    elif action == "toggle":
        if cell in permanent_obstacle_cells:
            permanent_obstacle_cells.remove(cell)
        else:
            permanent_obstacle_cells.add(cell)
    else:
        permanent_obstacle_cells.add(cell)

    save_permanent_obstacle_cells()
    print(f"[MAP] permanent_obstacles={sorted(permanent_obstacle_cells)}")
    return True


def add_sensor_obstacles_from_telemetry(telemetry: dict) -> None:
    """Convert range sensor readings into temporary blocked grid cells."""
    try:
        x = float(telemetry["x_cm"])
        y = float(telemetry["y_cm"])
        theta_deg = float(telemetry["theta_deg"])
    except (KeyError, TypeError, ValueError):
        return

    # Forward ultrasonics sit 5.5 in (~14 cm) apart, so +/-7.0 cm off center.
    # Side IR (disabled in firmware) sit at the robot edges, ~15 cm off center.
    sensors = (
        ("left_ultrasonic_cm", 9.5, 7.0, 0.0),
        ("right_ultrasonic_cm", 9.5, -7.0, 0.0),
        ("left_ir_cm", 0.0, 15.0, 90.0),
        ("right_ir_cm", 0.0, -15.0, -90.0),
    )

    theta_rad = math.radians(theta_deg)
    for field, forward_offset, lateral_offset, angle_offset in sensors:
        value = telemetry.get(field)
        if value is None:
            continue
        try:
            distance_cm = float(value)
        except (TypeError, ValueError):
            continue
        if not (MIN_OBSTACLE_SENSOR_CM <= distance_cm <= MAX_OBSTACLE_SENSOR_CM):
            continue

        sensor_x = x + forward_offset * math.cos(theta_rad) - lateral_offset * math.sin(theta_rad)
        sensor_y = y + forward_offset * math.sin(theta_rad) + lateral_offset * math.cos(theta_rad)
        ray_rad = math.radians(theta_deg + angle_offset)
        obstacle_x = sensor_x + distance_cm * math.cos(ray_rad)
        obstacle_y = sensor_y + distance_cm * math.sin(ray_rad)
        remember_obstacle_cell(cm_to_cell(obstacle_x, obstacle_y))


def dispatch_priority_task(goal_row: int, goal_col: int, priority: int = 1) -> tuple[bool, str]:
    goal = (clamp_cell(goal_row), clamp_cell(goal_col))
    task_id = task_db.create_task(
        task_type="priority_dispatch",
        priority=priority,
        goal_row=goal[0],
        goal_col=goal[1],
        payload={"goal": {"row": goal[0], "col": goal[1]}},
    )

    snapshot = get_robot_snapshot()
    robot_id = choose_priority_robot(snapshot, goal)
    if robot_id is None:
        reason = "no_available_robot"
        print(f"[TASK {task_id}] No available robot for priority dispatch to {goal}")
        return False, reason

    robot = snapshot[robot_id]
    start = pose_to_cell(robot.get("last_telemetry") or {})
    # Remember the goal up front so the watchdog keeps retrying if we can't find
    # a route right now -- the robot is never abandoned, but it also never gets
    # a blind straight line driven through a known obstacle.
    goal_by_robot[robot_id] = goal
    path = plan_route_keep_permanent(start, goal) if start is not None else None
    fallback_reason = ""
    if start is None:
        # No telemetry means no obstacle frame of reference; head for the goal
        # and let the firmware's reactive scan handle anything in the way.
        fallback_reason = "no_start_telemetry_fallback_direct_goal"
        waypoints = [cell_center_waypoint(goal)]
    elif path is None:
        # Boxed in by permanent obstacles for now. Hold rather than crash
        # through; the watchdog replans every couple seconds as the robot
        # backs up / rescans and transient obstacle memory expires.
        print(f"[TASK {task_id}] no route to {goal} yet for {robot_id}; watchdog will retry")
        task_db.update_task(task_id, "assigned", robot_id=robot_id)
        return True, "no_path_will_retry"
    else:
        waypoints = [cell_center_waypoint(cell) for cell in simplify_path(path)[1:]]
    if not waypoints:
        waypoints = [cell_center_waypoint(goal)]

    task_db.update_task(task_id, "assigned", robot_id=robot_id)
    ok = queue_robot_path({
        "type": "path_assignment",
        "robot_id": robot_id,
        "path_id": next_subpath_id(),
        "replace_existing": True,
        "waypoints": waypoints,
        "motion": None,
    })
    task_db.update_task(task_id, "dispatched" if ok else "failed", robot_id=robot_id)
    print(
        f"[TASK {task_id}] priority dispatch robot={robot_id} "
        f"goal={goal} waypoints={len(waypoints)} ok={ok} "
        f"fallback={fallback_reason!r}"
    )
    if ok:
        return True, fallback_reason or "dispatched"
    return False, "send_failed"


def replan_robot_to_goal(robot_id: str) -> bool:
    goal = goal_by_robot.get(robot_id)
    if goal is None:
        print(f"[REPLAN] No remembered goal for robot_id={robot_id}")
        return False

    with clients_lock:
        client_id = robots_by_id.get(robot_id)
        session = client_sessions.get(client_id) if client_id is not None else None
        if session is None:
            print(f"[REPLAN] No connected session for robot_id={robot_id}")
            return False

        telemetry = session.last_telemetry or {}
        start = pose_to_cell(telemetry)
        motion = dict(session.pending_motion) if isinstance(session.pending_motion, dict) else None

    if start is None:
        print(f"[REPLAN] Need telemetry before replanning robot_id={robot_id}")
        return False

    if start == goal:
        print(f"[REPLAN] robot_id={robot_id} already at goal={goal}")
        clear_active_goal(robot_id)
        return False

    # The robot stopped because a wall is directly ahead. Block that cell so
    # the planner is forced to find a different route instead of re-deriving
    # the same straight line back into the wall. The front cell stays blocked
    # in every attempt below; only the *other* remembered obstacles are relaxed.
    front_cell = front_cell_from_pose(telemetry)
    if front_cell is not None:
        remember_obstacle_cell(front_cell)
    front_block = {front_cell} if (front_cell is not None and front_cell != goal) else set()

    # Relax only transient sensor obstacles if needed, but NEVER drop permanent
    # obstacles -- driving through a known fixed hazard is worse than waiting.
    path = plan_route_keep_permanent(start, goal, front_block)

    if path is None:
        # Boxed in even ignoring sensor obstacles. Do NOT wedge the robot:
        # recover its flags so the watchdog keeps retrying, and let the firmware
        # back up + rescan on the next attempt as obstacle memory expires.
        print(f"[REPLAN] No path yet robot_id={robot_id} start={start} goal={goal} front={front_cell}; will keep retrying")
        recover_dispatch_flags(robot_id, state="needs_replan")
        return False

    waypoints = [cell_center_waypoint(cell) for cell in simplify_path(path)[1:]]
    if not waypoints:
        print(f"[REPLAN] robot_id={robot_id} already at goal={goal}")
        clear_active_goal(robot_id)
        return False

    with clients_lock:
        client_id = robots_by_id.get(robot_id)
        session = client_sessions.get(client_id) if client_id is not None else None
        if session is None:
            return False
        session.pending_waypoints = waypoints
        session.pending_motion = motion
        session.sequence_path_id = next_subpath_id()
        session.active_subpath_id = None
        session.awaiting_path_ack = False
        session.awaiting_path_complete = False
        session.state = "replanning"

    print(
        f"[REPLAN] robot_id={robot_id} start={start} goal={goal} "
        f"front_blocked={front_cell} waypoints={len(waypoints)}"
    )
    maybe_dispatch_waiting_sequences()
    return True


def inflate_blocked(
    blocked: set[tuple[int, int]],
    radius: int,
) -> set[tuple[int, int]]:
    """Grow every blocked cell by `radius` cells so the planned path keeps the
    robot's body clear of obstacles."""
    if radius <= 0:
        return set(blocked)
    inflated: set[tuple[int, int]] = set()
    for row, col in blocked:
        for d_row in range(-radius, radius + 1):
            for d_col in range(-radius, radius + 1):
                inflated.add((row + d_row, col + d_col))
    return inflated


def build_blocked_cells(
    start: tuple[int, int],
    goal: tuple[int, int],
    front_block: set[tuple[int, int]],
    drop_transient: bool,
) -> set[tuple[int, int]]:
    """Assemble the obstacle set the BFS plans against. Permanent walls (and the
    wall the robot is currently nosed against) get full body clearance; transient
    sensor obstacles get a lighter touch and can be dropped entirely if they have
    boxed the robot in. Start and goal are always left traversable."""
    permanent = set(permanent_obstacle_cells) | set(front_block)
    blocked = inflate_blocked(permanent, PERMANENT_INFLATE_CELLS)
    if not drop_transient:
        transient = set(obstacle_cells_snapshot()) - set(permanent_obstacle_cells)
        blocked |= inflate_blocked(transient, TRANSIENT_INFLATE_CELLS)
    blocked.discard(start)
    blocked.discard(goal)
    return blocked


def plan_grid_path(
    start: tuple[int, int],
    goal: tuple[int, int],
    blocked: set[tuple[int, int]],
) -> list[tuple[int, int]] | None:
    # NOTE: `blocked` must already be inflated by the caller (see
    # build_blocked_cells / plan_route_keep_permanent). This is pure BFS.
    if start == goal:
        return [start]

    min_row = min(start[0], goal[0]) - PLAN_MARGIN_CELLS
    max_row = max(start[0], goal[0]) + PLAN_MARGIN_CELLS
    min_col = min(start[1], goal[1]) - PLAN_MARGIN_CELLS
    max_col = max(start[1], goal[1]) + PLAN_MARGIN_CELLS

    queue = deque([start])
    came_from = {start: None}

    while queue:
        row, col = queue.popleft()
        for d_row, d_col in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            next_cell = (row + d_row, col + d_col)
            next_row, next_col = next_cell
            if not (min_row <= next_row <= max_row and min_col <= next_col <= max_col):
                continue
            if next_cell in blocked and next_cell != goal:
                continue
            if next_cell in came_from:
                continue

            came_from[next_cell] = (row, col)
            if next_cell == goal:
                path = [goal]
                current = (row, col)
                while current is not None:
                    path.append(current)
                    current = came_from[current]
                path.reverse()
                return path

            queue.append(next_cell)

    return None


def simplify_path(path: list[tuple[int, int]]) -> list[tuple[int, int]]:
    """Drop intermediate cells on a straight run, keeping only the start, the
    corners where the direction changes, and the goal. The robot then drives
    each straight leg in one motion instead of stopping every 10 cm at every
    grid cell, which is what made motion 'go a bit and stop a lot'."""
    if len(path) <= 2:
        return list(path)
    simplified = [path[0]]
    for i in range(1, len(path) - 1):
        prev_row, prev_col = path[i - 1]
        cur_row, cur_col = path[i]
        next_row, next_col = path[i + 1]
        in_dir = (cur_row - prev_row, cur_col - prev_col)
        out_dir = (next_row - cur_row, next_col - cur_col)
        if in_dir != out_dir:
            simplified.append(path[i])
    simplified.append(path[-1])
    return simplified


def plan_route_keep_permanent(
    start: tuple[int, int],
    goal: tuple[int, int],
    front_block: set[tuple[int, int]] | None = None,
) -> list[tuple[int, int]] | None:
    """Plan a route that ALWAYS respects permanent obstacles. We relax only the
    transient (sensor) obstacles if the full set boxes the robot in -- permanent
    obstacles are fixed hazards and must never be driven through. Returns None
    if no route avoids the permanent obstacles, in which case the caller should
    hold and let the watchdog retry rather than crash through a known wall."""
    front_block = set(front_block) if front_block else set()
    for drop_transient in (False, True):
        blocked = build_blocked_cells(start, goal, front_block, drop_transient)
        path = plan_grid_path(start, goal, blocked)
        if path is not None:
            return path
    return None


def start_coordinated_traverse(message: dict) -> bool:
    robots = message.get("robots", [])
    if len(robots) != 2:
        print("[COORD] Expected exactly two robots in coordinated_traverse request")
        return False

    robot_one = robots[0]
    robot_two = robots[1]

    robot_one_id = str(robot_one.get("robot_id", "")).strip()
    robot_two_id = str(robot_two.get("robot_id", "")).strip()
    if not robot_one_id or not robot_two_id or robot_one_id == robot_two_id:
        print("[COORD] Need two distinct robot ids for coordinated traverse")
        return False

    with clients_lock:
        client_one_id = robots_by_id.get(robot_one_id)
        client_two_id = robots_by_id.get(robot_two_id)
        session_one = client_sessions.get(client_one_id) if client_one_id is not None else None
        session_two = client_sessions.get(client_two_id) if client_two_id is not None else None

        if session_one is None or session_two is None:
            print("[COORD] One or both robots are not connected")
            return False

        start_one = pose_to_cell(session_one.last_telemetry or {})
        start_two = pose_to_cell(session_two.last_telemetry or {})
        if start_one is None or start_two is None:
            print("[COORD] Need telemetry from both robots before planning a coordinated traverse")
            return False

        goal_one = (clamp_cell(int(robot_one["goal_row"])), clamp_cell(int(robot_one["goal_col"])))
        goal_two = (clamp_cell(int(robot_two["goal_row"])), clamp_cell(int(robot_two["goal_col"])))

        clear_robot_sequence(session_one)
        clear_robot_sequence(session_two)

    if start_one == start_two:
        print("[COORD] Robots are in the same grid cell; refusing to plan until they are separated")
        return False

    if goal_one == goal_two:
        print("[COORD] Robots cannot share the same goal cell")
        return False

    blocked_one = build_blocked_cells(start_one, goal_one, set(), drop_transient=False)
    blocked_one |= inflate_blocked({start_two}, TRANSIENT_INFLATE_CELLS)
    blocked_one.discard(start_one)
    blocked_one.discard(goal_one)

    path_one = plan_grid_path(start_one, goal_one, blocked_one)
    if path_one is None:
        print(f"[COORD] No path found for {robot_one_id} from {start_one} to {goal_one}")
        return False

    blocked_two = build_blocked_cells(start_two, goal_two, set(), drop_transient=False)
    blocked_two |= inflate_blocked({goal_one}, TRANSIENT_INFLATE_CELLS)
    blocked_two.discard(start_two)
    blocked_two.discard(goal_two)

    path_two = plan_grid_path(start_two, goal_two, blocked_two)
    if path_two is None:
        print(f"[COORD] No path found for {robot_two_id} from {start_two} to {goal_two}")
        return False

    with clients_lock:
        session_one = client_sessions[robots_by_id[robot_one_id]]
        session_two = client_sessions[robots_by_id[robot_two_id]]

        session_one.pending_waypoints = [cell_center_waypoint(cell) for cell in path_one[1:]]
        session_one.pending_motion = None
        session_one.sequence_path_id = next_subpath_id()
        session_one.active_subpath_id = None
        session_one.awaiting_path_ack = False
        session_one.awaiting_path_complete = False

        session_two.pending_waypoints = [cell_center_waypoint(cell) for cell in path_two[1:]]
        session_two.pending_motion = None
        session_two.sequence_path_id = next_subpath_id()
        session_two.active_subpath_id = None
        session_two.awaiting_path_ack = False
        session_two.awaiting_path_complete = False

    print(f"[COORD] {robot_one_id}: {start_one} -> {goal_one} using {len(path_one) - 1} steps")
    print(f"[COORD] {robot_two_id}: {start_two} -> {goal_two} using {len(path_two) - 1} steps")
    maybe_dispatch_waiting_sequences()
    return True


def queue_robot_path(message: dict) -> bool:
    robot_id = message.get("robot_id")
    if not robot_id:
        return False

    waypoints = []
    for waypoint in message.get("waypoints", []):
        x_cm = waypoint.get("x_cm")
        y_cm = waypoint.get("y_cm")
        if x_cm is None or y_cm is None:
            continue
        waypoints.append({
            "x_cm": float(x_cm),
            "y_cm": float(y_cm),
        })

    if not waypoints:
        print(f"[SEQUENCE] Refusing to queue empty path for {robot_id}")
        return False

    final_waypoint = waypoints[-1]
    final_goal = pose_to_cell({
        "x_cm": final_waypoint["x_cm"],
        "y_cm": final_waypoint["y_cm"],
    })
    if final_goal is not None:
        goal_by_robot[robot_id] = final_goal

    with clients_lock:
        client_id = robots_by_id.get(robot_id)
        if client_id is None:
            print(f"[SEQUENCE] No connected session for robot_id={robot_id}")
            return False

        session = client_sessions.get(client_id)
        if session is None:
            print(f"[SEQUENCE] Session disappeared for robot_id={robot_id}")
            return False

        session.pending_waypoints = waypoints
        session.pending_motion = dict(message["motion"]) if isinstance(message.get("motion"), dict) else None
        session.sequence_path_id = int(message.get("path_id", next_subpath_id()))
        session.active_subpath_id = None
        session.awaiting_path_ack = False
        session.awaiting_path_complete = False

    maybe_dispatch_waiting_sequences()
    return True


# ----------------------------
# GUI command callback
# ----------------------------
def gui_command_sender(message_obj):
    """
    Called by GUI button handlers.
    message_obj is one of your dataclass message objects from messages.py.
    """
    try:
        message = message_obj if isinstance(message_obj, dict) else message_obj.to_dict()
        robot_id = message.get("robot_id")

        if message.get("type") == "coordinated_traverse":
            ok = start_coordinated_traverse(message)
            if ok:
                print("[GUI SEND] Started coordinated two-robot traverse")
            else:
                print("[GUI SEND] Failed to start coordinated two-robot traverse")
            return ok

        if message.get("type") == "priority_dispatch":
            goal_row = int(message.get("goal_row", 0))
            goal_col = int(message.get("goal_col", 0))
            priority = int(message.get("priority", 1))
            ok, reason = dispatch_priority_task(goal_row, goal_col, priority)
            if ok:
                print(f"[GUI SEND] Started priority dispatch ({reason})")
            else:
                print(f"[GUI SEND] Failed priority dispatch ({reason})")
            return {"ok": ok, "reason": reason}

        if message.get("type") == "fleet_query":
            print_fleet_summary()
            return True

        if message.get("type") == "permanent_obstacle_update":
            ok = update_permanent_obstacle_cell(
                message.get("action", "toggle"),
                message.get("row"),
                message.get("col"),
            )
            return ok

        if not robot_id:
            print("[GUI SEND] Refusing to send message with no robot_id")
            return False

        if message.get("type") == "path_assignment":
            ok = queue_robot_path(message)
        else:
            if message.get("type") in {"stop", "continuous_drive", "set_pose"}:
                # Operator override cancels the autonomous goal so the watchdog
                # stops trying to drive the old route.
                clear_active_goal(robot_id)
                with clients_lock:
                    client_id = robots_by_id.get(robot_id)
                    session = client_sessions.get(client_id) if client_id is not None else None
                    if session is not None:
                        clear_robot_sequence(session)

            ok = send_to_robot(robot_id, message)

        if ok:
            print(f"[GUI SEND] Sent {message.get('type')} to {robot_id}")
        else:
            print(f"[GUI SEND] Failed to send {message.get('type')} to {robot_id}")
        return ok

    except Exception as exc:
        print(f"[GUI SEND] Error building/sending message: {exc}")
        return False


# Initialize GUI
gui = TelemetryGUI(command_sender=gui_command_sender)


# ----------------------------
# Session / identity helpers
# ----------------------------
def touch_session(client_id: int) -> None:
    with clients_lock:
        session = client_sessions.get(client_id)
        if session:
            session.last_heartbeat = time.time()


def bind_identity_from_message(client_id: int, msg: dict) -> None:
    with clients_lock:
        session = client_sessions[client_id]

        if "name" in msg:
            session.name = str(msg["name"])
        elif "client_name" in msg:
            session.name = str(msg["client_name"])

        if "robot_id" in msg and msg["robot_id"] is not None:
            robot_id = str(msg["robot_id"])
            session.robot_id = robot_id
            robots_by_id[robot_id] = client_id

        if "state" in msg and msg["state"] is not None:
            session.state = str(msg["state"])

        if "path_id" in msg and msg["path_id"] is not None:
            session.current_path_id = str(msg["path_id"])

        if "waypoint_index" in msg and msg["waypoint_index"] is not None:
            try:
                session.current_waypoint_index = int(msg["waypoint_index"])
            except (TypeError, ValueError):
                pass


# ----------------------------
# Message handlers
# ----------------------------
def handle_hello(client_id: int, msg: dict, conn: socket.socket) -> None:
    touch_session(client_id)
    bind_identity_from_message(client_id, msg)

    session = get_session(client_id)
    print(
        f"[Client {client_id}] registered "
        f"name={session.name!r}, robot_id={session.robot_id!r}, state={session.state!r}"
    )

    send_json(conn, {
        "type": "ack",
        "for": "hello",
        "client_id": client_id,
        "robot_id": session.robot_id,
    })

    print_robot_table()


def handle_telemetry(client_id: int, msg: dict) -> None:
    touch_session(client_id)
    bind_identity_from_message(client_id, msg)

    with clients_lock:
        session = client_sessions[client_id]
        robot_id = session.robot_id or str(msg.get("robot_id", f"client_{client_id}"))
        msg["battery_percent"] = simulated_battery_percent(
            robot_id,
            msg,
            battery_by_robot.get(robot_id),
        )
        battery_by_robot[robot_id] = msg["battery_percent"]
        add_sensor_obstacles_from_telemetry(msg)
        session.last_telemetry = msg

    if session.robot_id:
        gui.update_robot(session.robot_id, msg)

    robot_label = session.robot_id if session.robot_id else f"client_{client_id}"
    print(f"[{robot_label}] telemetry: {msg}")


def handle_status(client_id: int, msg: dict) -> None:
    touch_session(client_id)
    bind_identity_from_message(client_id, msg)

    with clients_lock:
        session = client_sessions[client_id]
        session.last_status = msg

        # Merge status into GUI-visible state if we have prior telemetry
        merged = dict(session.last_telemetry or {})
        merged.update(msg)

    if session.robot_id:
        gui.update_robot(session.robot_id, merged)

    robot_label = session.robot_id if session.robot_id else f"client_{client_id}"
    print(f"[{robot_label}] status: {msg}")

    if msg.get("state") == "needs_replan" and session.robot_id:
        replan_robot_to_goal(session.robot_id)


def handle_path_event(client_id: int, msg: dict) -> None:
    touch_session(client_id)
    bind_identity_from_message(client_id, msg)

    with clients_lock:
        session = client_sessions[client_id]

        # Path lifecycle messages should be visible in the GUI the same way
        # status updates are, while preserving the most recent telemetry pose.
        merged = dict(session.last_telemetry or {})
        merged.update(msg)

        event_type = str(msg.get("type", ""))
        if event_type == "path_started":
            merged["state"] = "executing_path"
            session.state = "executing_path"
        elif event_type == "waypoint_reached":
            merged["state"] = "waypoint_reached"
            session.state = "waypoint_reached"
        elif event_type == "path_complete":
            merged["state"] = "idle"
            session.state = "idle"
            session.current_waypoint_index = None
            session.current_path_id = None
            session.awaiting_path_complete = False
            session.active_subpath_id = None

        session.last_status = merged

    if session.robot_id:
        gui.update_robot(session.robot_id, merged)

    robot_label = session.robot_id if session.robot_id else f"client_{client_id}"
    print(f"[{robot_label}] {msg.get('type')}: {msg}")

    if msg.get("type") == "path_complete":
        if session.robot_id:
            # If more batches remain, send the next one; otherwise the whole
            # route is done, so the robot has reached its goal -> forget it.
            if session.pending_waypoints:
                dispatch_next_waypoint(session.robot_id)
            else:
                clear_active_goal(session.robot_id)
        maybe_dispatch_waiting_sequences()


def handle_ack(client_id: int, msg: dict) -> None:
    touch_session(client_id)
    bind_identity_from_message(client_id, msg)

    with clients_lock:
        session = client_sessions.get(client_id)
        if session is not None and msg.get("for") == "path_assignment":
            ack_path_id = msg.get("path_id")
            if ack_path_id is None or session.active_subpath_id is None:
                session.awaiting_path_ack = False
            else:
                try:
                    if int(ack_path_id) == session.active_subpath_id:
                        session.awaiting_path_ack = False
                except (TypeError, ValueError):
                    session.awaiting_path_ack = False

    robot_label = session.robot_id if session and session.robot_id else f"client_{client_id}"
    print(f"[{robot_label}] ack: {msg}")


def handle_heartbeat(client_id: int, conn: socket.socket, msg: dict) -> None:
    touch_session(client_id)
    bind_identity_from_message(client_id, msg)

    session = get_session(client_id)
    send_json(conn, {
        "type": "heartbeat_ack",
        "robot_id": session.robot_id if session else None,
        "server_t": time.time(),
    })


# ----------------------------
# Client thread
# ----------------------------
def handle_client(client_id: int, conn: socket.socket, addr) -> None:
    print(f"[+] Client {client_id} connected from {addr}")

    try:
        send_json(conn, {
            "type": "hello_ack",
            "client_id": client_id,
            "message": "connected",
        })

        for line in recv_lines(conn):
            try:
                msg = json.loads(line)
            except json.JSONDecodeError:
                print(f"[Client {client_id}] Bad JSON: {line}")
                continue

            msg_type = msg.get("type", "")

            if msg_type == "hello":
                handle_hello(client_id, msg, conn)

            elif msg_type == "telemetry":
                handle_telemetry(client_id, msg)

            elif msg_type == "heartbeat":
                handle_heartbeat(client_id, conn, msg)

            elif msg_type == "status":
                handle_status(client_id, msg)

            elif msg_type in {"path_started", "waypoint_reached", "path_complete"}:
                handle_path_event(client_id, msg)

            elif msg_type == "ack":
                handle_ack(client_id, msg)

            else:
                print(f"[Client {client_id}] unknown message type: {msg_type}")
                send_json(conn, {
                    "type": "error",
                    "message": f"unknown message type: {msg_type}",
                })

    except ConnectionResetError:
        print(f"[!] Client {client_id} connection reset")
    except Exception as exc:
        print(f"[!] Client {client_id} error: {exc}")
    finally:
        with clients_lock:
            session = client_sessions.pop(client_id, None)
            if session and session.robot_id:
                mapped_client_id = robots_by_id.get(session.robot_id)
                if mapped_client_id == client_id:
                    robots_by_id.pop(session.robot_id, None)
                    goal_by_robot.pop(session.robot_id, None)

        conn.close()
        print(f"[-] Client {client_id} disconnected")
        print_robot_table()


# ----------------------------
# Accept loop
# ----------------------------
def accept_loop(server_sock: socket.socket) -> None:
    global next_client_id

    while True:
        conn, addr = server_sock.accept()

        with clients_lock:
            if len(client_sessions) >= MAX_CLIENTS:
                send_json(conn, {
                    "type": "error",
                    "message": "server full",
                })
                conn.close()
                continue

            client_id = next_client_id
            next_client_id += 1

            client_sessions[client_id] = ClientSession(
                client_id=client_id,
                conn=conn,
                addr=addr,
            )

        thread = threading.Thread(
            target=handle_client,
            args=(client_id, conn, addr),
            daemon=True,
        )
        thread.start()
        print_robot_table()


# ----------------------------
# Replan watchdog
# ----------------------------
def tick_replan_watchdog() -> None:
    """Keep every robot that has an active goal moving toward it. A goal is only
    forgotten on arrival or when the operator presses Stop, so the fleet never
    silently gives up on a dispatch."""
    now = time.time()
    actions: list[tuple[str, str]] = []
    with clients_lock:
        for robot_id in list(goal_by_robot.keys()):
            client_id = robots_by_id.get(robot_id)
            session = client_sessions.get(client_id) if client_id is not None else None
            if session is None:
                continue
            if session.awaiting_path_complete:
                stalled = session.awaiting_since and (now - session.awaiting_since) > DISPATCH_STALL_TIMEOUT_S
                if stalled:
                    actions.append((robot_id, "recover"))
                continue
            if session.pending_waypoints:
                actions.append((robot_id, "dispatch"))
            else:
                actions.append((robot_id, "replan"))

    for robot_id, action in actions:
        if action == "recover":
            print(f"[WATCHDOG] dispatch stalled for {robot_id}; recovering and retrying")
            recover_dispatch_flags(robot_id, state="needs_replan")
        elif action == "dispatch":
            dispatch_next_waypoint(robot_id)
        elif action == "replan":
            replan_robot_to_goal(robot_id)


def replan_watchdog_loop() -> None:
    while True:
        time.sleep(REPLAN_WATCHDOG_PERIOD_S)
        try:
            tick_replan_watchdog()
        except Exception as exc:
            print(f"[WATCHDOG] error: {exc}")


# ----------------------------
# Server main
# ----------------------------
def server_main() -> None:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_sock:
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_sock.bind((HOST, PORT))
        server_sock.listen()
        print(f"Server listening on {HOST}:{PORT}")

        accept_loop(server_sock)


def main() -> None:
    server_thread = threading.Thread(target=server_main, daemon=True)
    server_thread.start()

    watchdog_thread = threading.Thread(target=replan_watchdog_loop, daemon=True)
    watchdog_thread.start()

    gui.run()


if __name__ == "__main__":
    main()
