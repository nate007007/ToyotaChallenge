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
MIN_OBSTACLE_SENSOR_CM = 2.0
MAX_OBSTACLE_SENSOR_CM = 120.0
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
        cells.add((max(0, min(GRID_DIM_CELLS - 1, int(row))), max(0, min(GRID_DIM_CELLS - 1, int(col)))))
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

        waypoint = session.pending_waypoints.pop(0)
        subpath_id = next_subpath_id()
        message = {
            "type": "path_assignment",
            "robot_id": robot_id,
            "path_id": subpath_id,
            "replace_existing": True,
            "waypoints": [waypoint],
        }
        if session.pending_motion is not None:
            message["motion"] = dict(session.pending_motion)

        session.active_subpath_id = subpath_id
        session.current_path_id = str(subpath_id)
        session.current_waypoint_index = 0
        session.awaiting_path_ack = True
        session.awaiting_path_complete = True

    ok = send_to_robot(robot_id, message)
    if not ok:
        with clients_lock:
            client_id = robots_by_id.get(robot_id)
            session = client_sessions.get(client_id) if client_id is not None else None
            if session is not None:
                session.pending_waypoints.insert(0, waypoint)
                session.active_subpath_id = None
                session.awaiting_path_ack = False
                session.awaiting_path_complete = False
        return False

    print(f"[SEQUENCE] dispatched subpath {subpath_id} to {robot_id} -> {waypoint}")
    return True


def clamp_cell(value: int) -> int:
    return max(0, min(GRID_DIM_CELLS - 1, value))


def cm_to_cell(x_cm: float, y_cm: float) -> tuple[int, int] | None:
    if not (0.0 <= x_cm <= ARENA_SIZE_CM and 0.0 <= y_cm <= ARENA_SIZE_CM):
        return None
    return clamp_cell(int(y_cm // GRID_CELL_CM)), clamp_cell(int(x_cm // GRID_CELL_CM))


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

    sensors = (
        ("front_ultrasonic_cm", 9.5, 0.0, 0.0),
        ("left_ultrasonic_cm", 9.5, 16.0, 0.0),
        ("right_ultrasonic_cm", 9.5, -16.0, 0.0),
        ("front_ir_cm", 9.5, 0.0, 0.0),
        ("left_ir_cm", 0.0, 16.0, 90.0),
        ("right_ir_cm", 0.0, -16.0, -90.0),
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
    blocked = obstacle_cells_snapshot()
    if start is not None:
        blocked.discard(start)
    blocked.discard(goal)
    path = plan_grid_path(start, goal, blocked) if start is not None else None
    fallback_reason = ""
    if start is None:
        fallback_reason = "no_start_telemetry_fallback_direct_goal"
        waypoints = [cell_center_waypoint(goal)]
    elif path is None:
        fallback_reason = "no_path_fallback_direct_goal"
        waypoints = [cell_center_waypoint(goal)]
    else:
        waypoints = [cell_center_waypoint(cell) for cell in path[1:]]
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
        f"goal={goal} waypoints={len(waypoints)} blocked={len(blocked)} ok={ok} "
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

        start = pose_to_cell(session.last_telemetry or {})
        motion = dict(session.pending_motion) if isinstance(session.pending_motion, dict) else None

    if start is None:
        print(f"[REPLAN] Need telemetry before replanning robot_id={robot_id}")
        return False

    blocked = obstacle_cells_snapshot()
    blocked.discard(start)
    blocked.discard(goal)
    path = plan_grid_path(start, goal, blocked)
    if path is None:
        print(f"[REPLAN] No path robot_id={robot_id} start={start} goal={goal} blocked={len(blocked)}")
        return False

    waypoints = [cell_center_waypoint(cell) for cell in path[1:]]
    if not waypoints:
        print(f"[REPLAN] robot_id={robot_id} already at goal={goal}")
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
        f"waypoints={len(waypoints)} blocked={len(blocked)}"
    )
    maybe_dispatch_waiting_sequences()
    return True


def plan_grid_path(
    start: tuple[int, int],
    goal: tuple[int, int],
    blocked: set[tuple[int, int]],
) -> list[tuple[int, int]] | None:
    if start == goal:
        return [start]

    queue = deque([start])
    came_from = {start: None}

    while queue:
        row, col = queue.popleft()
        for d_row, d_col in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            next_cell = (row + d_row, col + d_col)
            next_row, next_col = next_cell
            if not (0 <= next_row < GRID_DIM_CELLS and 0 <= next_col < GRID_DIM_CELLS):
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

    dynamic_blocked = obstacle_cells_snapshot()
    dynamic_blocked.discard(start_one)
    dynamic_blocked.discard(start_two)
    dynamic_blocked.discard(goal_one)
    dynamic_blocked.discard(goal_two)

    path_one = plan_grid_path(start_one, goal_one, dynamic_blocked | {start_two})
    if path_one is None:
        print(f"[COORD] No path found for {robot_one_id} from {start_one} to {goal_one}")
        return False

    path_two = plan_grid_path(start_two, goal_two, dynamic_blocked | {goal_one})
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
            if message.get("type") in {"stop", "continuous_drive"}:
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
            dispatch_next_waypoint(session.robot_id)
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

    gui.run()


if __name__ == "__main__":
    main()
