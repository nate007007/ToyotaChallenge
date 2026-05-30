from __future__ import annotations

import json
import sqlite3
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional


DB_PATH = Path(__file__).with_name("fleet_tasks.db")
ARENA_SIZE_CM = 400.0
GRID_CELL_CM = 10.0


def stable_battery_seed(robot_id: str) -> float:
    return 75.0 + (sum(ord(ch) for ch in robot_id) % 20)


def simulated_battery_percent(robot_id: str, telemetry: dict, last_battery: Optional[float]) -> float:
    """Estimate a demo battery level without changing robot firmware."""
    base = stable_battery_seed(robot_id) if last_battery is None else last_battery
    state = str(telemetry.get("state", "")).lower()
    drain = 0.006
    if state in {"executing_path", "moving", "waypoint_reached"}:
        drain = 0.025
    elif state in {"idle", "ready", "connected"}:
        drain = 0.003
    return max(5.0, round(base - drain, 2))


def pose_to_cell_from_telemetry(telemetry: dict) -> tuple[int, int] | None:
    try:
        x_cm = float(telemetry["x_cm"])
        y_cm = float(telemetry["y_cm"])
    except (KeyError, TypeError, ValueError):
        return None

    col = max(0, min(39, int(x_cm // GRID_CELL_CM)))
    row = max(0, min(39, int(y_cm // GRID_CELL_CM)))
    return row, col


def summarize_robot(robot_id: str, snapshot: dict) -> str:
    telemetry = snapshot.get("last_telemetry") or {}
    cell = pose_to_cell_from_telemetry(telemetry)
    state = snapshot.get("state", "unknown")
    battery = telemetry.get("battery_percent", "?")
    path_id = snapshot.get("path_id") or telemetry.get("path_id", "-")
    return f"{robot_id}: state={state}, battery={battery}%, cell={cell}, path={path_id}"


def query_robots(snapshot: dict, state: str | None = None, min_battery: float | None = None) -> list[str]:
    matches = []
    for robot_id, robot in snapshot.items():
        telemetry = robot.get("last_telemetry") or {}
        if state is not None and str(robot.get("state", "")).lower() != state.lower():
            continue
        if min_battery is not None:
            try:
                if float(telemetry.get("battery_percent", 0.0)) < min_battery:
                    continue
            except (TypeError, ValueError):
                continue
        matches.append(robot_id)
    return sorted(matches)


def choose_priority_robot(snapshot: dict, goal_cell: tuple[int, int]) -> str | None:
    """Pick the best robot for an urgent task, preferring idle, nearby, high-battery robots."""
    candidates = []
    busy_states = {
        "executing_path",
        "moving",
        "avoiding_obstacle",
        "replanning",
        "turning",
        "driving",
    }
    for robot_id, robot in snapshot.items():
        telemetry = robot.get("last_telemetry") or {}
        cell = pose_to_cell_from_telemetry(telemetry)
        if cell is None:
            continue
        state = str(robot.get("state") or telemetry.get("state") or "").lower()
        try:
            battery = float(telemetry.get("battery_percent", 50.0))
        except (TypeError, ValueError):
            battery = 50.0
        distance = abs(cell[0] - goal_cell[0]) + abs(cell[1] - goal_cell[1])
        busy_penalty = 1000 if robot.get("awaiting_path_complete") or state in busy_states else 0
        candidates.append((busy_penalty, distance, -battery, robot_id))

    if not candidates:
        return None
    candidates.sort()
    return candidates[0][3]


@dataclass
class TaskRecord:
    task_id: int
    task_type: str
    priority: int
    status: str
    robot_id: Optional[str]
    goal_row: Optional[int]
    goal_col: Optional[int]
    payload_json: str
    created_at: float
    updated_at: float


class TaskDatabase:
    def __init__(self, path: Path = DB_PATH):
        self.path = path
        self._init_db()

    def _connect(self):
        return sqlite3.connect(self.path)

    def _init_db(self) -> None:
        with self._connect() as conn:
            conn.execute(
                """
                CREATE TABLE IF NOT EXISTS tasks (
                    task_id INTEGER PRIMARY KEY AUTOINCREMENT,
                    task_type TEXT NOT NULL,
                    priority INTEGER NOT NULL DEFAULT 5,
                    status TEXT NOT NULL DEFAULT 'queued',
                    robot_id TEXT,
                    goal_row INTEGER,
                    goal_col INTEGER,
                    payload_json TEXT NOT NULL DEFAULT '{}',
                    created_at REAL NOT NULL,
                    updated_at REAL NOT NULL
                )
                """
            )

    def create_task(
        self,
        task_type: str,
        priority: int = 5,
        robot_id: str | None = None,
        goal_row: int | None = None,
        goal_col: int | None = None,
        payload: dict | None = None,
    ) -> int:
        now = time.time()
        with self._connect() as conn:
            cur = conn.execute(
                """
                INSERT INTO tasks (
                    task_type, priority, status, robot_id, goal_row, goal_col,
                    payload_json, created_at, updated_at
                )
                VALUES (?, ?, 'queued', ?, ?, ?, ?, ?, ?)
                """,
                (
                    task_type,
                    int(priority),
                    robot_id,
                    goal_row,
                    goal_col,
                    json.dumps(payload or {}, separators=(",", ":")),
                    now,
                    now,
                ),
            )
            return int(cur.lastrowid)

    def update_task(self, task_id: int, status: str, robot_id: str | None = None) -> None:
        with self._connect() as conn:
            conn.execute(
                """
                UPDATE tasks
                SET status = ?, robot_id = COALESCE(?, robot_id), updated_at = ?
                WHERE task_id = ?
                """,
                (status, robot_id, time.time(), task_id),
            )

    def list_tasks(self, limit: int = 20, status: str | None = None) -> list[TaskRecord]:
        query = "SELECT * FROM tasks"
        params = []
        if status:
            query += " WHERE status = ?"
            params.append(status)
        query += " ORDER BY priority ASC, created_at ASC LIMIT ?"
        params.append(limit)

        with self._connect() as conn:
            conn.row_factory = sqlite3.Row
            rows = conn.execute(query, params).fetchall()
            return [TaskRecord(**dict(row)) for row in rows]
