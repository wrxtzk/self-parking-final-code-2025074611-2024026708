"""학생 자율주차 알고리즘 스켈레톤 모듈.

이 파일만 수정하면 되고, 네트워킹/IPC 관련 코드는 `ipc_client.py`에서
자동으로 처리합니다. 학생은 아래 `PlannerSkeleton` 클래스나 `planner_step`
함수를 원하는 로직으로 교체/확장하면 됩니다.
"""

from dataclasses import dataclass
import math
from typing import Any, Dict, List, Optional, Tuple


Point = Tuple[float, float]


def pretty_print_map_summary(map_payload: Dict[str, Any]) -> None:
    extent = map_payload.get("extent") or [None, None, None, None]
    slots = map_payload.get("slots") or []
    occupied = map_payload.get("occupied_idx") or []
    free_slots = len(slots) - sum(1 for v in occupied if v)
    print("[algo] map extent :", extent)
    print("[algo] total slots:", len(slots), "/ free:", free_slots)
    stationary = map_payload.get("grid", {}).get("stationary")
    if stationary:
        rows = len(stationary)
        cols = len(stationary[0]) if stationary else 0
        print("[algo] grid size  :", rows, "x", cols)


@dataclass
class PlannerSkeleton:
    """Rule-based parking planner with Stanley path tracking."""

    map_data: Optional[Dict[str, Any]] = None
    map_extent: Optional[Tuple[float, float, float, float]] = None
    cell_size: float = 0.5
    stationary_grid: Optional[List[List[float]]] = None
    waypoints: List[Point] = None
    path_signature: Optional[Tuple[float, ...]] = None
    phase: str = "approach"
    approach_path: List[Point] = None
    parking_path: List[Point] = None
    final_gear: str = "D"
    final_yaw: float = math.pi / 2.0
    last_target_idx: int = 0

    def __post_init__(self) -> None:
        if self.waypoints is None:
            self.waypoints = []
        if self.approach_path is None:
            self.approach_path = []
        if self.parking_path is None:
            self.parking_path = []

    def set_map(self, map_payload: Dict[str, Any]) -> None:
        """시뮬레이터에서 전송한 정적 맵 데이터를 보관합니다."""

        self.map_data = map_payload
        self.map_extent = tuple(
            map(float, map_payload.get("extent", (0.0, 0.0, 0.0, 0.0)))
        )
        self.cell_size = float(map_payload.get("cellSize", 0.5))
        self.stationary_grid = map_payload.get("grid", {}).get("stationary")
        pretty_print_map_summary(map_payload)
        self.path_signature = None
        self.phase = "approach"
        self.waypoints.clear()
        self.approach_path.clear()
        self.parking_path.clear()

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    @staticmethod
    def _wrap_to_pi(angle: float) -> float:
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    @staticmethod
    def _distance(a: Point, b: Point) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    @staticmethod
    def _slot_center(slot: List[float]) -> Point:
        return (0.5 * (slot[0] + slot[1]), 0.5 * (slot[2] + slot[3]))

    def _signature_for(self, obs: Dict[str, Any]) -> Tuple[float, ...]:
        slot = obs.get("target_slot") or [0.0, 0.0, 0.0, 0.0]
        expected = self.map_data.get("expected_orientation") if self.map_data else None
        orient_code = 1.0 if expected == "rear_in" else 0.0
        return tuple(round(float(v), 3) for v in slot) + (orient_code,)

    def _corridor_x(self, start_x: float, target_x: float) -> float:
        if not self.map_extent:
            return start_x
        xmin, xmax, _, _ = self.map_extent
        left_lane = xmin + 4.0
        return self._clamp(min(start_x, left_lane), xmin + 2.0, min(target_x - 3.5, xmax - 2.0))

    def _build_axis_path(self, start: Point, target_slot: List[float], expected: str) -> None:
        """Create a collision-aware aisle path and a final parking maneuver."""

        cx, cy = self._slot_center(target_slot)
        xmin, xmax, ymin, ymax = target_slot
        lane_x = self._corridor_x(start[0], cx)
        slot_height = ymax - ymin

        self.final_yaw = -math.pi / 2.0 if expected == "rear_in" else math.pi / 2.0

        if expected == "rear_in":
            # Stage 3 requires rear-in orientation. Bottom-row targets are
            # safer as a forward downward park from the upper aisle; middle/top
            # targets use a reverse upward final maneuver from below the slot.
            if self.map_extent:
                map_mid_x = 0.5 * (self.map_extent[0] + self.map_extent[1])
                side_offset = -5.0 if cx < map_mid_x else 5.0
                side_x = self._clamp(cx + side_offset, self.map_extent[0] + 3.0, self.map_extent[1] - 3.0)
            else:
                map_mid_x = cx
                side_x = cx - 5.0
            if cy < 14.0:
                self.final_gear = "D"
                approach_y = ymax + max(5.8, 1.35 * slot_height)
                lane_y = approach_y + max(1.8, 0.42 * slot_height)
                pre_y = approach_y + max(2.7, 0.65 * slot_height)
            else:
                self.final_gear = "R"
                approach_y = ymin - max(8.8, 2.15 * slot_height)
                lane_y = max(start[1] + 2.0, approach_y)
                pre_y = min(ymin - 1.8, approach_y + max(6.5, 1.55 * slot_height))
                if cy > 35.0 and cx < map_mid_x:
                    pre_y = min(ymin - 5.1, approach_y + max(3.0, 0.75 * slot_height))
            raw_approach = [
                start,
                (lane_x, start[1]),
                (lane_x, lane_y),
                (side_x, lane_y),
                (side_x, pre_y),
                (cx, pre_y),
                (cx, approach_y),
            ]
        else:
            self.final_yaw = math.pi / 2.0
            if cy < 14.0:
                # The first row is safest from the upper aisle. Stop above it,
                # stay nose-up, and reverse down into the target slot.
                self.final_gear = "R"
                approach_y = ymax + max(5.6, 1.25 * slot_height)
                lane_y = ymax + max(2.4, 0.55 * slot_height)
            else:
                # Middle/top rows: approach from below and drive forward in.
                self.final_gear = "D"
                approach_y = ymin - max(6.0, 1.45 * slot_height)
                lane_y = approach_y - max(5.4, 1.3 * slot_height)

            turn_x = self._clamp(cx - 5.0, lane_x + 3.0, cx)

            raw_approach = [
                start,
                (lane_x, start[1]),
                (lane_x, lane_y),
                (turn_x, lane_y),
                (cx, lane_y),
                (cx, approach_y),
            ]

        parking_target = (cx, cy)
        self.approach_path = self._densify_points(self._dedupe_points(raw_approach), spacing=0.8)
        self.parking_path = self._densify_points(
            self._dedupe_points([self.approach_path[-1], parking_target]),
            spacing=0.45,
        )
        self.waypoints = list(self.approach_path)
        self.phase = "approach"
        self.last_target_idx = 0
        print(
            "[algo] planned",
            expected or "front_in",
            "final_gear",
            self.final_gear,
            "approach_points",
            len(self.approach_path),
            "parking_points",
            len(self.parking_path),
        )

    @classmethod
    def _dedupe_points(cls, points: List[Point]) -> List[Point]:
        cleaned: List[Point] = []
        for point in points:
            if not cleaned or cls._distance(cleaned[-1], point) > 0.25:
                cleaned.append((float(point[0]), float(point[1])))
        return cleaned

    @classmethod
    def _densify_points(cls, points: List[Point], spacing: float) -> List[Point]:
        if len(points) < 2:
            return list(points)
        dense: List[Point] = [points[0]]
        for start, end in zip(points, points[1:]):
            segment_len = cls._distance(start, end)
            steps = max(1, int(math.ceil(segment_len / max(spacing, 0.1))))
            for idx in range(1, steps + 1):
                ratio = idx / steps
                dense.append((
                    start[0] + (end[0] - start[0]) * ratio,
                    start[1] + (end[1] - start[1]) * ratio,
                ))
        return dense

    def compute_path(self, obs: Dict[str, Any]) -> None:
        """Prepare a path whenever the simulator sends a new target slot."""

        signature = self._signature_for(obs)
        if signature == self.path_signature and self.waypoints:
            return

        state = obs.get("state", {})
        start = (float(state.get("x", 0.0)), float(state.get("y", 0.0)))
        slot = [float(v) for v in (obs.get("target_slot") or [])]
        if len(slot) != 4:
            self.approach_path = [start]
            self.parking_path = [start]
            self.waypoints = [start]
            self.path_signature = signature
            return

        expected = "front_in"
        if self.map_data:
            expected = str(self.map_data.get("expected_orientation") or "front_in")
        self._build_axis_path(start, slot, expected)
        self.path_signature = signature

    def _advance_waypoint(self, pos: Point, lookahead: float) -> None:
        while self.last_target_idx < len(self.waypoints) - 1:
            if self._distance(pos, self.waypoints[self.last_target_idx]) > lookahead * 0.7:
                break
            self.last_target_idx += 1

    def _lookahead_point(self, pos: Point, lookahead: float) -> Point:
        self._advance_waypoint(pos, lookahead)
        for idx in range(self.last_target_idx, len(self.waypoints)):
            if self._distance(pos, self.waypoints[idx]) >= lookahead:
                self.last_target_idx = idx
                return self.waypoints[idx]
        return self.waypoints[-1]

    def _advance_segment(self, pos: Point) -> None:
        while self.last_target_idx < len(self.waypoints) - 2:
            start = self.waypoints[self.last_target_idx]
            end = self.waypoints[self.last_target_idx + 1]
            seg_x = end[0] - start[0]
            seg_y = end[1] - start[1]
            seg_len = max(1e-6, math.hypot(seg_x, seg_y))
            rel_x = pos[0] - start[0]
            rel_y = pos[1] - start[1]
            progress = (rel_x * seg_x + rel_y * seg_y) / seg_len
            if progress < seg_len - 0.18 and self._distance(pos, end) > 0.55:
                break
            self.last_target_idx += 1

    def _passed_path_end(self, pos: Point, allow_far: bool = False) -> bool:
        if len(self.waypoints) < 2:
            return True
        start = self.waypoints[-2]
        end = self.waypoints[-1]
        if not allow_far and self._distance(pos, end) > max(5.0, self._distance(start, end) + 2.0):
            return False
        seg_x = end[0] - start[0]
        seg_y = end[1] - start[1]
        seg_len = max(1e-6, math.hypot(seg_x, seg_y))
        rel_x = pos[0] - start[0]
        rel_y = pos[1] - start[1]
        progress = (rel_x * seg_x + rel_y * seg_y) / seg_len
        return progress >= seg_len - 0.05

    def _stanley_steer(
        self,
        pos: Point,
        yaw: float,
        speed: float,
        gear: str,
        max_steer: float,
    ) -> float:
        self._advance_segment(pos)
        if len(self.waypoints) < 2:
            return 0.0
        idx = min(self.last_target_idx, len(self.waypoints) - 2)
        start = self.waypoints[idx]
        end = self.waypoints[idx + 1]
        seg_x = end[0] - start[0]
        seg_y = end[1] - start[1]
        seg_len = max(1e-6, math.hypot(seg_x, seg_y))
        path_yaw = math.atan2(seg_y, seg_x)
        effective_yaw = yaw if gear == "D" else self._wrap_to_pi(yaw + math.pi)
        heading_error = self._wrap_to_pi(path_yaw - effective_yaw)
        unit_x = seg_x / seg_len
        unit_y = seg_y / seg_len
        rel_x = pos[0] - start[0]
        rel_y = pos[1] - start[1]
        crosstrack = unit_x * rel_y - unit_y * rel_x
        gain = 0.85 if self.phase == "approach" else 0.55
        correction = -math.atan2(gain * crosstrack, abs(speed) + 0.8)
        steer = heading_error + correction
        if gear == "R":
            steer = -steer
        return self._clamp(steer, -max_steer, max_steer)

    def _pure_pursuit_steer(
        self,
        pos: Point,
        yaw: float,
        target: Point,
        wheelbase: float,
        gear: str,
        max_steer: float,
    ) -> float:
        dx = target[0] - pos[0]
        dy = target[1] - pos[1]
        local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
        distance = max(0.5, math.hypot(local_x, local_y))
        alpha = math.atan2(local_y, local_x)
        direction = -1.0 if gear == "R" else 1.0
        steer = math.atan2(direction * 2.0 * wheelbase * math.sin(alpha), distance)
        return self._clamp(steer, -max_steer, max_steer)

    def _longitudinal_control(self, v: float, target_speed: float, gear: str) -> Tuple[float, float]:
        current_along_gear = -v if gear == "R" else v
        speed_error = target_speed - current_along_gear
        if abs(speed_error) < 0.08:
            return 0.0, 0.0
        if speed_error > 0.0:
            return self._clamp(0.35 * speed_error + 0.08, 0.0, 0.75), 0.0
        return 0.0, self._clamp(-0.45 * speed_error + 0.08, 0.0, 1.0)

    def _stop_command(self, v: float, gear: str) -> Dict[str, Any]:
        brake = 1.0 if abs(v) > 0.05 else 0.45
        return {"steer": 0.0, "accel": 0.0, "brake": brake, "gear": gear}

    def compute_control(self, obs: Dict[str, Any]) -> Dict[str, float]:
        """경로를 따라가기 위한 조향/가감속 명령을 산출합니다."""

        self.compute_path(obs)
        state = obs.get("state", {})
        pos = (float(state.get("x", 0.0)), float(state.get("y", 0.0)))
        yaw = float(state.get("yaw", 0.0))
        v = float(state.get("v", 0.0))
        limits = obs.get("limits", {})
        wheelbase = float(limits.get("L", 2.6))
        max_steer = float(limits.get("maxSteer", math.radians(35.0)))

        if not self.waypoints:
            return self._stop_command(v, "D")

        active_goal = self.waypoints[-1]
        goal_distance = self._distance(pos, active_goal)

        approach_passed = self._passed_path_end(pos, allow_far=self.final_yaw > 0.0)
        reached_approach = goal_distance < 0.85 or (
            goal_distance < 1.55 and approach_passed
        ) or (self.final_yaw > 0.0 and approach_passed)
        if self.phase == "approach" and reached_approach:
            if abs(v) > 0.25:
                return self._stop_command(v, "D")
            self.phase = "parking"
            self.waypoints = list(self.parking_path)
            self.last_target_idx = 0

        gear = "D" if self.phase == "approach" else self.final_gear
        active_goal = self.waypoints[-1]
        goal_distance = self._distance(pos, active_goal)

        if self.phase == "parking":
            yaw_error = abs(self._wrap_to_pi(self.final_yaw - yaw))
            parking_done = goal_distance < 0.90 or (
                goal_distance < 1.50 and self._passed_path_end(pos)
            )
            if parking_done and yaw_error < 0.75:
                return self._stop_command(v, gear)
            target_speed = self._clamp(0.35 + 0.32 * goal_distance, 0.35, 1.05)
            lookahead = self._clamp(0.9 + 0.25 * abs(v), 0.8, 1.6)
        else:
            next_corner = False
            if self.last_target_idx < len(self.waypoints) - 3:
                a = self.waypoints[self.last_target_idx]
                b = self.waypoints[self.last_target_idx + 1]
                c = self.waypoints[self.last_target_idx + 2]
                yaw1 = math.atan2(b[1] - a[1], b[0] - a[0])
                yaw2 = math.atan2(c[1] - b[1], c[0] - b[0])
                next_corner = abs(self._wrap_to_pi(yaw2 - yaw1)) > 0.35 and self._distance(pos, b) < 2.5
            if next_corner:
                target_speed = 0.55
            else:
                target_speed = self._clamp(0.45 + 0.09 * goal_distance, 0.55, 1.15)
            lookahead = self._clamp(0.95 + 0.22 * abs(v), 0.9, 1.8)

        steer = self._stanley_steer(pos, yaw, v, gear, max_steer)

        if self.phase == "parking":
            yaw_error = self._wrap_to_pi(self.final_yaw - yaw)
            if goal_distance < 1.4:
                steer += self._clamp(0.55 * yaw_error, -0.25, 0.25)
                steer = self._clamp(steer, -max_steer, max_steer)

        accel, brake = self._longitudinal_control(v, target_speed, gear)
        return {"steer": steer, "accel": accel, "brake": brake, "gear": gear}


# 전역 planner 인스턴스 (통신 모듈이 이 객체를 사용합니다.)
planner = PlannerSkeleton()


def handle_map_payload(map_payload: Dict[str, Any]) -> None:
    """통신 모듈에서 맵 패킷을 받을 때 호출됩니다."""

    planner.set_map(map_payload)


def planner_step(obs: Dict[str, Any]) -> Dict[str, Any]:
    """통신 모듈에서 매 스텝 호출하여 명령을 생성합니다."""

    try:
        return planner.compute_control(obs)
    except Exception as exc:
        print(f"[algo] planner_step error: {exc}")
        return {"steer": 0.0, "accel": 0.0, "brake": 0.5, "gear": "D"}
