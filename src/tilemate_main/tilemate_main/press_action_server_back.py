#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import json
import math
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import rclpy
import DR_init

from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from tilemate_msgs.action import Press
from tilemate_main.robot_config import RobotConfig, GripperConfig


class PressActionServer(Node):
    def __init__(self, robot_cfg: RobotConfig, gripper_cfg: GripperConfig):
        super().__init__("press_action_server", namespace=robot_cfg.robot_id)

        self.robot_cfg = robot_cfg
        self.gripper_cfg = gripper_cfg
        self.cb_group = ReentrantCallbackGroup()

        # --------------------------------------------------
        # parameters
        # --------------------------------------------------
        self.declare_parameter("default_result_json_path", "")
        self.declare_parameter("default_press_threshold_mm", 3.0)
        self.declare_parameter("default_approach_offset_mm", 30.0)
        self.declare_parameter("default_press_overshoot_mm", 2.0)

        self.declare_parameter("move_vel", 30.0)
        self.declare_parameter("move_acc", 30.0)
        self.declare_parameter("press_vel", 10.0)
        self.declare_parameter("press_acc", 10.0)

        self.declare_parameter("compliance_stx_x", 5000.0)
        self.declare_parameter("compliance_stx_y", 500.0)
        self.declare_parameter("compliance_stx_z", 5000.0)
        self.declare_parameter("compliance_stx_rx", 300.0)
        self.declare_parameter("compliance_stx_ry", 300.0)
        self.declare_parameter("compliance_stx_rz", 300.0)

        self.declare_parameter("desired_force_n", 15.0)
        self.declare_parameter("force_check_max", 10.0)
        self.declare_parameter("contact_timeout_sec", 3.0)

        # Y 방향 부호
        # -1 이면 -Y 방향으로 벽 쪽 접근
        # +1 이면 +Y 방향으로 벽 쪽 접근
        self.declare_parameter("wall_push_direction_sign", -1)

        # --------------------------------------------------
        # cached params
        # --------------------------------------------------
        self.default_result_json_path = self.get_parameter(
            "default_result_json_path"
        ).value
        self.default_press_threshold_mm = float(
            self.get_parameter("default_press_threshold_mm").value
        )
        self.default_approach_offset_mm = float(
            self.get_parameter("default_approach_offset_mm").value
        )
        self.default_press_overshoot_mm = float(
            self.get_parameter("default_press_overshoot_mm").value
        )

        self.move_vel = float(self.get_parameter("move_vel").value)
        self.move_acc = float(self.get_parameter("move_acc").value)
        self.press_vel = float(self.get_parameter("press_vel").value)
        self.press_acc = float(self.get_parameter("press_acc").value)

        self.compliance_stx = [
            float(self.get_parameter("compliance_stx_x").value),
            float(self.get_parameter("compliance_stx_y").value),
            float(self.get_parameter("compliance_stx_z").value),
            float(self.get_parameter("compliance_stx_rx").value),
            float(self.get_parameter("compliance_stx_ry").value),
            float(self.get_parameter("compliance_stx_rz").value),
        ]

        self.desired_force_n = float(self.get_parameter("desired_force_n").value)
        self.force_check_max = float(self.get_parameter("force_check_max").value)
        self.contact_timeout_sec = float(
            self.get_parameter("contact_timeout_sec").value
        )
        self.wall_push_direction_sign = int(
            self.get_parameter("wall_push_direction_sign").value
        )

        if self.wall_push_direction_sign not in (-1, 1):
            self.get_logger().warn(
                "wall_push_direction_sign must be -1 or +1. fallback -> -1"
            )
            self.wall_push_direction_sign = -1

        # --------------------------------------------------
        # action server
        # --------------------------------------------------
        self._action_server = ActionServer(
            self,
            Press,
            "press",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group,
        )

        self.get_logger().info("[PRESS] initialize Done!")

    # ------------------------------------------------------------------
    # action callbacks
    # ------------------------------------------------------------------
    def goal_callback(self, goal_request: Press.Goal):
        self.get_logger().info("[PRESS] goal request accepted")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("[PRESS] cancel request accepted")
        return CancelResponse.ACCEPT

    # ------------------------------------------------------------------
    # helpers
    # ------------------------------------------------------------------
    def _publish_feedback(
        self,
        goal_handle,
        step: int,
        progress: float,
        state: str,
    ):
        fb = Press.Feedback()
        fb.step = int(step)
        fb.progress = float(progress)
        fb.state = str(state)
        goal_handle.publish_feedback(fb)
        self.get_logger().info(
            f"[PRESS][FB] step={step} {progress:.0f}% state={state}"
        )

    def _result(self, success: bool, message: str) -> Press.Result:
        res = Press.Result()
        res.success = bool(success)
        res.message = str(message)
        return res

    def _load_json(self, path_str: str) -> Dict[str, Any]:
        path = Path(path_str)
        if not path.exists():
            raise FileNotFoundError(f"json not found: {path}")
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)

    def _resolve_goal_params(
        self, goal: Press.Goal
    ) -> Tuple[str, float, float, float]:
        result_json_path = (
            goal.result_json_path.strip()
            if hasattr(goal, "result_json_path")
            else ""
        )
        if not result_json_path:
            result_json_path = self.default_result_json_path

        if not result_json_path:
            raise ValueError("result_json_path is empty")

        press_threshold_mm = float(
            goal.press_threshold_mm
            if hasattr(goal, "press_threshold_mm") and goal.press_threshold_mm > 0.0
            else self.default_press_threshold_mm
        )
        approach_offset_mm = float(
            goal.approach_offset_mm
            if hasattr(goal, "approach_offset_mm") and goal.approach_offset_mm > 0.0
            else self.default_approach_offset_mm
        )
        press_overshoot_mm = float(
            goal.press_overshoot_mm
            if hasattr(goal, "press_overshoot_mm") and goal.press_overshoot_mm > 0.0
            else self.default_press_overshoot_mm
        )

        return (
            result_json_path,
            press_threshold_mm,
            approach_offset_mm,
            press_overshoot_mm,
        )

    def _extract_press_targets(
        self,
        data: Dict[str, Any],
        press_threshold_mm: float,
    ) -> List[Dict[str, Any]]:
        """
        우선순위:
        1) data['press_targets']
        2) data['tiles'] 중 press_candidate=True
        3) data['tiles'] 중 protrusion_mm >= threshold
        """
        targets: List[Dict[str, Any]] = []

        if isinstance(data.get("press_targets"), list):
            for item in data["press_targets"]:
                if isinstance(item, dict):
                    targets.append(item)

        if targets:
            return targets

        tiles = data.get("tiles", [])
        if not isinstance(tiles, list):
            return []

        for tile in tiles:
            if not isinstance(tile, dict):
                continue

            if tile.get("press_candidate", False):
                targets.append(tile)
                continue

            protrusion_mm = float(tile.get("protrusion_mm", 0.0) or 0.0)
            if protrusion_mm >= press_threshold_mm:
                tile = dict(tile)
                tile["press_candidate"] = True
                targets.append(tile)

        return targets

    def _get_tile_center_mm(self, tile: Dict[str, Any]) -> Tuple[float, float, float]:
        center = tile.get("base_center_mm", None)
        if center is None or not isinstance(center, (list, tuple)) or len(center) < 3:
            raise ValueError(
                f"tile '{tile.get('name', '?')}' has no valid base_center_mm"
            )
        return float(center[0]), float(center[1]), float(center[2])

    def _build_pre_contact_pose(
        self,
        tile: Dict[str, Any],
        approach_offset_mm: float,
    ) -> Tuple[List[float], List[float]]:
        """
        요구사항:
        - 중심점만 사용
        - 회전은 [90,90,90]
        - 접촉 직전 좌표는 Y축 방향으로만 offset
        """
        cx, cy, cz = self._get_tile_center_mm(tile)

        y_pre = cy - self.wall_push_direction_sign * approach_offset_mm

        pre_contact_pose = [cx, y_pre, cz, 90.0, 90.0, 90.0]
        contact_pose = [cx, cy, cz, 90.0, 90.0, 90.0]
        return pre_contact_pose, contact_pose

    def _safe_release_force(self):
        try:
            release_force()
        except Exception:
            pass

    def _safe_release_compliance(self):
        try:
            release_compliance_ctrl()
        except Exception:
            pass

    def _enable_y_only_compliance(self):
        """
        Y축만 낮은 강성으로 둬서 Y 방향 순응
        """
        task_compliance_ctrl(stx=self.compliance_stx)

    def _set_desired_force_y(self):
        """
        Y축으로만 힘 제어
        sign=-1 -> -Y 방향으로 누름
        sign=+1 -> +Y 방향으로 누름
        """
        fy = float(self.wall_push_direction_sign) * float(self.desired_force_n)
        set_desired_force(
            fd=[0.0, fy, 0.0, 0.0, 0.0, 0.0],
            dir=[0, 1, 0, 0, 0, 0],
            mod=DR_FC_MOD_REL,
        )

    def _wait_contact_y(self, timeout_sec: float):
        t0 = time.time()
        while True:
            ok = check_force_condition(DR_AXIS_Y, max=self.force_check_max)
            if ok:
                return
            if (time.time() - t0) > timeout_sec:
                raise TimeoutError("force contact timeout on Y axis")
            time.sleep(0.05)

    def _press_along_y_relative(self, press_mm: float):
        dy = float(self.wall_push_direction_sign) * float(press_mm)
        movel(
            [0.0, dy, 0.0, 0.0, 0.0, 0.0],
            vel=self.press_vel,
            acc=self.press_acc,
            mod=DR_MV_MOD_REL,
        )

    def _move_pose_abs(self, pose: List[float], vel: Optional[float] = None, acc: Optional[float] = None):
        movel(
            pose,
            vel=self.move_vel if vel is None else vel,
            acc=self.move_acc if acc is None else acc,
            ref=DR_BASE,
        )

    def _press_one_tile(
        self,
        tile: Dict[str, Any],
        approach_offset_mm: float,
        press_overshoot_mm: float,
    ) -> Dict[str, Any]:
        """
        단일 타일 press 수행
        """
        tile_name = str(tile.get("name", "unknown"))
        protrusion_mm = float(tile.get("protrusion_mm", 0.0) or 0.0)

        pre_contact_pose, contact_pose = self._build_pre_contact_pose(
            tile, approach_offset_mm
        )

        self.get_logger().info(
            f"[PRESS] target={tile_name} "
            f"protrusion_mm={protrusion_mm:.2f} "
            f"pre_contact_pose={pre_contact_pose} "
            f"contact_pose={contact_pose}"
        )

        press_depth_mm = max(0.0, protrusion_mm) + float(press_overshoot_mm)

        set_ref_coord(DR_BASE)

        # 1) 접촉 직전 이동
        self._move_pose_abs(pre_contact_pose)

        current_before = None
        try:
            current_before = get_current_posx()[0]
        except Exception:
            pass

        try:
            # 2) Y축 순응만 활성화
            self._enable_y_only_compliance()

            # 3) Y축 force control
            self._set_desired_force_y()

            # 4) 접촉 감지
            self._wait_contact_y(self.contact_timeout_sec)

            # 5) force release 후 추가 press
            self._safe_release_force()
            self._press_along_y_relative(press_depth_mm)

            result = {
                "tile_name": tile_name,
                "success": True,
                "message": "pressed",
                "pre_contact_pose": pre_contact_pose,
                "contact_pose": contact_pose,
                "press_depth_mm": press_depth_mm,
            }
            return result

        finally:
            self._safe_release_force()
            self._safe_release_compliance()

            # 6) 접촉 직전 위치로 복귀
            try:
                self._move_pose_abs(pre_contact_pose)
            except Exception as e:
                self.get_logger().warn(
                    f"[PRESS] failed to return pre_contact pose for {tile_name}: {e}"
                )

            if current_before is not None:
                self.get_logger().info(
                    f"[PRESS] current_before={current_before}"
                )

    # ------------------------------------------------------------------
    # execute
    # ------------------------------------------------------------------
    def execute_callback(self, goal_handle):
        self.get_logger().info("[PRESS] start press action")

        try:
            self._publish_feedback(goal_handle, 1, 10.0, "load_json")

            result_json_path, press_threshold_mm, approach_offset_mm, press_overshoot_mm = (
                self._resolve_goal_params(goal_handle.request)
            )

            data = self._load_json(result_json_path)

            self.get_logger().info(
                f"[PRESS] load json: {result_json_path}"
            )

            self._publish_feedback(goal_handle, 2, 20.0, "parse_targets")

            targets = self._extract_press_targets(data, press_threshold_mm)

            self.get_logger().info(
                f"[PRESS] parsed press targets: {len(targets)}"
            )

            if not targets:
                goal_handle.succeed()
                return self._result(True, "no press target")

            self._publish_feedback(goal_handle, 3, 30.0, "press_start")

            pressed_results: List[Dict[str, Any]] = []
            total = len(targets)

            for i, tile in enumerate(targets):
                if goal_handle.is_cancel_requested:
                    self.get_logger().warn("[PRESS] cancel requested")
                    goal_handle.canceled()
                    return self._result(False, "press canceled")

                tile_name = str(tile.get("name", f"tile_{i}"))
                self.get_logger().info(
                    f"[PRESS] ({i+1}/{total}) start tile={tile_name}"
                )

                try:
                    one_result = self._press_one_tile(
                        tile=tile,
                        approach_offset_mm=approach_offset_mm,
                        press_overshoot_mm=press_overshoot_mm,
                    )
                    pressed_results.append(one_result)

                    progress = 30.0 + (60.0 * float(i + 1) / float(total))
                    self._publish_feedback(
                        goal_handle,
                        4,
                        progress,
                        f"pressed_{tile_name}",
                    )

                except Exception as e:
                    self.get_logger().error(
                        f"[PRESS] failed on tile={tile_name}: {e}"
                    )
                    pressed_results.append(
                        {
                            "tile_name": tile_name,
                            "success": False,
                            "message": str(e),
                        }
                    )

            # 결과 저장
            self._publish_feedback(goal_handle, 5, 95.0, "save_result")

            save_path = Path(result_json_path).with_name(
                Path(result_json_path).stem + "_pressed.json"
            )

            out_data = dict(data)
            out_data["press_execution"] = {
                "success_count": sum(1 for r in pressed_results if r.get("success")),
                "fail_count": sum(1 for r in pressed_results if not r.get("success")),
                "results": pressed_results,
            }

            with open(save_path, "w", encoding="utf-8") as f:
                json.dump(out_data, f, ensure_ascii=False, indent=2)

            self.get_logger().info(
                f"[PRESS] pressed json saved: {save_path}"
            )

            self._publish_feedback(goal_handle, 6, 100.0, "done")

            success_count = sum(1 for r in pressed_results if r.get("success"))
            fail_count = sum(1 for r in pressed_results if not r.get("success"))

            msg = (
                f"press done: total={len(pressed_results)}, "
                f"success={success_count}, fail={fail_count}, "
                f"saved={save_path}"
            )

            goal_handle.succeed()
            return self._result(True, msg)

        except Exception as e:
            self.get_logger().error(f"[PRESS] execute failed: {e}")
            goal_handle.abort()
            return self._result(False, str(e))


def main(args=None):
    rclpy.init(args=args)

    # 프로젝트 설정 객체 사용
    robot_cfg = RobotConfig()
    gripper_cfg = GripperConfig()

    node = PressActionServer(robot_cfg, gripper_cfg)

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("[PRESS] interrupted by user")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()