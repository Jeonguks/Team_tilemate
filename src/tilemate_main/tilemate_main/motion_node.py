#!/usr/bin/env python3
import time
import threading
import traceback
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, Float64

import DR_init
from tilemate_main.robot_config import RobotConfig
from tilemate_main.scraper_task_lib import ScraperTask


class _GripperClient:
    def __init__(self, node: Node):
        self._node = node
        self._pub = node.create_publisher(Float64, "/gripper/width_m", 10)

    def set_width(self, width_m: float):
        msg = Float64()
        msg.data = float(width_m)
        self._pub.publish(msg)


class MotionNode(Node):
    def __init__(self, cfg: RobotConfig, boot_node=None):
        # DR_init 세팅은 main()에서 이미 완료됨
        super().__init__("motion_node", namespace=cfg.robot_id)
        self.cfg = cfg
        self._boot_node = boot_node
        self.get_logger().info("[DBG] MotionNode 초기화 시작")

        self._pause = False
        self._stop_soft = False
        self._run_token = 0
        self._run_event = threading.Event()
        self._lock = threading.Lock()

        self.create_subscription(Int32, "/task/run_once", self._cb_run_once, 10)
        self.create_subscription(Bool,  "/task/pause",    self._cb_pause,    10)
        self.create_subscription(Bool,  "/task/stop_soft",self._cb_stop_soft,10)

        self.gripper = _GripperClient(self)

        try:
            self.task = ScraperTask(self.gripper, cfg)
        except TypeError:
            self.task = ScraperTask(self, self.gripper, cfg)

        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

        self.task.initialize_robot()
        self.get_logger().info("MotionNode ready")

    def _cb_run_once(self, msg: Int32):
        with self._lock:
            if msg.data <= self._run_token:
                return
            self._run_token = int(msg.data)
        self._run_event.set()

    def _cb_pause(self, msg: Bool):
        self._pause = bool(msg.data)

    def _cb_stop_soft(self, msg: Bool):
        self._stop_soft = bool(msg.data)

    def _wait_if_paused(self):
        while rclpy.ok() and self._pause and not self._stop_soft:
            time.sleep(0.05)

    def _worker_loop(self):
        while rclpy.ok():
            self._run_event.wait(timeout=0.1)
            if not self._run_event.is_set():
                continue
            self._run_event.clear()
            if self._stop_soft:
                continue
            self._wait_if_paused()
            if self._stop_soft:
                continue
            try:
                self.get_logger().info("[TASK] run_once start")
                self.task.run_once()
                self.get_logger().info("[TASK] run_once done")
            except Exception as e:
                self.get_logger().error(f"[TASK] run_once exception: {e}")
                self.get_logger().error(traceback.format_exc())


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        cfg = RobotConfig()

        # ✅ move_scraper.py와 완전히 동일한 패턴:
        # 1. 클래스 밖(함수 스코프)에서 raw 노드 생성 → name mangling 없음
        _boot_node = rclpy.create_node("motion_node", namespace=cfg.robot_id)

        # 2. DR_init 세팅
        DR_init.__dsr__id    = cfg.robot_id
        DR_init.__dsr__model = cfg.robot_model
        DR_init.__dsr__node  = _boot_node

        # 3. DSR_ROBOT2 import (g_node가 유효한 상태)
        import DSR_ROBOT2  # noqa: F401
        print("[DBG] DSR_ROBOT2 import OK")

        node = MotionNode(cfg, boot_node=_boot_node)
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"[ERROR] {e}")
        traceback.print_exc()
    finally:
        if node is not None:
            try:
                node.destroy_node()
                if node._boot_node:
                    node._boot_node.destroy_node()
            except Exception:
                pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()