#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Bool

class TaskManagerNode(Node):
    """
    단일 최상단 상태관리/오케스트레이션 노드.

    start 흐름:
      scraper/run_once(token)
        -> scraper/status done:token
        -> tile/run_once(token)
        -> tile/status done:token

    /robot/step (UI/모니터링용 단일 정의)
      0: READY
      1: GLUE_PICK
      2: GLUE_SPREAD
      3: TILE_PICK
      4: TILE_PLACE
      5: FINISHED (정상 완료)
    """
    STEP_READY        = 0
    STEP_GLUE_PICK    = 1
    STEP_GLUE_SPREAD  = 2
    STEP_TILE_PICK    = 3
    STEP_TILE_PLACE   = 4
    STEP_FINISHED     = 5

    def __init__(self):
        super().__init__("task_manager_node")

        # State Machine
        self._busy = False
        self._phase = "IDLE"        # IDLE / SCRAPER / TILE
        self._active_token = 0      # 현재 사이클 token

        # publishers
        self.pub_scraper = self.create_publisher(Int32, "/scraper/run_once", 10)
        self.pub_tile    = self.create_publisher(Int32, "/tile/run_once", 10)

        self.pub_pause     = self.create_publisher(Bool, "/task/pause", 10)
        self.pub_stop_soft = self.create_publisher(Bool, "/task/stop_soft", 10)

        self.pub_step = self.create_publisher(Int32, "/robot/step", 10)
        self._set_step(self.STEP_READY)

        # subscribers
        self.create_subscription(String, "/robot/command", self._cb_cmd, 10)
        self.create_subscription(String, "/scraper/status", self._cb_scraper_status, 10)
        self.create_subscription(String, "/tile/status", self._cb_tile_status, 10)

        self.get_logger().info("TaskManagerNode ready")

    # -----------------
    # publish helpers
    # -----------------
    def _set_step(self, step: int):
        m = Int32()
        m.data = int(step)
        self.pub_step.publish(m)
        self.get_logger().info(f"[TASK] /robot/step={m.data}")

    def _pub_bool(self, pub, v: bool):
        m = Bool()
        m.data = bool(v)
        pub.publish(m)

    def _next_token(self) -> int:
        self._active_token += 1
        return self._active_token

    # -----------------
    # orchestrate actions
    # -----------------
    def _start_cycle(self):
        """새 사이클 시작: scraper -> tile"""
        self._busy = True
        self._phase = "SCRAPER"

        tok = self._next_token()

        # 1) glue pick
        self._set_step(self.STEP_GLUE_PICK)

        m = Int32()
        m.data = tok
        self.pub_scraper.publish(m)
        self.get_logger().info(f"[TASK] start SCRAPER token={tok}")

        # 2) glue spread (세부 단계가 필요하면 scraper가 status를 더 잘게 내도록 확장)
        self._set_step(self.STEP_GLUE_SPREAD)

    def _start_tile(self, tok: int):
        """scraper 완료 후 tile 시작 (동일 token 사용)"""
        self._phase = "TILE"

        # 3) tile pick
        self._set_step(self.STEP_TILE_PICK)

        m = Int32()
        m.data = tok
        self.pub_tile.publish(m)
        self.get_logger().info(f"[TASK] start TILE token={tok}")

        # 4) tile place
        self._set_step(self.STEP_TILE_PLACE)

    def _finish_ok(self):
        """정상 완료"""
        self._busy = False
        self._phase = "IDLE"
        self._set_step(self.STEP_FINISHED)

    def _finish_abort(self):
        """중단/리셋/에러 등 비정상 종료(사용자 중단 포함)"""
        self._busy = False
        self._phase = "IDLE"
        self._set_step(self.STEP_READY)

    # -----------------
    # status parsing
    # -----------------
    def _parse_status(self, s: str):
        # "done:3" / "error:3:msg"
        parts = s.strip().split(":", 2)
        if len(parts) < 2:
            return None, None, None
        state = parts[0].strip().lower()
        try:
            tok = int(parts[1])
        except ValueError:
            return None, None, None
        msg = parts[2] if len(parts) == 3 else ""
        return state, tok, msg

    def _is_active_token(self, tok: int) -> bool:
        # stop/reset 이후 늦게 들어오는 status 무시용
        return self._busy and (tok == self._active_token)

    # -----------------
    # callbacks
    # -----------------
    def _cb_cmd(self, msg: String):
        cmd = msg.data.strip().lower()

        if cmd in ("start", "once"):
            if self._busy:
                self.get_logger().warn("[TASK] start ignored (busy)")
                return
            self._pub_bool(self.pub_stop_soft, False)
            self._pub_bool(self.pub_pause, False)
            self._start_cycle()

        elif cmd == "pause":
            self.get_logger().warn("[TASK] pause True")
            self._pub_bool(self.pub_pause, True)

        elif cmd == "resume":
            self.get_logger().info("[TASK] pause False")
            self._pub_bool(self.pub_pause, False)

        elif cmd == "stop":
            self.get_logger().warn("[TASK] stop_soft True + abort")
            self._pub_bool(self.pub_pause, False)
            self._pub_bool(self.pub_stop_soft, True)
            self._finish_abort()

        elif cmd == "reset":
            self.get_logger().warn("[TASK] reset -> stop_soft True + abort")
            self._pub_bool(self.pub_pause, False)
            self._pub_bool(self.pub_stop_soft, True)
            self._finish_abort()

        else:
            self.get_logger().warn(f"[TASK] unknown command: {cmd}")

    def _cb_scraper_status(self, msg: String):
        state, tok, emsg = self._parse_status(msg.data)
        if state is None:
            return

        if self._phase != "SCRAPER":
            return
        if not self._is_active_token(tok):
            return

        if state == "done":
            self.get_logger().info(f"[TASK] scraper done token={tok} -> start tile")
            self._start_tile(tok)

        elif state == "error":
            self.get_logger().error(f"[TASK] scraper error token={tok}: {emsg}")
            self._finish_abort()

    def _cb_tile_status(self, msg: String):
        state, tok, emsg = self._parse_status(msg.data)
        if state is None:
            return

        if self._phase != "TILE":
            return
        if not self._is_active_token(tok):
            return

        if state == "done":
            self.get_logger().info(f"[TASK] tile done token={tok} -> FINISHED")
            self._finish_ok()

        elif state == "error":
            self.get_logger().error(f"[TASK] tile error token={tok}: {emsg}")
            self._finish_abort()


def main(args=None):
    rclpy.init(args=args)
    node = TaskManagerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == "__main__":
    main()