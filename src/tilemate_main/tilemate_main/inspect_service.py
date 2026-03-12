#!/usr/bin/env python3
import json
import time
import requests
import rclpy
import DR_init

from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from tilemate_msgs.srv import Inspect
from tilemate_main.robot_config import RobotConfig, GripperConfig
from tilemate_main.wall_tile_inspection_engine import WallTileInspectionEngine


class InspectService(Node):

    def __init__(self, robot_cfg: RobotConfig, gripper_cfg: GripperConfig, boot_node: Node):
        super().__init__("inspect_service", namespace=robot_cfg.robot_id)

        self._boot_node = boot_node
        self.robot_cfg = robot_cfg
        self.gripper_cfg = gripper_cfg

        self.declare_parameter("web_result_url", "http://127.0.0.1:8000/api/inspect/result")
        self.web_result_url = self.get_parameter("web_result_url").value

        from tilemate_main.onrobot import RG
        self.gripper = RG(
            gripper_cfg.GRIPPER_NAME,
            gripper_cfg.TOOLCHARGER_IP,
            gripper_cfg.TOOLCHARGER_PORT,
        )

        self.cb_group = ReentrantCallbackGroup()

        self.srv = self.create_service(
            Inspect,
            "tile/inspect",
            self.inspect_callback,
            callback_group=self.cb_group,
        )

        self.inspection_engine = WallTileInspectionEngine(self)

        self.initialize_robot()
        self.get_logger().info("\033[94m [5/5] [INSPECT] initialize Done!\033[0m")

    def initialize_robot(self):
        from DSR_ROBOT2 import (
            set_tool,
            set_tcp,
            ROBOT_MODE_MANUAL,
            ROBOT_MODE_AUTONOMOUS,
            set_robot_mode,
            wait,
        )

        set_robot_mode(ROBOT_MODE_MANUAL)
        set_tool(self.robot_cfg.tool)
        set_tcp(self.robot_cfg.tcp)
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)

        time.sleep(1.0)
        wait(1.0)

    def move_to_inspect_pose(self):
        from DSR_ROBOT2 import posx, movesx, mwait

        candidates = [
            posx([380.6733093261719, 177.2272491455078, 179.8480987548828, 89.89385223388672, 91.91939544677734, 92.74739837646484]),
            posx([380.705, 157.182, 139.804, 90.000, 90.001, 89.999]),
            posx([380.705, 127.182, 109.804, 90.000, 90.001, 89.999]),
        ]

        movesx(candidates, vel=80, acc=80)
        mwait()

    def send_result_to_web(self, payload):
        url = self.web_result_url

        if not url:
            return False, "web_result_url is empty"

        try:
            r = requests.post(url, json=payload, timeout=3)
            r.raise_for_status()
            return True, r.text
        except Exception as e:
            return False, str(e)

    def inspect_callback(self, request, response):
        del request

        try:
            self.get_logger().info("[INSPECT] start inspection")

            self.gripper.open_gripper()
            self.move_to_inspect_pose()

            result = self.inspection_engine.analyze_once()
            if result["success"]:
                with open("inspect_result.json", "w") as f:
                    json.dump(result["result_dict"], f, indent=2)

            response.success = bool(result.get("success", False))
            response.message = str(result.get("message", ""))

            if response.success:
                payload = result.get("result_dict", None)
                self.get_logger().info(f"{payload}")
                if payload is None:
                    self.get_logger().warn("[INSPECT] result_dict is missing")
                    response.message = "inspect_complete_but_result_dict_missing"
                    return response

                ok, msg = self.send_result_to_web(payload)

                if ok:
                    self.get_logger().info("[INSPECT] web send success")
                    response.message = "inspect_complete"
                else:
                    self.get_logger().warn(f"[INSPECT] web send failed: {msg}")
                    response.message = f"inspect_complete_but_web_send_failed:{msg}"

            self.get_logger().info(f"{response}")
            return response

        except Exception as e:
            self.get_logger().error(f"[INSPECT] failed: {e}")
            response.success = False
            response.message = f"exception:{e}"
            return response


def main(args=None):
    rclpy.init(args=args)
    robot_cfg = RobotConfig()
    gripper_cfg = GripperConfig()

    boot = rclpy.create_node("dsr_boot_inspect", namespace=robot_cfg.robot_id)

    DR_init.__dsr__node = boot
    DR_init.__dsr__id = robot_cfg.robot_id
    DR_init.__dsr__model = robot_cfg.robot_model

    import DSR_ROBOT2  # noqa

    node = InspectService(robot_cfg, gripper_cfg, boot)

    ex = MultiThreadedExecutor()
    ex.add_node(node)

    try:
        ex.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
    finally:
        try:
            ex.remove_node(node)
        except Exception:
            pass

        try:
            node.destroy_node()
            boot.destroy_node()
        except Exception:
            pass

        rclpy.shutdown()


if __name__ == "__main__":
    main()