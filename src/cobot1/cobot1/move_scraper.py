#!/usr/bin/env python3
import time
import rclpy
import DR_init

from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32, String

# =========================
# 로봇 설정 상수
# =========================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

VELOCITY = 40
ACC = 60

ON, OFF = 1, 0

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


is_start_requested = False
stop_requested = False


def initialize_robot():
    """로봇의 Tool과 TCP를 설정"""
    from DSR_ROBOT2 import (
        set_tool, set_tcp, get_tool, get_tcp,
        ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS,
        get_robot_mode, set_robot_mode
    )

    # Tool/TCP 설정은 Manual에서 수행
    set_robot_mode(ROBOT_MODE_MANUAL)
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)

    # 다시 Autonomous로 복귀
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    time.sleep(1.0)  # 설정 안정화

    print("#" * 50)
    print("Initializing robot with the following settings:")
    print(f"ROBOT_ID: {ROBOT_ID}")
    print(f"ROBOT_MODEL: {ROBOT_MODEL}")
    print(f"ROBOT_TCP: {get_tcp()}")
    print(f"ROBOT_TOOL: {get_tool()}")
    print(f"ROBOT_MODE (0:수동, 1:자동): {get_robot_mode()}")
    print(f"VELOCITY: {VELOCITY}")
    print(f"ACC: {ACC}")
    print("#" * 50)

def trigger_cb(msg: String):
    global is_start_requested
    global stop_requested
    command = msg.data.strip().lower()

    if command == "start":
        print("[CMD] start received")
        is_start_requested = True
        stop_requested = False  
    elif command == "pause":
        print("[CMD] pause received")
    elif command == "reset":
        print("[CMD] reset received")

def perform_task(node: Node):
    """로봇이 수행할 작업 + 그리퍼 퍼블리시"""
    from DSR_ROBOT2 import posx,movej,movel, posj # movel은 지금 주석처리라 posx만
    print("Performing grip task...")
    from DSR_ROBOT2 import (
        set_digital_output,
        get_digital_input,
        movej,wait
    )
    # 디지털 입력 신호 대기 함수
    def wait_digital_input(sig_num):
        while not get_digital_input(sig_num):
            wait(0.5)
            # print("Waiting for digital input...")

    def go_home_and_stop():
        """stop 명령 시: 초기 위치로 복귀 후 작업 종료"""
        try:
            node.get_logger().warn("STOP requested -> moving to JReady and exiting task")
            movej(JReady, vel=VELOCITY, acc=ACC)
        except Exception as e:
            node.get_logger().error(f"Failed to move to JReady on STOP: {e}")
        return True  # stop 처리됨

    def check_stop()->bool:
        """동작 사이사이에 호출: stop이면 복귀 후 True"""
        # 콜백 처리를 위해 spin 한번 돌려줌 (중요)
        rclpy.spin_once(node, timeout_sec=0.0)

        if stop_requested:
            stop_requested = False  # 1회성 처리
            return go_home_and_stop()
        return False
    
    # Release 동작
    def release():
        print("Releasing...")
        set_digital_output(2, ON)
        set_digital_output(1, OFF)

    # Grip 동작
    def grip():
        print("Gripping...")
        # release()
        set_digital_output(1, ON)
        set_digital_output(2, OFF)

    # ✅ OnRobot 그리퍼 명령 퍼블리셔
    gripper_pub = node.create_publisher(
        Float64MultiArray,
        "/onrobot/finger_width_controller/commands",
        10
    )

    step_pub = node.create_publisher(
        Int32,
        "/robot/step",
        10
    )

    state_pub = node.create_publisher(
        String,
        "/robot/state",
        10
    )





    '''
    initial 0
    접착제 파지중1 
    도포중 2
    타일 3

    /robot/command/action 
    "start" -> 시작 
    '''
    def set_gripper(width_m: float):
        """
        width_m: finger width (m)
        예) 0.05 => 50mm
        """
        msg = Float64MultiArray()
        msg.data = [float(width_m)]  # unit : meter
        gripper_pub.publish(msg)
        node.get_logger().info(f"[GRIPPER] publish: {msg.data}")

    def pick_scraper():
        step_msg = Int32()
        step_msg.data = 1  
        step_pub.publish(step_msg)
        state_msg = String()
        state_msg.data = "접착제 도포준비중"
        state_pub.publish(state_msg)
    
        #밀대 파지전 위치 이동 
        pre_grasp_pos = posx([634.6129150390625, 70.18247985839844, 216.4523162841797, 52.00111389160156, 179.09434509277344, 52.347633361816406])
        movel(pre_grasp_pos,vel=60,acc=60)
        time.sleep(1.0)
        #밀대 파지위치 이동
        grasp_pos = posx([635.6554565429688, 69.99335479736328, 156.9518280029297, 122.97427368164062, 179.7987518310547, 123.16593170166016])
        movel(grasp_pos,vel=60,acc=60)
        time.sleep(1.0)
        set_gripper(0.003)
        time.sleep(4.0)

    def place_scraper():
        step_msg = Int32()
        step_msg.data = 3 
        step_pub.publish(step_msg)
        state_msg = String()
        state_msg.data = "타일파지 준비중"
        state_pub.publish(state_msg)
        #밀대 파지전 위치 이동 
        pre_grasp_pos = posx([634.6129150390625, 70.18247985839844, 216.4523162841797, 52.00111389160156, 179.09434509277344, 52.347633361816406])
        movel(pre_grasp_pos,vel=60,acc=60)
        time.sleep(1.0)
        #밀대 파지위치 이동
        grasp_pos = posx([635.6554565429688, 69.99335479736328, 186.9518280029297, 122.97427368164062, 179.7987518310547, 123.16593170166016])
        movel(grasp_pos,vel=60,acc=60)
        time.sleep(1.0)
        set_gripper(0.04)
        time.sleep(4.0)



    # 초기 위치 및 목표 위치
    JReady = [0, 0, 90, 0, 90, 0]
    node.get_logger().info(f"JReady = {JReady}")

    # 반복 동작
    while rclpy.ok():
        movej(JReady, vel=VELOCITY, acc=ACC)
        set_gripper(0.06) # release
        time.sleep(2.0)
        
        pick_scraper()

        #밀대 들고 위로 이동 
        pos3 = posx([634.6129150390625, 70.18247985839844, 216.4523162841797, 52.00111389160156, 179.09434509277344, 52.347633361816406])
        movel(pos3,vel=60,acc=60)
        time.sleep(1.0)

        #밀대 들고 가운데로 이동 
        pos4 = posx([480.86981201171875, 68.99758911132812, 167.26080322265625, 59.91958999633789, 179.1564178466797, 60.55112075805664])
        movel(pos4,vel=60,acc=60)
        time.sleep(1.0)

        pos5 = posj([6.65,16.37,75.84,0.56,87.29,95.63])
        movej(pos5, vel=40,acc=40)
        time.sleep(1.0)

        ########################################
    
        print("시멘트 펴바르기 작업 시작")
        step_msg = Int32()
        step_msg.data = 2
        step_pub.publish(step_msg)
        state_msg = String()
        state_msg.data = "접착제 도포중"
        state_pub.publish(state_msg)
        tilt_right = posx([465.40,-34.13,160.68,92.35,-142.05,179.91])   #tilt +x
        movel(tilt_right, vel=40,acc=40)
        time.sleep(1.0)

        pos7 = posx([485.40,250.13,160.68,92.35,-142.05,179.91])
        movel(pos7, vel=40,acc=40)
        time.sleep(1.0)

        tilt_left = posx([485.40,274.11,160.68,93.23,148.62,-179.15]) # tilt -x
        movel(tilt_left, vel=40,acc=40)

        time.sleep(1.0)

        pos8 = posx([505.40,-115.13,160.68,93.23,148.62,-179.15]) 
        movel(pos8, vel=40,acc=40)

        time.sleep(1.0)

        tilt_right2 = posx([505.40,-115.13,165.68,92.35,-142.05,179.91])   #tilt +x
        movel(tilt_right2, vel=40,acc=40)
        time.sleep(1.0)

        tilt_right2 = posx([505.40,-115.13,160.68,92.35,-142.05,179.91])   #move -5 z axis
        movel(tilt_right2, vel=40,acc=40)
        time.sleep(1.0)
    
        pos9 = posx([505.40, 250.13,165.68,92.35,-142.05,179.91])  
        movel(pos9, vel=40,acc=40)
        time.sleep(1.0)



        time.sleep(1.0)
        print("시멘트 펴바르기 작업 종료")
        #########################################
        movej(JReady, vel=VELOCITY, acc=ACC)
        time.sleep(2.0)

        place_scraper()



def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node("move_scraper", namespace=ROBOT_ID)
    global is_start_requested
    # DR_init에 노드 설정
    DR_init.__dsr__node = node

    node.create_subscription(
        String,
        "/robot/command",
        trigger_cb,
        10
    )

    try:
        initialize_robot()
        print("Waiting for /robot/command 'start' ...")
        # while rclpy.ok() and not is_start_requested:
        #     rclpy.spin_once(node, timeout_sec=0.1)

        # if not rclpy.ok():
        #     return

        # is_start_requested = False  # 1회성 트리거면 리셋
        perform_task(node)

    except KeyboardInterrupt:
        print("\nInterrupted by user. Shutting down...")

    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        raise

    finally:
        try:
            node.destroy_node()
        except Exception:
            pass

        # init 안 된 상태에서 shutdown 방지용 (안전)
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
