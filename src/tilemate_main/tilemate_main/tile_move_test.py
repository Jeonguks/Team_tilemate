#!/usr/bin/env python3
import time
import rclpy
import DR_init

# ----------------------------
# 로봇 설정 상수
# ----------------------------
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

VELOCITY = 40
ACC = 60

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def initialize_robot():
    """로봇의 Tool과 TCP를 설정"""
    from DSR_ROBOT2 import (
        set_tool, set_tcp, get_tool, get_tcp,
        ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS,
        get_robot_mode, set_robot_mode
    )

    set_robot_mode(ROBOT_MODE_MANUAL)
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)

    time.sleep(1.0)

    print("#" * 50)
    print("Initializing robot with the following settings:")
    print(f"ROBOT_ID: {ROBOT_ID}")
    print(f"ROBOT_MODEL: {ROBOT_MODEL}")
    print(f"ROBOT_TCP: {get_tcp()}")
    print(f"ROBOT_TOOL: {get_tool()}")
    print(f"ROBOT_MODE 0:수동, 1:자동 : {get_robot_mode()}")
    print(f"VELOCITY: {VELOCITY}")
    print(f"ACC: {ACC}")
    print("#" * 50)


def move_relative(dx: float, dy: float, dz: float, dw: float=0.0, dp: float=0.0, dr:float=0.0):
    from DSR_ROBOT2 import posx, posj, movej, movel, mwait, wait, DR_BASE, get_current_posx

    cur, _ = get_current_posx(DR_BASE)
    target = [
        cur[0] + dx,
        cur[1] + dy,
        cur[2] + dz,
        cur[3] + dw,
        cur[4] + dp,
        cur[5] + dr,
    ]
    movel(posx(target), ref=DR_BASE, vel=30, acc=30)
    mwait()
    print(get_current_posx(DR_BASE))

def test_tilt_positions():
    from DSR_ROBOT2 import posx, posj, movej, movel, mwait, wait, get_current_posx, DR_BASE

    JReady = posj([0, 0, 90, 0, 90, 0])

    print("[TEST] Home으로 이동")
    movej(JReady, vel=VELOCITY, acc=ACC)
    mwait()

    print(f"툴쪽 타일로 이동")
    movel(posx([351.784, -406.931, 200.327, 141.574, 178.994, -127.464]), vel=40, acc=40, ref=DR_BASE)
    mwait()
    wait(1.0)

    move_relative(0.0, 0.0, -200.0)


    move_relative(0.0, 0.0, 200.0)

    movej(JReady, vel=VELOCITY, acc=ACC)

    print(f"타일로 이동")
    movel(posx([436.158, -403.759, 200.0 , 170.548, 178.998, -98.723]), vel=30, acc=30, ref=DR_BASE)
    mwait()  
    wait(1.0)


    move_relative(0.0, 0.0, -200.0)

    move_relative(0.0, 0.0, 200.0)

    movej(JReady, vel=VELOCITY, acc=ACC)


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("test_tilt", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        initialize_robot()
        test_tilt_positions()

    except KeyboardInterrupt:
        print("\nNode interrupted by user. Shutting down...")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()