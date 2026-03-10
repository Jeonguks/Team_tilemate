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

    tool_pre_tilt = [369.025, 160.678, 196.749, 44.152, -179.9, -137.748]
    tool_tilt     = [369.025, 160.678, 196.749, 91.799, -167.285, -90.1]
    tile_wid = 80.0  # mm (타일 60mm + 간격 20mm)

    tile_offsets = {
        1: (-tile_wid,  tile_wid),   # 좌상
        2: (0.0,        tile_wid),   # 중상
        3: (tile_wid,   tile_wid),   # 우상
        4: (-tile_wid,  0.0),        # 좌중
        5: (0.0,        0.0),        # 중앙
        6: (tile_wid,   0.0),        # 우중
        7: (-tile_wid, -tile_wid),   # 좌하
        8: (0.0,       -tile_wid),   # 중하
        9: (tile_wid,  -tile_wid),   # 우하
    }

    print("[TEST] Home으로 이동")
    movej(JReady, vel=VELOCITY, acc=ACC)
    mwait()

    for i in range(1, 10):
        dx, dz = tile_offsets[i]
        print(f"\n[TEST] ===== Index={i} 시작 =====")

        # 1. pre_tilt 위치로 이동
        pre_tilt_target = tool_pre_tilt.copy()
        pre_tilt_target[0] += dx
        pre_tilt_target[2] += dz
        print(f"[TEST] Index={i} - 1. pre_tilt 이동: {pre_tilt_target}")
        movel(posx(pre_tilt_target), 30, 30, ref=DR_BASE)
        mwait()
        wait(1.0)

        # 2. 접근
        cur, _ = get_current_posx(DR_BASE)
        approach_target = [
            cur[0] + 0.0,
            cur[1] + 3.68,
            cur[2] - 5.0,
            cur[3], cur[4], cur[5]
        ]
        print(f"[TEST] Index={i} - 2. 접근 이동: {approach_target}")
        movel(posx(approach_target), 30, 30, ref=DR_BASE)
        mwait()
        wait(1.0)

        move_relative(0.0, 40.0, 0.0)
        
        # 3. 기울이기
        tilt_target = tool_tilt.copy()
        tilt_target[0] += dx
        tilt_target[2] += dz
        print(f"[TEST] Index={i} - 3. 기울이기 이동: {tilt_target}")
        movel(posx(tilt_target), 30, 30, ref=DR_BASE)
        mwait()
        wait(1.0)

        move_relative(0.0, 40.0, 0.0)

    print("\n[TEST] 완료 - Home으로 복귀")
    movej(JReady, vel=VELOCITY, acc=ACC)
    mwait()


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