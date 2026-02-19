# ============================================================
# M0609 Pick&Place + (각 타일 Place 직후) 압착 시퀀스
# - 타일 1개 Place 완료 → 즉시 press_only(i) 실행 (그리드 적용)
# - RG2 Modbus: pymodbus 키워드 호환(device_id/unit/slave) + positional 제거
# ============================================================

import rclpy
import DR_init
import time
from pymodbus.client import ModbusTcpClient

# ----------------------------
# 로봇 기본 설정
# ----------------------------
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP  = "GripperDA_v1"

VELOCITY = 30
ACC      = 30

DR_init.__dsr__id    = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# ----------------------------
# RG2 Modbus (그리퍼)
# ----------------------------
IP = "192.168.1.1"
PORT = 502
DEVICE = 65

REG_FORCE  = 0x0000
REG_WIDTH  = 0x0001
REG_START  = 0x0002
REG_STATUS = 0x010C

OPEN_W   = 0.025
CLOSE_W  = 0.015
CLOSE2_W = 0.006  # 압착도구 파지용

# ----------------------------
# 타일 개수 / 그리드 간격
# ----------------------------
NUM_TILES = 5

PICK_ABOVE_BASE = [340, 7, 200, 12, -180, 102]
PICK_DOWN_BASE  = [340, 7, 165, 168, -180, -102]
PICK_MOVE_BASE  = [415, 7, 200, 168, -180, -102]
PICK_DY_MM      = 32.0

PLACE_TILT_BASE  = [415, 175, 200, 103, 173, -163]
PLACE_DOWN_BASE  = [415, 175, 170,  87, 125,  178]
PLACE_MOVE_BASE1 = [415, 175, 145,  87, 125,  178]
PLACE_MOVE_BASE2 = [415, 165, 140,  87, 125,  178]
PLACE_MOVE_BASE3 = [415, 165, 135,  87, 125,  178]
PLACE_MOVE_BASE4 = [415, 160, 135,  87, 125,  178]
PLACE_MOVE_BASE5 = [415, 165, 135,  87, 125,  178]
PLACE_DY_MM      = -70.0

# ----------------------------
# 압착 파라미터 / 압착 그리드 간격(추가)
# ----------------------------
PRESS_FZ_N    = 2.0
PRESS_DWELL_S = 0.4

# ✅ 압착 그리드 간격: 기본은 PLACE와 동일하게 시작 (필요 시 따로 튜닝)
PRESS_DY_MM = PLACE_DY_MM
# PRESS_Y_OFFSET_MM = 0.0  # 필요하면 추가해서 press_dy = i*PRESS_DY_MM + PRESS_Y_OFFSET_MM

# ----------------------------
# 압착도구(픽업) 위치
# ----------------------------
PICK_PRESS_BASE = [340, -100, 200, 12, -180, 102]
PICK_PRESS_DOWN = [340, -100, 125, 168, -180, -102]
PICK_PRESS_MOVE = [415, -100, 200, 168, -180, -102]

# ----------------------------
# 압착 위치(타일 위치 기준)
# ----------------------------
PLACE_PRESS_BASE  = [415, 175, 200, 168, -180, -102]
PLACE_PRESS_DOWN1 = [415, 175, 170, 168, -180, -102]
PLACE_PRESS_DOWN2 = [415, 175, 135, 168, -180, -102]


# ============================================================
# 초기화
# ============================================================
def initialize_robot():
    from DSR_ROBOT2 import (
        set_tool, set_tcp,
        ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS,
        set_robot_mode
    )
    set_robot_mode(ROBOT_MODE_MANUAL)
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    time.sleep(1)


# ============================================================
# pymodbus 호환 wrapper (키워드 only)
# ============================================================
def _mb_write_register(client, address, value, device):
    for kw in ("device_id", "unit", "slave"):
        try:
            return client.write_register(address=address, value=value, **{kw: device})
        except TypeError:
            continue
    return client.write_register(address=address, value=value)


def _mb_read_holding_registers(client, address, count, device):
    for kw in ("device_id", "unit", "slave"):
        try:
            return client.read_holding_registers(address=address, count=count, **{kw: device})
        except TypeError:
            continue
    return client.read_holding_registers(address=address, count=count)


# ============================================================
# RG2 그리퍼 제어
# ============================================================
def set_gripper(width_m: float, force_n: float = 40.0):
    client = ModbusTcpClient(IP, port=PORT)
    if not client.connect():
        raise RuntimeError("Modbus 연결 실패")

    try:
        force_val = int(force_n * 10)  # 0.1N 단위 가정
        width_mm  = width_m * 1000.0
        width_val = int(width_mm * (386 / 30.0))  # 너가 쓰던 스케일

        _mb_write_register(client, REG_FORCE, force_val, DEVICE)
        _mb_write_register(client, REG_WIDTH, width_val, DEVICE)
        _mb_write_register(client, REG_START, 1, DEVICE)

        while True:
            rr = _mb_read_holding_registers(client, REG_STATUS, 1, DEVICE)
            busy = rr.registers[0] & 0x01
            if not busy:
                break
            time.sleep(0.05)

    finally:
        client.close()


def add_y_offset(posx, dy_mm: float):
    p = posx[:]
    p[1] += dy_mm
    return p


# ============================================================
# 압착 그리드 좌표 생성
# ============================================================
def press_positions(i: int):
    press_dy = i * PRESS_DY_MM
    # press_dy = i * PRESS_DY_MM + PRESS_Y_OFFSET_MM  # 필요 시
    p_base  = add_y_offset(PLACE_PRESS_BASE,  press_dy)
    p_down1 = add_y_offset(PLACE_PRESS_DOWN1, press_dy)
    p_down2 = add_y_offset(PLACE_PRESS_DOWN2, press_dy)
    return p_base, p_down1, p_down2


# ============================================================
# 압착도구 집기/놓기
# ============================================================
def pick_press_tool():
    from DSR_ROBOT2 import movel, wait
    print("▶ 압착도구 집기")
    movel(PICK_PRESS_BASE, vel=VELOCITY, acc=ACC)
    movel(PICK_PRESS_DOWN, vel=5, acc=5)
    set_gripper(CLOSE2_W)
    wait(0.5)
    movel(PICK_PRESS_BASE, vel=VELOCITY, acc=ACC)
    movel(PICK_PRESS_MOVE, vel=VELOCITY, acc=ACC)


def release_press_tool():
    from DSR_ROBOT2 import movel, wait
    print("▶ 압착도구 내려놓기")
    movel(PICK_PRESS_BASE, vel=VELOCITY, acc=ACC)
    movel(PICK_PRESS_DOWN, vel=5, acc=5)
    set_gripper(OPEN_W)
    wait(0.5)
    movel(PICK_PRESS_BASE, vel=VELOCITY, acc=ACC)
    movel(PICK_PRESS_MOVE, vel=VELOCITY, acc=ACC)


# ============================================================
# 타일 i번째 압착 수행 (Place 직후 호출용)
# ============================================================
def press_only_i(i: int):
    from DSR_ROBOT2 import (
        movel, wait,
        task_compliance_ctrl,
        release_compliance_ctrl,
        release_force,
        set_desired_force,
        check_force_condition,
        set_ref_coord,
        DR_BASE,
        DR_AXIS_Z,
        DR_FC_MOD_REL
    )

    print(f"===================================")
    print(f"▶ [{i+1}/{NUM_TILES}] 압착 시작")
    print(f"===================================")

    # 1) 도구 집기
    pick_press_tool()

    # 2) 압착 위치 이동(그리드 적용)
    p_base, p_down1, p_down2 = press_positions(i)
    print("▶ 압착 위치 이동")
    movel(p_base,  vel=VELOCITY, acc=ACC)
    movel(p_down1, vel=VELOCITY, acc=ACC)
    movel(p_down2, vel=3,        acc=3)

    # 3) 힘제어
    print("▶ 힘제어 시작")
    set_ref_coord(1)  # Tool 기준
    task_compliance_ctrl(stx=[1000, 1000, 200, 200, 200, 200])
    wait(0.2)

    set_desired_force(
        fd=[0, 0, PRESS_FZ_N, 0, 0, 0],
        dir=[0, 0, 1,         0, 0, 0],
        mod=DR_FC_MOD_REL
    )

    print(f"▶ 목표 힘 도달 대기: {PRESS_FZ_N:.1f}N")
    while True:
        ret = check_force_condition(DR_AXIS_Z, min=0, max=PRESS_FZ_N)
        if ret == -1:
            print("▶ 목표 힘 도달")
            break
        wait(0.2)

    print(f"▶ 유지(dwell): {PRESS_DWELL_S:.2f}s")
    wait(PRESS_DWELL_S)

    print("▶ 힘/순응 해제")
    release_force()
    release_compliance_ctrl()

    print("▶ 기준좌표 DR_BASE 복귀")
    set_ref_coord(DR_BASE)
    wait(0.2)

    print("▶ 상승")
    movel(p_base, vel=VELOCITY, acc=ACC)

    # 4) 도구 내려놓기
    release_press_tool()

    print(f"===================================")
    print(f"▶ [{i+1}/{NUM_TILES}] 압착 완료")
    print(f"===================================")


# ============================================================
# Pick&Place + 압착(타일마다 반복)
# ============================================================
def perform_task(node):
    from DSR_ROBOT2 import (
        movej, movel, wait,
        release_compliance_ctrl, release_force,
        check_force_condition,
        task_compliance_ctrl,
        set_desired_force,
        set_ref_coord,
        DR_FC_MOD_REL,
        DR_AXIS_Z,
        DR_BASE
    )

    JReady = [0, 0, 90, 0, 90, 90]
    movej(JReady, vel=VELOCITY, acc=ACC)

    node.get_logger().info("RG2 OPEN - init")
    set_gripper(OPEN_W)
    time.sleep(0.5)

    for i in range(NUM_TILES):
        if i == 0:
            v_pick_down, a_pick_down = 8, 8
            v_place_down, a_place_down = 3, 3
            v_slide, a_slide = 10, 10
            open_hold = 1.0
        else:
            v_pick_down, a_pick_down = 12, 12
            v_place_down, a_place_down = 8, 8
            v_slide, a_slide = VELOCITY, ACC
            open_hold = 0.3

        pick_dy  = i * PICK_DY_MM
        place_dy = i * PLACE_DY_MM

        pick_above = add_y_offset(PICK_ABOVE_BASE, pick_dy)
        pick_down  = add_y_offset(PICK_DOWN_BASE,  pick_dy)
        pick_move  = add_y_offset(PICK_MOVE_BASE,  pick_dy)

        place_tilt  = add_y_offset(PLACE_TILT_BASE,  place_dy)
        place_down  = add_y_offset(PLACE_DOWN_BASE,  place_dy)
        place_move1 = add_y_offset(PLACE_MOVE_BASE1, place_dy)
        place_move2 = add_y_offset(PLACE_MOVE_BASE2, place_dy)
        place_move3 = add_y_offset(PLACE_MOVE_BASE3, place_dy)
        place_move4 = add_y_offset(PLACE_MOVE_BASE4, place_dy)
        place_move5 = add_y_offset(PLACE_MOVE_BASE5, place_dy)

        # -----------------------
        # PICK
        # -----------------------
        node.get_logger().info(f"[{i+1}/{NUM_TILES}] PICK")
        movel(pick_above, vel=VELOCITY, acc=ACC)
        movel(pick_down,  vel=v_pick_down, acc=a_pick_down)
        set_gripper(CLOSE_W)
        wait(0.5)
        movel(pick_above, vel=VELOCITY, acc=ACC)
        movel(pick_move,  vel=VELOCITY, acc=ACC)

        # -----------------------
        # PLACE + 4N 임시 고정(네 기존 로직)
        # -----------------------
        node.get_logger().info(f"[{i+1}/{NUM_TILES}] PLACE")
        movel(place_tilt, vel=VELOCITY, acc=ACC)
        movel(place_down, vel=v_place_down, acc=a_place_down)
        movel(place_move1, vel=v_slide, acc=a_slide)
        movel(place_move2, vel=v_slide, acc=a_slide)

        print("힘 제어 시작(4N 임시 고정)...")
        set_ref_coord(1)
        task_compliance_ctrl(stx=[1000, 1000, 200, 200, 200, 200])
        wait(0.5)
        set_desired_force(fd=[0, 0, 4, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)

        while True:
            ret = check_force_condition(DR_AXIS_Z, min=0, max=4)
            if ret == -1:
                break
            wait(0.2)

        print("힘 제어 해제...")
        release_force()
        release_compliance_ctrl()

        print("기준좌표 복귀 + 안정화")
        set_ref_coord(DR_BASE)
        wait(0.2)

        # 릴리즈
        set_gripper(OPEN_W)
        time.sleep(open_hold)

        # 후퇴 슬라이드
        movel(place_move3, vel=v_slide, acc=a_slide)
        movel(place_move4, vel=v_slide, acc=a_slide)
        movel(place_move5, vel=v_slide, acc=a_slide)
        movel(place_move4, vel=v_slide, acc=a_slide)
        movel(place_tilt,  vel=VELOCITY, acc=ACC)

        # -----------------------
        # ✅ 각 타일 Place 직후 "압착" 실행(그리드 적용)
        # -----------------------
        press_only_i(i)

    movej(JReady, vel=VELOCITY, acc=ACC)


# ============================================================
# Main
# ============================================================
def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node("tile_pick_place_with_press_each", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    initialize_robot()
    perform_task(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
