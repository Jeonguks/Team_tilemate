# ============================================================
# M0609 Pick & Place (2x2) - (n,m,offset) 방식 (수정본)
#
# 목표
# - PLACE: PLACE_TILT_GRID에서 (n,m)로 "틸트 접근" 좌표를 직접 선택 + offset(X +45mm) 적용
# - PRESS/힘제어/컴플라이언스 전부 제거
# - PLACE_DOWN / SLIDE / RETREAT:
#     * XYZ는 place_tilt 기준 "상대 오프셋(mm)"로 생성
#     * RPY(회전)는 각 단계별 기준좌표(PLACE_DOWN_BASE, PLACE_MOVE_BASE*)의 RPY로 강제 적용
#       -> 너가 겪은 문제(87,125,178 같은 RPY를 오프셋으로 넣어서 XYZ가 튀는 문제) 방지
# - PICK: 지금은 Y만 누적(g * PICK_DY_MM) (공급 라인을 Y로 늘린 버전)
#
# 주의
# - posx = [x(mm), y(mm), z(mm), r(deg), p(deg), y(deg)] 형식 가정
# - PLACE_TILT_GRID는 "각 타일 위치별 접근 자세"를 이미 보정해둔 값이라고 가정
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
# RG2 Modbus
# ----------------------------
IP = "192.168.1.1"
PORT = 502
DEVICE = 65

REG_FORCE  = 0x0000
REG_WIDTH  = 0x0001
REG_START  = 0x0002
REG_STATUS = 0x010C

OPEN_W  = 0.025
CLOSE_W = 0.015

# ----------------------------
# PICK 설정 (공급 라인)
# ----------------------------
# 지금은 Y만 누적해서 공급 위치를 아래로 내려가며 집는 시나리오
PICK_DY_MM = 32.5   # g=0..3 => 0, 32.5, 65, 97.5

PICK_ABOVE_BASE = [345, 13, 200, 12, -180, 102]
PICK_DOWN_BASE  = [345, 13, 165, 168, -180, -102]
PICK_MOVE_BASE  = [415, 13, 200, 168, -180, -102]

# ----------------------------
# PLACE (n,m) TILT 그리드 (너가 추가한 좌표)
# - 이 좌표는 "각 (n,m) 위치로 접근할 때"의 자세를 이미 맞춘 값으로 사용
# ----------------------------
# PLACE_TILT_BASE01 = [415, 175, 200, 103, 173, -163] # 라인 시작 위치
# PLACE_TILT_BASE02 = [415, 105, 200, 103, 173, -163]
# PLACE_TILT_BASE03 = [415,  45, 200, 103, 173, -163]
# PLACE_TILT_BASE04 = [415, -25, 200, 103, 173, -163]
# PLACE_TILT_BASE05 = [415, -95, 200, 103, 173, -163]

# PLACE_TILT_BASE06 = [495,-165, 200, 103, 173, -163]
# PLACE_TILT_BASE07 = [495, 175, 200, 103, 173, -163]
# PLACE_TILT_BASE08 = [495, 115, 200, 103, 173, -163]
# PLACE_TILT_BASE09 = [495,  45, 200, 103, 173, -163]
# PLACE_TILT_BASE10 = [495, -25, 200, 103, 173, -163]

# PLACE_TILT_BASE11 = [575, 175, 200, 103, 173, -163]
# PLACE_TILT_BASE12 = [575, 115, 200, 103, 173, -163]
# PLACE_TILT_BASE13 = [575,  45, 200, 103, 173, -163]
# PLACE_TILT_BASE14 = [575, -25, 200, 103, 173, -163]
# PLACE_TILT_BASE15 = [575, -95, 200, 103, 173, -163]

# pitch 가정: 70mm (타일 60 + 여유 10)

# X = 415 (5개)
PLACE_TILT_BASE01 = [415, 175, 200, 103, 173, -163]
PLACE_TILT_BASE02 = [415, 100, 200, 103, 173, -163]
PLACE_TILT_BASE03 = [415,  40, 200, 103, 173, -163]
PLACE_TILT_BASE04 = [415, -30, 200, 103, 173, -163]
PLACE_TILT_BASE05 = [415,-100, 200, 103, 173, -163]

# X = 495 (5개)  <- BASE06을 여기 첫번째로 재배치
PLACE_TILT_BASE06 = [485, 175, 200, 103, 173, -163]
PLACE_TILT_BASE07 = [485, 100, 200, 103, 173, -163]
PLACE_TILT_BASE08 = [485,  40, 200, 103, 173, -163]
PLACE_TILT_BASE09 = [485, -30, 200, 103, 173, -163]
PLACE_TILT_BASE10 = [485,-100, 200, 103, 173, -163]

# X = 575 (5개)
PLACE_TILT_BASE11 = [575, 175, 200, 103, 173, -163]
PLACE_TILT_BASE12 = [575, 100, 200, 103, 173, -163]
PLACE_TILT_BASE13 = [575,  40, 200, 103, 173, -163]
PLACE_TILT_BASE14 = [575, -30, 200, 103, 173, -163]
PLACE_TILT_BASE15 = [575,-100, 200, 103, 173, -163]


PLACE_TILT_GRID = {
    (0,0): PLACE_TILT_BASE01, (0,1): PLACE_TILT_BASE02, (0,2): PLACE_TILT_BASE03, (0,3): PLACE_TILT_BASE04, (0,4): PLACE_TILT_BASE05,
    (1,0): PLACE_TILT_BASE06, (1,1): PLACE_TILT_BASE07, (1,2): PLACE_TILT_BASE08, (1,3): PLACE_TILT_BASE09, (1,4): PLACE_TILT_BASE10,
    (2,0): PLACE_TILT_BASE11, (2,1): PLACE_TILT_BASE12, (2,2): PLACE_TILT_BASE13, (2,3): PLACE_TILT_BASE14, (2,4): PLACE_TILT_BASE15,
}

# ----------------------------
# 2x2 배치 계획 (n,m,offset)
# - offset: 1 => x +0mm, 2 => x +45mm
# ----------------------------
# 2x2 그리드에서 (n,m) = (0,0), (0,1), (1,0), (1,1) 위치에 타일을 놓는 시나리오
PLACE_PLAN_2x2 = [
    (0,0,1),  # 1) 좌상
    (0,1,1),  # 2) 좌하
    (1,0,1),  # 3) 우상
    (1,1,1),  # 4) 우하
]

# ----------------------------
# "기준 좌표" (한 위치에서 측정한 down/move 단계의 기준)
# - 여기서 RPY를 추출해서 각 단계별로 강제 적용한다.
# - XYZ는 place_tilt 기반으로 상대 오프셋으로 만들고, RPY는 아래 기준의 RPY로 덮어쓴다.
# ----------------------------
PLACE_TILT_BASE  = [415, 175, 200, 103, 173, -163] # (n,m,offset)에서 타일 배치 접근 자세로 이동할 때의 기준 좌표 (위 그리드에서 하나 선택)
PLACE_DOWN_BASE  = [415, 175, 170,  87, 125,  178] # (tilt에서 z만 30mm 내려간 위치, RPY는 down 기준으로 적용)
PLACE_MOVE_BASE1 = [415, 175, 145,  87, 125,  178] # (꺽어 놓기용, 상황에 맞게 조절 필요)
PLACE_MOVE_BASE2 = [415, 250, 140,  87, 125,  178] # (Y를 좀 빼서 슬라이드할 때 뒤로 빠지도록, 상황에 맞게 조절 필요)
PLACE_MOVE_BASE3 = [415, 250, 130,  87, 125,  178] # (조금더 하강용, 상황에 맞게 조절 필요)
PLACE_MOVE_BASE4 = [415, 250, 130,  87, 125,  178] # 후퇴할 때는 Y를 크게 빼서 뒤로 빠지도록 (250은 임의값, 상황에 맞게 조절 필요)
PLACE_MOVE_BASE5 = [415, 150, 140,  87, 125,  178] # (조금더 후 후퇴용, 상황에 맞게 조절 필요)

# ----------------------------
# PLACE 상대 오프셋(tilt 기준)  ※ "XYZ(mm)"만 사용해야 함
# - (중요) 87,125,178 같은 값은 RPY이므로 오프셋으로 쓰면 좌표가 폭발함
# - 아래 오프셋은 위 BASE들에서 XYZ 차이로 계산한 값
# ----------------------------
REL_DOWN = (0.0,   0.0, -30.0)   # 200 -> 170
REL_M1   = (0.0,   0.0, -55.0)   # 200 -> 145
REL_M2   = (0.0, -10.0, -60.0)   # 175 -> 165, 200 -> 140
REL_M3   = (0.0, -10.0, -65.0)   # 175 -> 155, 200 -> 135
REL_M4   = (0.0, -15.0, -65.0)   # 175 -> 160, 200 -> 135
REL_M5   = (0.0,  -5.0, -65.0)   # 175 -> 170, 200 -> 135

# ============================================================
# Print Helper
# ============================================================
def step(msg: str): print(f"\n[STEP] {msg}")
def info(msg: str): print(f"[INFO] {msg}")

# ============================================================
# 초기화
# ============================================================
def initialize_robot():
    from DSR_ROBOT2 import set_tool, set_tcp, ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS, set_robot_mode
    step("Robot Initialize")
    set_robot_mode(ROBOT_MODE_MANUAL)
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    time.sleep(1)

# ============================================================
# Modbus Wrapper (pymodbus 호환)
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
# Gripper
# ============================================================
def set_gripper(width_m: float, force_n: float = 40.0, timeout_s: float = 3.0):
    client = ModbusTcpClient(IP, port=PORT)
    if not client.connect():
        raise RuntimeError("Modbus 연결 실패")

    try:
        force_val = int(force_n * 10)
        width_mm  = width_m * 1000.0
        width_val = int(width_mm * (386 / 30.0))

        _mb_write_register(client, REG_FORCE, force_val, DEVICE)
        _mb_write_register(client, REG_WIDTH, width_val, DEVICE)
        _mb_write_register(client, REG_START, 1, DEVICE)

        t0 = time.time()
        while True:
            rr = _mb_read_holding_registers(client, REG_STATUS, 1, DEVICE)
            if rr is None or getattr(rr, "isError", lambda: True)():
                raise RuntimeError("Modbus read error (STATUS)")
            if not hasattr(rr, "registers") or len(rr.registers) < 1:
                raise RuntimeError("Modbus STATUS empty")

            busy = rr.registers[0] & 0x01
            if not busy:
                break

            if time.time() - t0 > timeout_s:
                raise RuntimeError("Gripper busy timeout")
            time.sleep(0.05)
    finally:
        client.close()

# ============================================================
# 좌표 유틸
# ============================================================
def add_xy_offset(posx, dx_mm: float, dy_mm: float):
    """XYZ 중 X,Y에 mm 오프셋 적용 (PICK용)"""
    p = posx[:]
    p[0] += dx_mm
    p[1] += dy_mm
    return p

def add_xyz_offset_keep_rpy(posx, dx_mm: float, dy_mm: float, dz_mm: float):
    """XYZ에 mm 오프셋 적용, RPY는 유지"""
    p = posx[:]
    p[0] += dx_mm
    p[1] += dy_mm
    p[2] += dz_mm
    return p

def apply_rpy(dst_posx, rpy_src_posx):
    """dst_posx의 RPY를 rpy_src_posx의 RPY로 덮어쓰기"""
    p = dst_posx[:]
    p[3], p[4], p[5] = rpy_src_posx[3], rpy_src_posx[4], rpy_src_posx[5]
    return p

# ============================================================
# (n,m,offset) -> place_tilt
# ============================================================
def get_place_tilt_nm(n: int, m: int, offset: int = 1):
    if (n, m) not in PLACE_TILT_GRID:
        raise ValueError(f"Invalid (n,m)=({n},{m})")

    base = PLACE_TILT_GRID[(n, m)][:]

    # offset 규칙: 1 -> 0mm, 2 -> +45mm
    if offset == 1:
        n_offset_mm = 0.0
    elif offset == 2:
        n_offset_mm = 0.0
    else:
        raise ValueError(f"Invalid offset={offset} (only 1 or 2)")

    base[0] += n_offset_mm
    return base

##  힘제어 순응제어 추가 (슬라이드 후 타일 고정용)
def hold_with_force_4n(node, hold_n=4.0, timeout_s=3.0):
    # [타일 임시 고정] 슬라이드 정렬 끝난 뒤, 그리퍼 오픈 전에 4N으로 잠깐 눌러 고정
    from DSR_ROBOT2 import (
    set_ref_coord, task_compliance_ctrl, release_compliance_ctrl,
    set_desired_force, release_force, check_force_condition, wait,
    DR_BASE, DR_AXIS_Z, DR_FC_MOD_REL
)
    set_ref_coord(1)  # Tool 기준(환경에 따라 Tool 번호 확인 필요)
    task_compliance_ctrl(stx=[1000, 1000, 200, 200, 200, 200])
    wait(0.3)

    set_desired_force(
        fd=[0, 0, hold_n, 0, 0, 0],
        dir=[0, 0, 1, 0, 0, 0],
        mod=DR_FC_MOD_REL
    )

    t0 = time.time()
    while True:
        ret = check_force_condition(DR_AXIS_Z, min=0, max=hold_n)
        if ret == -1:
            break
        if time.time() - t0 > timeout_s:
            node.get_logger().warn("HOLD: force condition timeout -> release")
            break
        wait(0.2)

    release_force()
    release_compliance_ctrl()
    set_ref_coord(DR_BASE)
    wait(0.2)


# ============================================================
# Pick & Place (2x2)
# ============================================================
def perform_task(node):
    from DSR_ROBOT2 import (
    movej, movel, wait,
    set_ref_coord, task_compliance_ctrl, release_compliance_ctrl,
    set_desired_force, release_force, check_force_condition,
    DR_BASE, DR_AXIS_Z, DR_FC_MOD_REL
    )

    JReady = [0, 0, 90, 0, 90, 90]

    step("Move to JReady")
    movej(JReady, vel=VELOCITY, acc=ACC)

    step("Gripper OPEN init")
    set_gripper(OPEN_W)
    time.sleep(0.3)

    total = len(PLACE_PLAN_2x2)

    for k, (n, m, offset) in enumerate(PLACE_PLAN_2x2):
        g = k  # PICK 누적 인덱스(4장이라 g=0..3)

        # ----------------------------
        # PICK: Y 누적
        # ----------------------------
        pick_dy = g * PICK_DY_MM

        pick_above = add_xy_offset(PICK_ABOVE_BASE, 0.0, pick_dy)
        pick_down  = add_xy_offset(PICK_DOWN_BASE,  0.0, pick_dy)
        pick_move  = add_xy_offset(PICK_MOVE_BASE,  0.0, pick_dy)

        # ----------------------------
        # PLACE: (n,m,offset)로 tilt 접근 좌표 선택
        # ----------------------------
        place_tilt = get_place_tilt_nm(n, m, offset) 

        # ----------------------------
        # PLACE 단계 좌표 생성
        # - XYZ: place_tilt 기준 상대 오프셋 적용
        # - RPY: 단계별 기준 좌표의 RPY로 덮어씀
        # ----------------------------
        place_down = apply_rpy(add_xyz_offset_keep_rpy(place_tilt, *REL_DOWN), PLACE_DOWN_BASE)
        place_m1   = apply_rpy(add_xyz_offset_keep_rpy(place_tilt, *REL_M1),   PLACE_MOVE_BASE1)
        place_m2   = apply_rpy(add_xyz_offset_keep_rpy(place_tilt, *REL_M2),   PLACE_MOVE_BASE2)
        place_m3   = apply_rpy(add_xyz_offset_keep_rpy(place_tilt, *REL_M3),   PLACE_MOVE_BASE3)
        place_m4   = apply_rpy(add_xyz_offset_keep_rpy(place_tilt, *REL_M4),   PLACE_MOVE_BASE4)
        place_m5   = apply_rpy(add_xyz_offset_keep_rpy(place_tilt, *REL_M5),   PLACE_MOVE_BASE5)

        # step(f"[{k+1}/{total}] PLACE (n,m,offset)=({n},{m},{offset}) | PICK(dy={pick_dy:.1f})")
        info(f"place_tilt={place_tilt}")
        info(f"place_down={place_down}")
        info(f"place_m1  ={place_m1}")
        info(f"place_m2  ={place_m2}")

        # ---------------- PICK ----------------
        movel(pick_above, vel=VELOCITY, acc=ACC) #  
        movel(pick_down,  vel=10, acc=10)
        set_gripper(CLOSE_W)
        wait(0.3)
        movel(pick_above, vel=VELOCITY, acc=ACC)
        movel(pick_move,  vel=VELOCITY, acc=ACC)

        # ---------------- PLACE ----------------
        movel(place_tilt, vel=VELOCITY, acc=ACC) # 타일 배치 접근 자세로 이동
        movel(place_down, vel=5, acc=5) # 타일 배치 자세로 하강

        # 슬라이드(정렬)
        movel(place_m1, vel=10, acc=10) # 꺽어 놓기
        movel(place_m2, vel=10, acc=10) #

        # 힘제어로 타일 고정 (슬라이드 후, 그리퍼 오픈 전에 4N으로 잠깐 눌러 고정)
        hold_with_force_4n(node, hold_n=4.0, timeout_s=3.0)  # 여기

        # 릴리즈
        set_gripper(OPEN_W)
        time.sleep(0.3)

        # 후퇴
        movel(place_m3, vel=10, acc=10) #조금더 하강
        #movel(place_m4, vel=10, acc=10) #후퇴 
        #movel(place_m5, vel=10, acc=10) 
        movel(place_tilt, vel=VELOCITY, acc=ACC)

    step("Finish: Move to JReady")
    movej(JReady, vel=VELOCITY, acc=ACC)

# ============================================================
# Main
# ============================================================
def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("tile_place_2x2_nm", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    initialize_robot()
    perform_task(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
