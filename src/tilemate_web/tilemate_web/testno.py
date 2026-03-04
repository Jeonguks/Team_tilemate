"""
testno_interactive.py
TILING MONITOR 인터랙티브 테스트 노드 (8단계 공정 대응 버전)

index.html의 8단계 공정 흐름과 Step 7 단차 확인 로직을 검증합니다.
"""

import time
import random
import firebase_admin
from firebase_admin import credentials, db
import os

# Firebase 설정 (사용자 환경에 맞게 경로 확인 필요)
SERVICE_ACCOUNT_KEY_PATH = os.path.expanduser(
    "~/Team_tilemate/src/tilemate_web/config/co1-tiling-firebase-adminsdk-fbsvc-f4f88c3832.json"
)
FIREBASE_DB_URL = "https://co1-tiling-default-rtdb.asia-southeast1.firebasedatabase.app"

TOTAL_TILES = 9

# 콘솔 색상 정의
R     = "\033[0m"
BOLD  = "\033[1m"
CYAN  = "\033[96m"
YEL   = "\033[93m"
GRN   = "\033[92m"
RED   = "\033[91m"
GRAY  = "\033[90m"
BLUE  = "\033[94m"
MAG   = "\033[95m"

def c(color, text): print(f"{color}{text}{R}")

def init_firebase():
    if not os.path.exists(SERVICE_ACCOUNT_KEY_PATH):
        c(RED, f"오류: 키 파일 없음 → {SERVICE_ACCOUNT_KEY_PATH}")
        exit(1)
    if not firebase_admin._apps:
        cred = credentials.Certificate(SERVICE_ACCOUNT_KEY_PATH)
        firebase_admin.initialize_app(cred, {"databaseURL": FIREBASE_DB_URL})
    return db.reference("robot_status"), db.reference("robot_command")

def ask(prompt, default=None):
    suffix = f" [{default}]" if default is not None else ""
    val = input(f"{BOLD}  {prompt}{suffix}: {R}").strip()
    return val if val else str(default) if default is not None else ""

def ask_tile(prompt="타일 번호 (1~9, 엔터=전체)"):
    val = ask(prompt)
    if val == "": return list(range(1, TOTAL_TILES + 1))
    try:
        n = int(val)
        if 1 <= n <= TOTAL_TILES: return [n]
    except: pass
    return list(range(1, TOTAL_TILES + 1))

def delay(sec=0.3):
    time.sleep(sec)

def print_menu(design):
    print()
    c(BOLD + CYAN, "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
    c(BOLD + CYAN, "   TILING MONITOR — 8단계 공정 테스트 노드")
    c(BOLD + CYAN, f"   현재 디자인: {design}")
    c(BOLD + CYAN, "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
    c(YEL,  "  [1] Step 1  타일 파지        [2] Step 2  타일 툴에 배치")
    c(YEL,  "  [3] Step 3  스크래퍼 파지    [4] Step 4  접착제 풀기")
    c(YEL,  "  [5] Step 5  접착제 바르기    [6] Step 6  타일 배치")
    c(MAG,  "  [7] Step 7  단차 확인 (inspect_no, tile_level)")
    c(MAG,  "  [8] Step 8  압착      (press_no)")
    c(GRN,  "  ────────────────────────────────────────────")
    c(GRN,  "  [A] 전체 자동 실행 (1~8단계 전체 공정)")
    c(GRN,  "  [P] 단차 확인 후 필요 타일만 압착 (7+8 묶음)")
    c(BLUE, "  ────────────────────────────────────────────")
    c(BLUE, "  [D] 디자인 변경 | [S] 상태 조회 | [E] 비상정지 | [R] 초기화 | [Q] 종료")
    c(BOLD + CYAN, "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")

# ────────────────────────────────────────────────────────
# 공정별 함수 (index.html 로직에 최적화)
# ────────────────────────────────────────────────────────

def step_simple(sr, step_num, state_text):
    c(CYAN, f"\n[STEP {step_num}] {state_text}")
    sr.update({
        "current_step": step_num,
        "state": f"{state_text} 중",
        "joint_speed": round(random.uniform(1.0, 2.5), 3),
    })
    c(GRN, f"  → current_step={step_num} 업데이트 완료")

def step6_tile_place(sr, tiles):
    """Step 6: 타일 배치 (completed_jobs를 올려 타이머 시작)"""
    c(CYAN, f"\n[STEP 6] 타일 배치 — 대상: {tiles}")
    for t in tiles:
        sr.update({
            "current_step": 6,
            "state": f"타일 {t} 배치 중",
            "completed_jobs": t - 1,
            "joint_speed": 2.0
        })
        delay(0.5)
        sr.update({
            "completed_jobs": t,
            "state": f"타일 {t} 배치 완료",
            "joint_speed": 0
        })
        c(GRN, f"  타일 {t} 배치 완료 (completed_jobs={t}) → 웹 타이머 시작")

def step7_level_check(sr, tile_num, fixed_val=None):
    """
    Step 7: 단차 확인
    - tile_level 0.0 -> '단차 확인 중'
    - tile_level >= 1.5 -> '압착 필요'
    - tile_level < 1.5 -> '배치 완료'
    """
    c(CYAN, f"\n[STEP 7] 단차 확인 — 타일 {tile_num}")
    # 1. 측정 시작 (0 전송)
    sr.update({
        "current_step": 7,
        "state": f"타일 {tile_num} 단차 측정 중",
        "inspect_no": tile_num,
        "tile_level": 0.0,
    })
    c(YEL, f"  inspect_no={tile_num}, tile_level=0.0  →  웹: '단차 확인 중'")
    delay(0.8)

    # 2. 결과 전송
    if fixed_val is not None:
        val = fixed_val
    else:
        # 5,7,8,9번은 압착이 필요한 상황(1.5 이상)으로 랜덤 설정
        val = round(random.uniform(1.6, 2.8), 3) if tile_num in [5,7,8,9] else round(random.uniform(0.1, 1.3), 3)

    sr.update({"tile_level": val})
    
    status = f"{RED}압착 필요{R}" if val >= 1.5 else f"{GRN}배치 완료{R}"
    c(GRN, f"  타일 {tile_num} 결과: {val}mm  →  웹: {status}")
    delay(0.5)
    return val

def step8_pressing(sr, tile_num):
    """Step 8: 압착 (press_no를 전송하여 '압착 완료' 표시)"""
    c(CYAN, f"\n[STEP 8] 압착 실행 — 타일 {tile_num}")
    sr.update({
        "current_step": 8,  # index.html step8 압착
        "state": f"타일 {tile_num} 압착 중",
        "press_no": tile_num,
    })
    c(YEL, f"  press_no={tile_num} 전송  →  웹: '압착 완료'")
    delay(0.8)

# ────────────────────────────────────────────────────────
# 메인 제어 로직
# ────────────────────────────────────────────────────────

def main():
    sr, cr = init_firebase()
    c(BOLD + GRN, "\nFirebase 연결 성공!")
    design = 1

    while True:
        print_menu(design)
        choice = ask("선택").upper()

        if choice in ["1", "2", "3", "4", "5"]:
            steps = ["", "타일 파지", "타일 툴에 배치", "스크래퍼 파지", "접착제 풀기", "접착제 바르기"]
            step_simple(sr, int(choice), steps[int(choice)])

        elif choice == "6":
            tiles = ask_tile()
            step6_tile_place(sr, tiles)

        elif choice == "7":
            tiles = ask_tile()
            for t in tiles:
                step7_level_check(sr, t)

        elif choice == "8":
            tiles = ask_tile()
            for t in tiles:
                step8_pressing(sr, t)

        elif choice == "A":
            c(BOLD + GRN, "\n[전체 공정 자동 실행]")
            step_simple(sr, 1, "타일 파지"); delay(0.5)
            step_simple(sr, 2, "타일 툴에 배치"); delay(0.5)
            step_simple(sr, 3, "스크래퍼 파지"); delay(0.5)
            step_simple(sr, 4, "접착제 풀기"); delay(0.5)
            step_simple(sr, 5, "접착제 바르기"); delay(0.5)
            
            # 타일 배치 (1~9)
            for t in range(1, 10):
                step6_tile_place(sr, [t])
            
            # 단차 확인 및 필요시 압착
            for t in range(1, 10):
                val = step7_level_check(sr, t)
                if val >= 1.5:
                    step8_pressing(sr, t)
            
            sr.update({"current_step": 0, "state": "모든 공정 완료", "inspect_no": 0, "press_no": 0})
            c(BOLD + GRN, "\n🎉 전체 타일링 공정이 완료되었습니다!")

        elif choice == "P":
            tiles = ask_tile("7+8단계 대상")
            for t in tiles:
                val = step7_level_check(sr, t)
                if val >= 1.5:
                    step8_pressing(sr, t)

        elif choice == "D":
            design = int(ask("디자인 선택 (1~3)", 1))
            sr.update({"design": design})

        elif choice == "S":
            data = sr.get()
            c(BLUE, f"\n현재 상태: {data.get('state')} (Step: {data.get('current_step')})")
            c(BLUE, f"진행도: {data.get('completed_jobs')}/9 | 단차: {data.get('tile_level')}mm")

        elif choice == "E":
            sr.update({"state": "충돌 감지 - 비상정지", "collision_joint": 0, "collision_torque": 75.5})
            c(RED, "\n[비상정지] 시뮬레이션 활성화")

        elif choice == "R":
            sr.set({
                "current_step": 0, "state": "대기", "completed_jobs": 0,
                "inspect_no": 0, "tile_level": 0.0, "press_no": 0, "design": 0,
                "joint_speed": 0.0, "force_z": 0.0, "force_total": 0.0
            })
            c(GRAY, "\n초기화 완료")

        elif choice == "Q":
            break

if __name__ == "__main__":
    main()