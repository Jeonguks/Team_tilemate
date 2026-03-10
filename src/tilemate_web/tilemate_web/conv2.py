#!/usr/bin/env python3
"""
inject_tile_step.py
────────────────────────────────────────────
TaskManager 없이 Firebase에 tile_step을 수동으로 주입해서
conv.py 의 시멘트 흐름을 테스트합니다.

별도 터미널에서 실행:
  python3 inject_tile_step.py
"""

import os
import time
from dotenv import load_dotenv

load_dotenv(dotenv_path=".env")

import firebase_admin
from firebase_admin import credentials, db as firebase_db

SERVICE_ACCOUNT_KEY_PATH = os.path.expanduser(
    "~/Team_tilemate/src/tilemate_web/config/co1-tiling-firebase-adminsdk-fbsvc-f4f88c3832.json"
)
DATABASE_URL = "https://co1-tiling-default-rtdb.asia-southeast1.firebasedatabase.app"

if not firebase_admin._apps:
    cred = credentials.Certificate(SERVICE_ACCOUNT_KEY_PATH)
    firebase_admin.initialize_app(cred, {"databaseURL": DATABASE_URL})

ref = firebase_db.reference("/robot_status")

STEP_NAMES = {
    0: "IDLE",
    1: "PICK  (타일 파지)",
    2: "PLACE (타일 부착)",   # ← conv.py가 이 전환에서 트리거
    3: "INSPECT (단차 검수)",
    4: "COMPACT (압착 보정)",
    5: "DONE",
}

def set_step(step: int):
    ref.update({
        "tile_step": step,
        "current_step": 1,
        "state": STEP_NAMES.get(step, str(step)),
    })
    print(f"  → tile_step = {step}  ({STEP_NAMES.get(step, '')})")

def main():
    print("="*50)
    print("  tile_step 수동 주입기")
    print("  conv.py가 실행 중인 상태에서 사용하세요.")
    print("="*50)

    print("\n[1] step 0→1 자동 주입 (파지 시뮬레이션)")
    set_step(0)
    time.sleep(0.5)
    set_step(1)
    print("  파지 중... (1→2 전환 시 conv.py 가 시멘트 TTS 실행)")

    input("\n  [Enter] 를 누르면 step 2 로 전환합니다 (시멘트 트리거)...")

    set_step(2)
    print("  conv.py 에서 TTS + STT 대기 시작됐으면 성공!")

    input("\n  [Enter] 를 누르면 step 3→4→5 (완료) 로 진행합니다...")

    for s in [3, 4, 5]:
        set_step(s)
        time.sleep(0.8)

    ref.update({"completed_jobs": 1, "current_step": 2, "overall_progress": 1.0, "state": "작업 완료"})
    print("\n  타일 1개 완료!")

if __name__ == "__main__":
    main()