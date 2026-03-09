#!/usr/bin/env python3
# firebase_monitor.py
# Firebase robot_command / robot_status 실시간 모니터링

import os
import time
from dotenv import load_dotenv

load_dotenv(dotenv_path=".env")

import firebase_admin
from firebase_admin import credentials, db

SERVICE_ACCOUNT_KEY_PATH = os.path.expanduser(
    "~/Team_tilemate/src/tilemate_web/config/co1-tiling-firebase-adminsdk-fbsvc-f4f88c3832.json"
)
DATABASE_URL = "https://co1-tiling-default-rtdb.asia-southeast1.firebasedatabase.app"

if not firebase_admin._apps:
    cred = credentials.Certificate(SERVICE_ACCOUNT_KEY_PATH)
    firebase_admin.initialize_app(cred, {"databaseURL": DATABASE_URL})

cmd_ref    = db.reference("/robot_command")
status_ref = db.reference("/robot_status")

print("=" * 60)
print("Firebase 모니터 시작 (Ctrl+C 종료)")
print("=" * 60)

prev_cmd    = None
prev_layout = None

while True:
    try:
        cmd = cmd_ref.get()
        if cmd != prev_cmd:
            prev_cmd = cmd
            print("\n[robot_command 변경]")
            if isinstance(cmd, dict):
                for k, v in cmd.items():
                    print(f"  {k}: {v}")
            else:
                print(f"  {cmd}")

        layout = status_ref.child("stt_layout").get()
        if layout != prev_layout:
            prev_layout = layout
            print("\n[robot_status/stt_layout 변경]")
            print(f"  layout: {layout}")
            if layout:
                symbols = ["⬛" if v == 1 else "⬜" for v in layout]
                for row in range(3):
                    print("  " + "".join(symbols[row*3:row*3+3]))

    except Exception as e:
        print(f"[ERROR] {e}")

    time.sleep(0.3)
