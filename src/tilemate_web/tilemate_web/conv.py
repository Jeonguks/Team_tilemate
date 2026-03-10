#!/usr/bin/env python3
"""
test_full_flow.py
─────────────────────────────────────────────────────────
firebase_bridge.py 의 시멘트 대기 흐름만 단독 테스트합니다.
패턴 선택과 시작은 웹(index.html)에서 하고,
이 노드는 Firebase를 감시하다가 tile_step 1→2 전환 시
TTS + STT 대기 흐름을 실행합니다.

실행 순서:
  1) python3 test_full_flow.py 실행
  2) 웹에서 시작 버튼 클릭 → 패턴 선택
  3) 노드가 tile_step 1→2 전환 감지 → 시멘트 TTS
  4) "끝났어" 말하면 → Step 2 재개

의존성:
  pip install openai sounddevice scipy gtts pygame python-dotenv firebase-admin
"""

import os
import sys
import time
import tempfile

from dotenv import load_dotenv

load_dotenv(dotenv_path=".env")
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY", "")

# ── Firebase ──────────────────────────────────────────
try:
    import firebase_admin
    from firebase_admin import credentials, db as firebase_db

    SERVICE_ACCOUNT_KEY_PATH = os.path.expanduser(
        "~/Team_tilemate/src/tilemate_web/config/co1-tiling-firebase-adminsdk-fbsvc-f4f88c3832.json"
    )
    DATABASE_URL = "https://co1-tiling-default-rtdb.asia-southeast1.firebasedatabase.app"

    if os.path.exists(SERVICE_ACCOUNT_KEY_PATH):
        if not firebase_admin._apps:
            cred = credentials.Certificate(SERVICE_ACCOUNT_KEY_PATH)
            firebase_admin.initialize_app(cred, {"databaseURL": DATABASE_URL})
        ref     = firebase_db.reference("/robot_status")
        cmd_ref = firebase_db.reference("/robot_command")
        FIREBASE_AVAILABLE = True
        print("[INIT] Firebase 연결 성공")
    else:
        FIREBASE_AVAILABLE = False
        ref = cmd_ref = None
        print("[INIT] Firebase key 없음 -> 로컬 시뮬레이션 모드")
except ImportError:
    FIREBASE_AVAILABLE = False
    ref = cmd_ref = None
    print("[INIT] firebase-admin 없음. pip install firebase-admin")

# ── TTS ───────────────────────────────────────────────
try:
    from gtts import gTTS
    import pygame
    pygame.mixer.init()
    TTS_AVAILABLE = True
    print("[INIT] TTS (gTTS + pygame) 준비됨")
except ImportError:
    TTS_AVAILABLE = False
    print("[INIT] gTTS / pygame 없음 -> TTS 출력 생략")

# ── STT ───────────────────────────────────────────────
try:
    _this_dir = os.path.dirname(os.path.abspath(__file__))
    if _this_dir not in sys.path:
        sys.path.insert(0, _this_dir)
    from STT import STT as _STT
    STT_AVAILABLE = True
    print("[INIT] STT (Whisper) 준비됨")
except ImportError:
    STT_AVAILABLE = False
    print("[INIT] STT.py 없음 -> 콘솔 입력 모드로 대체")


# ══════════════════════════════════════════════════════
# 헬퍼
# ══════════════════════════════════════════════════════

def speak(text: str):
    print(f"[TTS] '{text}'")
    if not TTS_AVAILABLE:
        return
    try:
        with tempfile.NamedTemporaryFile(suffix=".mp3", delete=False) as f:
            tmp_path = f.name
        gTTS(text=text, lang="ko").save(tmp_path)
        pygame.mixer.music.load(tmp_path)
        pygame.mixer.music.play()
        while pygame.mixer.music.get_busy():
            time.sleep(0.05)
        os.remove(tmp_path)
    except Exception as e:
        print(f"[TTS] 오류: {e}")


def fb_update(data: dict):
    if FIREBASE_AVAILABLE and ref:
        ref.update(data)
    else:
        print(f"  [FB_STATUS] {data}")


def fb_cmd_send(data: dict):
    if FIREBASE_AVAILABLE and cmd_ref:
        cmd_ref.update(data)
    else:
        print(f"  [FB_CMD] {data}")


def get_status() -> dict:
    if FIREBASE_AVAILABLE and ref:
        return ref.get() or {}
    return {}


def get_command() -> dict:
    if FIREBASE_AVAILABLE and cmd_ref:
        return cmd_ref.get() or {}
    return {}


# ══════════════════════════════════════════════════════
# 시멘트 대기 흐름 (firebase_bridge._cement_wait_flow 와 동일)
# ══════════════════════════════════════════════════════

def cement_wait_flow(stt):
    DONE_KEYWORDS = ["끝났어", "끝났다", "다 됐어", "완료", "됐어", "다됐어"]

    speak("타일에 시멘트를 발라주세요")
    fb_update({
        "state": "시멘트 작업 대기 중 - '끝났어' 라고 말해주세요",
        "cement_state": "waiting",
    })
    print("[CEMENT] 시멘트를 다 바르면 '끝났어'라고 말해주세요.\n")

    recognized = False

    if stt is None:
        while not recognized:
            user_input = input("  > 말할 내용 입력 (예: 끝났어): ").strip()
            if any(kw in user_input for kw in DONE_KEYWORDS):
                recognized = True
            else:
                print("  [CEMENT] 키워드 미감지. 다시 입력하세요.")
                speak("아직 시멘트 작업 중인가요? 끝나면 끝났어라고 말해주세요")
    else:
        while not recognized:
            try:
                print("[CEMENT] 음성 청취 중...")
                text = stt.speech2text()
                print(f"[CEMENT] STT 결과: '{text}'")
                if any(kw in text for kw in DONE_KEYWORDS):
                    recognized = True
                else:
                    print("[CEMENT] 키워드 미감지. 재청취...")
                    speak("아직 시멘트 작업 중인가요? 끝나면 끝났어라고 말해주세요")
            except Exception as e:
                print(f"[CEMENT] STT 오류: {e}")
                time.sleep(1.0)

    speak("작업을 재개할게")
    fb_update({"state": "시멘트 완료 - Step 2 재개", "cement_state": "done"})
    fb_cmd_send({"action": "cement_done", "timestamp": int(time.time() * 1000)})
    print("[CEMENT] Step 2 (타일 부착) 재개")


# ══════════════════════════════════════════════════════
# Firebase 감시 루프
# ══════════════════════════════════════════════════════

def watch_loop(stt):
    """
    Firebase robot_status 를 폴링.
    웹에서 시작 버튼이 눌릴 때까지 대기하고,
    tile_step 1->2 전환 시 시멘트 흐름 실행.
    """
    print("\n[WATCH] Firebase 감시 시작...")
    print("[WATCH] 웹에서 시작 버튼을 눌러주세요.\n")

    last_tile_step  = -1
    cement_triggered = False
    job_started      = False
    last_action      = ""

    while True:
        try:
            status = get_status()
            cmd    = get_command()

            action    = cmd.get("action", "idle") if isinstance(cmd, dict) else "idle"
            tile_step = int(status.get("tile_step", 0))
            overall   = int(status.get("current_step", 0))

            # 웹에서 start 눌렀을 때 (action이 start로 바뀌는 순간 1회)
            if action == "start" and last_action != "start":
                job_started      = True
                cement_triggered = False
                last_tile_step   = -1
                print("[WATCH] 웹 시작 감지! 작업 모니터링 시작")

            # 리셋 or 완료 시 초기화
            if action in ("reset",) or (job_started and overall == 0 and action == "idle"):
                if job_started:
                    print("[WATCH] 작업 종료/리셋 감지. 대기 상태로 복귀...")
                job_started      = False
                cement_triggered = False
                last_tile_step   = -1

            # tile_step 1->2 전환 감지 -> 시멘트 흐름
            if (
                job_started
                and tile_step == 2
                and last_tile_step == 1
                and not cement_triggered
            ):
                cement_triggered = True
                print("[WATCH] tile_step 1->2 전환 감지 -> 시멘트 흐름 실행")
                cement_wait_flow(stt)
                print("[WATCH] 시멘트 완료. 계속 감시 중...")

            # 타일 하나 완료(step==5)되면 다음 타일을 위해 트리거 리셋
            if tile_step == 5 and last_tile_step != 5:
                cement_triggered = False
                print("[WATCH] 타일 완료. 다음 타일 대기...")

            last_action    = action
            last_tile_step = tile_step

        except Exception as e:
            print(f"[WATCH] 오류: {e}")

        time.sleep(0.3)


# ══════════════════════════════════════════════════════
# main
# ══════════════════════════════════════════════════════

def main():
    print("\n" + "="*54)
    print("  TILEMATE 시멘트 흐름 테스트 노드")
    print("  웹에서 시작 버튼을 눌러 작업을 시작하세요.")
    print("="*54)

    stt = None
    if STT_AVAILABLE and OPENAI_API_KEY:
        try:
            stt = _STT(OPENAI_API_KEY)
            print("[INIT] STT 초기화 완료 (실제 마이크 사용)")
        except Exception as e:
            print(f"[INIT] STT 초기화 실패: {e} -> 콘솔 입력 모드")
    else:
        print("[INIT] STT 없음 -> '끝났어' 직접 타이핑으로 대체")

    try:
        watch_loop(stt)
    except KeyboardInterrupt:
        print("\n[WATCH] 종료됨 (Ctrl+C)")


if __name__ == "__main__":
    main()