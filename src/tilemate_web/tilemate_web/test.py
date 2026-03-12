#!/usr/bin/env python3
"""
test_inspection_node.py
─────────────────────────────────────────────────────────────
ROS2 토픽을 직접 퍼블리시해서
tile_step 4 진입 시 단차 검수 모달 자동 오픈을 테스트합니다.

실제 firebase_bridge 노드가 실행 중인 상태에서 사용하세요.
(firebase_bridge가 토픽을 받아 Firebase에 반영 → 웹 모달 오픈)

사용법:
    # 터미널 1: firebase_bridge 실행
    ros2 run tilemate_web firebase_bridge

    # 터미널 2: 이 테스트 노드 실행
    python3 test_inspection_node.py

퍼블리시하는 토픽:
    /robot/state          (std_msgs/String)   → 로봇 상태 문자열
    /dsr01/joint_states   는 건드리지 않음

    ※ tile_step은 ExecuteJob 액션 피드백으로 전달되는 값이라
      이 노드에서는 Firebase를 직접 update해서 tile_step을 흉내냅니다.
      (ROS2 액션 없이 tile_step만 테스트하는 용도)
"""

import json
import os
import sys
import time
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Float32

import firebase_admin
from firebase_admin import credentials, db

# ── 경로 설정 ──────────────────────────────────────────────
SERVICE_ACCOUNT_KEY_PATH = os.path.expanduser(
    "~/Team_tilemate/src/tilemate_web/config/co1-tiling-firebase-adminsdk-fbsvc-f4f88c3832.json"
)
DATABASE_URL = "https://co1-tiling-default-rtdb.asia-southeast1.firebasedatabase.app"

INSPECTION_JSON_PATH = os.path.expanduser(
    "~/Team_tilemate/src/tilemate_web/config/wall_tile_inspection_result.json"
)


# ── Firebase 초기화 ────────────────────────────────────────
def init_firebase():
    if not os.path.exists(SERVICE_ACCOUNT_KEY_PATH):
        print(f"[ERROR] Firebase 키 파일 없음: {SERVICE_ACCOUNT_KEY_PATH}")
        sys.exit(1)
    if not firebase_admin._apps:
        cred = credentials.Certificate(SERVICE_ACCOUNT_KEY_PATH)
        firebase_admin.initialize_app(cred, {"databaseURL": DATABASE_URL})
    return db.reference("/robot_status")


def load_inspection_json():
    """inspection result JSON 로드. 파일 없으면 내장 샘플 반환."""
    if os.path.exists(INSPECTION_JSON_PATH):
        with open(INSPECTION_JSON_PATH, "r") as f:
            data = json.load(f)
        print(f"  [JSON] 파일 로드: {INSPECTION_JSON_PATH}")
        return data

    print("  [JSON] 파일 없음 → 내장 샘플 데이터 사용")
    return {
        "frame_id": "test_frame",
        "timestamp_sec": time.time(),
        "wall": {
            "name": "wall", "roi": [248, 87, 597, 360],
            "sample_points": [[-0.15, -0.13, 0.392], [0.15, -0.13, 0.391], [0.0, 0.1, 0.391]],
            "centroid": [-0.003, -0.011, 0.391],
            "normal": [-0.0115, 0.0129, 0.9999],
            "center_point": [-0.004, -0.010, 0.386],
            "patch_vertices": [
                [-0.153, -0.127, 0.391], [0.147, -0.127, 0.395],
                [0.147,  0.104, 0.392], [-0.153,  0.104, 0.388],
            ],
            "patch_indices": [0, 1, 2, 0, 2, 3],
            "rmse_mm": 2.186, "roll_deg": -0.74,
            "pitch_deg": -0.66, "yaw_deg": 0.01,
            "color": [1.0, 0.9, 0.1],
        },
        "tiles": [{
            "name": "1", "roi": [401, 233, 449, 279],
            "sample_points": [[-0.013, 0.007, 0.382], [0.011, 0.029, 0.382]],
            "centroid": [-0.002, 0.018, 0.381],
            "normal": [0.0238, -0.0487, 0.9985],
            "center_point": [-0.001, 0.019, 0.382],
            "patch_vertices": [
                [-0.013, 0.007, 0.381], [0.011, 0.007, 0.381],
                [0.011, 0.030, 0.382], [-0.013, 0.030, 0.382],
            ],
            "patch_indices": [0, 1, 2, 0, 2, 3],
            "rmse_mm": 1.342, "roll_deg": 2.80,
            "pitch_deg": 1.36, "yaw_deg": 0.07,
            "color": [0.1, 1.0, 0.2],
        }],
    }


# ── 테스트 ROS2 노드 ───────────────────────────────────────
class InspectionTestNode(Node):
    def __init__(self, firebase_ref):
        super().__init__("inspection_test_node")
        self.ref = firebase_ref

        # firebase_bridge가 구독하는 토픽들
        self._pub_state = self.create_publisher(String, "/robot/state", 10)
        self._pub_step  = self.create_publisher(Int32,  "/robot/step",  10)

        self.get_logger().info("InspectionTestNode 시작됨")
        self.get_logger().info("퍼블리시 토픽: /robot/state, /robot/step")

    def pub_state(self, text: str):
        msg = String()
        msg.data = text
        self._pub_state.publish(msg)
        self.get_logger().info(f"[PUB] /robot/state → '{text}'")

    def pub_step(self, step: int):
        msg = Int32()
        msg.data = step
        self._pub_step.publish(msg)
        self.get_logger().info(f"[PUB] /robot/step → {step}")

    def set_tile_step(self, step: int, label: str, keep_working_tile: bool = False):
        update_data = {
            "tile_step":    step,
            "current_step": 1,
            "state":        label,
        }
        if not keep_working_tile:
            update_data["working_tile"] = 0
        self.ref.update(update_data)
        self.pub_state(label)
        print(f"  → tile_step={step}  ({label})")
        time.sleep(0.5)  # 웹이 Firebase 업데이트를 수신할 시간 확보

    def upload_inspection_result(self):
        data = load_inspection_json()
        self.ref.update({"inspection_result": data})
        print("  → Firebase inspection_result 업로드 완료 ✅")

    def run_auto_scenario(self):
        """PICK → COWORK → PLACE → INSPECT(+JSON) → 확인 → COMPACT 자동 시나리오"""
        print("\n[자동 시나리오 시작] 웹 브라우저 준비하세요!\n")
        time.sleep(1.5)

        steps = [
            (1, "타일 파지 중",   2.0),
            (2, "시멘트 도포 중", 2.0),
            (3, "타일 부착 중",   2.0),
        ]
        for step, label, delay in steps:
            self.set_tile_step(step, label)
            time.sleep(delay)

        # INSPECT 진입 → working_tile=0 유지, tile_step=4 먼저 확실히 전송
        print("\n  ▶ INSPECT 단계 진입 → 모달이 열려야 합니다!")
        self.set_tile_step(4, "단차 검수 중")  # 내부에서 0.5초 대기
        time.sleep(1.0)  # 웹이 tile_step=4를 확실히 수신하도록 추가 대기

        print("  ▶ inspection_result 업로드 중...")
        self.upload_inspection_result()
        time.sleep(0.5)

        print("\n  웹에서 3D 검수 모달을 확인하세요.")
        print("  모달 확인 후 엔터를 누르면 COMPACT(5)로 진행합니다...")
        input()

        # COMPACT 진입: working_tile 유지 필수
        print("\n  ▶ COMPACT 단계 진입 → 모달 닫히고 판정 결과가 표시되어야 합니다!")
        self.set_tile_step(5, "압착 보정 중", keep_working_tile=True)
        time.sleep(1.0)
        print("  ✅ 시나리오 완료")


# ── 메뉴 루프 (별도 스레드) ────────────────────────────────
def menu_loop(node: InspectionTestNode):
    time.sleep(0.5)  # 노드 초기화 대기

    print("\n" + "=" * 55)
    print("  단차 검수 모달 ROS2 테스트 노드")
    print("  웹 브라우저에서 index.html을 열어두세요!")
    print("=" * 55)

    # 초기 상태 세팅
    node.ref.update({
        "current_step": 1,
        "tile_step":    0,
        "state":        "대기",
        "completed_jobs": 0,
        "working_tile": 0,
        "overall_progress": 0.0,
        "tile_progress":    0.0,
        "inspection_result": None,
        "design": 1,
    })
    print("[RESET] Firebase 초기화 완료\n")

    MENU = """
─── 테스트 메뉴 ───────────────────────────────
  1) PICK    (tile_step=1)
  2) COWORK  (tile_step=2)
  3) PLACE   (tile_step=3)
  4) INSPECT (tile_step=4)  ← 모달 오픈
  5) COMPACT (tile_step=5)  ← 모달 닫힘
  6) DONE    (tile_step=6)
  j) inspection_result JSON 업로드
  a) 전체 자동 시나리오 (1→2→3→4+JSON→5)
  r) 초기화
  q) 종료
───────────────────────────────────────────────"""

    STEP_LABELS = {
        1: "타일 파지 중",
        2: "시멘트 도포 중",
        3: "타일 부착 중",
        4: "단차 검수 중",
        5: "압착 보정 중",
        6: "타일 완료",
    }

    while True:
        print(MENU)
        choice = input("선택: ").strip().lower()

        if choice in "123456" and choice.isdigit():
            step = int(choice)
            # COMPACT(5)는 INSPECT 직후이므로 working_tile 유지
            keep = (step == 5)
            node.set_tile_step(step, STEP_LABELS[step], keep_working_tile=keep)
            # INSPECT 진입 시 JSON도 같이 업로드할지 확인
            if step == 4:
                ans = input("  JSON도 업로드할까요? (엔터=예 / n=아니오): ").strip().lower()
                if ans != "n":
                    node.upload_inspection_result()

        elif choice == "j":
            node.upload_inspection_result()

        elif choice == "a":
            node.run_auto_scenario()

        elif choice == "r":
            node.ref.update({
                "tile_step": 0, "current_step": 1,
                "state": "대기", "inspection_result": None,
            })
            print("  → 초기화 완료")

        elif choice == "q":
            print("\n[종료]")
            rclpy.shutdown()
            break

        else:
            print("  잘못된 입력입니다.")


# ── 엔트리포인트 ───────────────────────────────────────────
def main():
    rclpy.init()

    firebase_ref = init_firebase()
    print("[OK] Firebase 연결 완료")

    node = InspectionTestNode(firebase_ref)

    # 메뉴는 별도 스레드에서 실행 (rclpy.spin이 메인 스레드 점유)
    menu_thread = threading.Thread(target=menu_loop, args=(node,), daemon=True)
    menu_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[Ctrl+C] 종료")
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()