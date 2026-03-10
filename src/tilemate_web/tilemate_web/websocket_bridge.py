# websocket_bridge.py  (로봇 PC에서 실행)
# ROS2 카메라 토픽 → WebSocket 실시간 스트리밍
#
# 구독 토픽:
#   /camera/camera/color/image_raw        (sensor_msgs/Image)    → JPEG 압축 → binary 전송
#   /camera/camera/depth/image_rect_raw   (sensor_msgs/Image)    → PNG + 메타데이터 → binary 전송
#   /camera/camera/depth/color/points     (sensor_msgs/PointCloud2) → JSON → text 전송
#
# 프로토콜:
#   헤더 5바이트 (ASCII) + 페이로드
#   "COLOR" + JPEG bytes
#   "DEPTH" + 4바이트(d_min uint16 LE) + 4바이트(d_max uint16 LE) + PNG bytes
#   "POINT" + JSON bytes (UTF-8)
#
# 실행:
#   pip install websockets opencv-python
#   python3 websocket_bridge.py
#
# 포트: 8765 (index.html의 WS_URL과 일치해야 함)

import asyncio
import threading
import math
import struct
import json
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2

import cv2
import numpy as np
import websockets

# ── 설정 ──────────────────────────────────────────────────────
DEPTH_CLIP_MIN = 100   # mm - 깊이 시각화 최솟값
DEPTH_CLIP_MAX = 500   # mm - 깊이 시각화 최댓값
WS_HOST = "0.0.0.0"   # 모든 인터페이스에서 수신
WS_PORT = 8765

COLOR_FPS_LIMIT  = 10.0   # 최대 10fps (0.1초 간격)
DEPTH_FPS_LIMIT  =  5.0   # 최대  5fps (0.2초 간격)
PC_FPS_LIMIT     =  1.0   # 최대  1fps (1초 간격)

COLOR_JPEG_QUALITY = 75   # JPEG 품질 (0~100)
PC_MAX_POINTS      = 8000  # 포인트 클라우드 최대 샘플 수

# ── 연결된 클라이언트 관리 ─────────────────────────────────────
clients: set = set()
clients_lock = threading.Lock()

# ── asyncio 이벤트 루프 (WebSocket 서버용) ──────────────────────
loop: asyncio.AbstractEventLoop = None


def broadcast_sync(data: bytes | str):
    """ROS 스레드에서 호출 → asyncio 루프로 안전하게 전달."""
    if loop is None:
        return
    asyncio.run_coroutine_threadsafe(_broadcast(data), loop)


async def _broadcast(data: bytes | str):
    with clients_lock:
        targets = set(clients)
    if not targets:
        return
    dead = set()
    for ws in targets:
        try:
            await ws.send(data)
        except Exception:
            dead.add(ws)
    if dead:
        with clients_lock:
            clients.difference_update(dead)


async def ws_handler(websocket, path=None):
    """WebSocket 연결 핸들러."""
    addr = websocket.remote_address
    with clients_lock:
        clients.add(websocket)
    print(f"[WS] 연결: {addr}  (총 {len(clients)}명)")
    try:
        async for _ in websocket:
            pass  # 클라이언트 → 서버 메시지는 무시
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        with clients_lock:
            clients.discard(websocket)
        print(f"[WS] 해제: {addr}  (총 {len(clients)}명)")


# ── ROS2 노드 ─────────────────────────────────────────────────
class WsImageBridgeNode(Node):
    def __init__(self):
        super().__init__("ws_image_bridge")

        self._last_color = 0.0
        self._last_depth = 0.0
        self._last_pc    = 0.0

        self.create_subscription(
            Image,
            "/camera/camera/color/image_raw",
            self._cb_color,
            10,
        )
        self.create_subscription(
            Image,
            "/camera/camera/depth/image_rect_raw",
            self._cb_depth,
            10,
        )
        self.create_subscription(
            PointCloud2,
            "/camera/camera/depth/color/points",
            self._cb_points,
            10,
        )
        self.get_logger().info(f"WebSocket bridge 시작 → ws://{WS_HOST}:{WS_PORT}")

    # ── 컬러 이미지 ──────────────────────────────────────────
    def _cb_color(self, msg: Image):
        now = time.time()
        if now - self._last_color < 1.0 / COLOR_FPS_LIMIT:
            return
        self._last_color = now

        try:
            # ROS Image → numpy (bgr8 또는 rgb8)
            dtype = np.uint8
            img = np.frombuffer(bytes(msg.data), dtype=dtype).reshape(
                msg.height, msg.width, -1
            )
            if msg.encoding == "rgb8":
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            elif msg.encoding == "bgra8":
                img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)

            # JPEG 압축
            _, buf = cv2.imencode(
                ".jpg", img,
                [cv2.IMWRITE_JPEG_QUALITY, COLOR_JPEG_QUALITY]
            )
            payload = b"COLOR" + buf.tobytes()
            broadcast_sync(payload)
            self.get_logger().debug(
                f"[COLOR] {msg.width}x{msg.height} → {len(payload)} bytes"
            )
        except Exception as e:
            self.get_logger().error(f"[COLOR] error: {e}")

    # ── 깊이 이미지 ──────────────────────────────────────────
    def _cb_depth(self, msg: Image):
        now = time.time()
        if now - self._last_depth < 1.0 / DEPTH_FPS_LIMIT:
            return
        self._last_depth = now

        try:
            # 16UC1 깊이 이미지 (row_step 기준 reshape → 패딩 있어도 안전)
            raw_depth = np.frombuffer(bytes(msg.data), dtype=np.uint8)
            depth = np.zeros((msg.height, msg.width), dtype=np.uint16)
            for r in range(msg.height):
                row_start = r * msg.step
                depth[r] = raw_depth[row_start: row_start + msg.width * 2].view(np.uint16)

            # 유효 픽셀만 사용해 실제 범위 계산 (메타데이터용)
            valid = depth[depth > 0]
            if valid.size == 0:
                return
            d_min = int(valid.min())
            d_max = int(valid.max())

            # 시각화용: 관심 범위(DEPTH_CLIP_MIN~DEPTH_CLIP_MAX)로 클리핑 후 컬러맵 변환
            depth_f = depth.astype(np.float32)
            depth_f[depth == 0] = 0
            clip_min = float(DEPTH_CLIP_MIN)
            clip_max = float(DEPTH_CLIP_MAX)
            norm = ((depth_f - clip_min) / (clip_max - clip_min) * 255).clip(0, 255).astype(np.uint8)
            colored = cv2.applyColorMap(norm, cv2.COLORMAP_JET)
            # 무효 픽셀 및 범위 밖 픽셀은 검정으로
            colored[depth == 0] = 0
            colored[depth < DEPTH_CLIP_MIN] = 0
            colored[depth > DEPTH_CLIP_MAX] = 0

            # PNG 압축
            _, buf = cv2.imencode(".png", colored)

            # 헤더: "DEPTH" + d_min(4B LE) + d_max(4B LE) + PNG
            meta = struct.pack("<II", d_min, d_max)
            payload = b"DEPTH" + meta + buf.tobytes()
            broadcast_sync(payload)
            self.get_logger().debug(
                f"[DEPTH] {msg.width}x{msg.height} d={d_min}~{d_max}mm → {len(payload)} bytes"
            )
        except Exception as e:
            self.get_logger().error(f"[DEPTH] error: {e}")

    # ── 포인트 클라우드 ──────────────────────────────────────
    def _cb_points(self, msg: PointCloud2):
        now = time.time()
        if now - self._last_pc < 1.0 / PC_FPS_LIMIT:
            return
        self._last_pc = now

        try:
            point_step = msg.point_step
            raw        = bytes(msg.data)
            total_pts  = msg.width * msg.height

            ox, oy, oz, orgb = 0, 4, 8, 16  # x, y, z, rgb 오프셋

            step = max(1, total_pts // PC_MAX_POINTS)
            xs, ys, zs, rs, gs, bs = [], [], [], [], [], []

            for i in range(0, total_pts, step):
                base = i * point_step
                if base + point_step > len(raw):
                    break

                x = struct.unpack_from("<f", raw, base + ox)[0]
                y = struct.unpack_from("<f", raw, base + oy)[0]
                z = struct.unpack_from("<f", raw, base + oz)[0]

                # NaN / Inf / 무효 깊이 제거
                if not math.isfinite(x) or not math.isfinite(y) or not math.isfinite(z):
                    continue
                if z <= 0 or z > 5.0:
                    continue

                rgb_f = struct.unpack_from("<f", raw, base + orgb)[0]
                rgb_i = struct.unpack("<I", struct.pack("<f", rgb_f))[0]
                r = (rgb_i >> 16) & 0xFF
                g = (rgb_i >> 8)  & 0xFF
                b =  rgb_i        & 0xFF

                xs.append(round(float(x), 4))
                ys.append(round(float(y), 4))
                zs.append(round(float(z), 4))
                rs.append(int(r))
                gs.append(int(g))
                bs.append(int(b))

            if not xs:
                self.get_logger().warn("[PC] 유효 포인트 없음")
                return

            data = {
                "x": xs, "y": ys, "z": zs,
                "r": rs, "g": gs, "b": bs,
                "count": len(xs),
                "timestamp": int(now * 1000),
            }
            payload = b"POINT" + json.dumps(data, separators=(",", ":")).encode("utf-8")
            broadcast_sync(payload)
            self.get_logger().info(f"[PC] {len(xs)}pts → WebSocket")

        except Exception as e:
            self.get_logger().error(f"[PC] error: {e}")


# ── ROS2 스핀 스레드 ──────────────────────────────────────────
def ros_spin(node: WsImageBridgeNode):
    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"[ROS] spin error: {e}")


# ── 메인 ─────────────────────────────────────────────────────
def main():
    global loop

    rclpy.init()
    node = WsImageBridgeNode()

    # asyncio 이벤트 루프를 먼저 생성 (broadcast_sync가 참조하므로)
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    # ROS2는 별도 스레드에서 spin
    ros_thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    ros_thread.start()

    print(f"[WS] 서버 시작: ws://{WS_HOST}:{WS_PORT}")
    print("[WS] Ctrl+C 로 종료")

    async def run_server():
        async with websockets.serve(ws_handler, WS_HOST, WS_PORT, max_size=10 * 1024 * 1024):
            await asyncio.Future()  # 영원히 대기

    try:
        loop.run_until_complete(run_server())
    except KeyboardInterrupt:
        print("\n[WS] 종료 중...")
    finally:
        loop.close()
        rclpy.shutdown()


if __name__ == "__main__":
    main()