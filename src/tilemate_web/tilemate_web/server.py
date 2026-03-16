#!/usr/bin/env python3
import argparse
import asyncio
import json
import os
from datetime import datetime
from pathlib import Path
from typing import Any, Dict

from fastapi import FastAPI, Request
from fastapi.responses import FileResponse, JSONResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
import uvicorn

BASE_DIR = Path(__file__).resolve().parent
DATA_DIR = BASE_DIR / "data"
DATA_DIR.mkdir(exist_ok=True)

LATEST_PATH = DATA_DIR / "latest.json"
HISTORY_DIR = DATA_DIR / "history"
HISTORY_DIR.mkdir(exist_ok=True)


def _resolve_static_dir() -> Path | None:
    candidates = [
        BASE_DIR / "static",
        BASE_DIR.parent / "static",
        BASE_DIR.parents[1] / "src" / "tilemate_web" / "tilemate_web" / "static",
    ]

    try:
        from ament_index_python.packages import get_package_share_directory

        candidates.append(Path(get_package_share_directory("tilemate_web")) / "static")
    except Exception:
        pass

    for candidate in candidates:
        if candidate.exists() and candidate.is_dir():
            return candidate

    return None

def _resolve_config_dir() -> Path:
    candidates = [
        BASE_DIR / "config",
        BASE_DIR.parent / "config",
        BASE_DIR.parents[1] / "src" / "tilemate_web" / "config",
    ]

    try:
        from ament_index_python.packages import get_package_share_directory

        candidates.append(Path(get_package_share_directory("tilemate_web")) / "config")
    except Exception:
        pass

    for candidate in candidates:
        if candidate.exists():
            return candidate

    return BASE_DIR.parent / "config"


CONFIG_DIR = _resolve_config_dir()
WALL_JSON_PATH = CONFIG_DIR / "wall.json"
DUMMY_PATH = CONFIG_DIR / "dummy.json"


def _fallback_dummy_payload() -> Dict[str, Any]:
    return {
        "success": True,
        "message": "inspect_dummy_fallback",
        "timestamp_sec": datetime.now().timestamp(),
        "tiles": [
            {
                "name": "pattern_3",
                "conf_score": 0.95,
                "center_uv": [488.0, 411.0],
                "size_uv": [144.0, 146.0],
                "rpy_deg": [0.0, 0.0, 1.7],
                "plane_normal": [0.0, 0.0, 1.0],
                "plane_d": -381.0,
                "plane_centroid_mm": [-58.5, 14.5, 380.1],
                "base_center_mm": [288.8, 271.8, 179.2],
                "anomaly_score": 0.15,
            },
            {
                "name": "pattern_5",
                "conf_score": 0.99,
                "center_uv": [669.0, 419.0],
                "size_uv": [145.5, 147.4],
                "rpy_deg": [0.0, 0.0, 1.7],
                "plane_normal": [0.0, 0.0, 1.0],
                "plane_d": -381.0,
                "plane_centroid_mm": [17.2, 18.0, 380.4],
                "base_center_mm": [364.3, 272.9, 176.5],
                "anomaly_score": 0.47,
            },
        ],
    }


app = FastAPI(title="Wall Tile Inspection Web")

STATIC_DIR = _resolve_static_dir()
if STATIC_DIR is not None:
    app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")
else:
    print("[tilemate_web.server] WARNING: static directory not found; /static will be unavailable")

DEFAULT_HOST = os.getenv("TILEMATE_WEB_HOST", "0.0.0.0")
DEFAULT_PORT = int(os.getenv("TILEMATE_WEB_PORT", "8000"))

# SSE 구독자별 큐
subscribers: list[asyncio.Queue] = []


@app.get("/")
async def index():
    return FileResponse(str(BASE_DIR / "index.html"))

@app.get("/report")
async def report():
    return FileResponse(str(BASE_DIR / "report_page.html"))


@app.get("/wall.json")
async def get_wall_json():
    if not WALL_JSON_PATH.exists():
        return JSONResponse(
            {
                "success": False,
                "message": "wall.json not found",
                "wall": None,
            },
            status_code=404,
        )

    return FileResponse(
        str(WALL_JSON_PATH),
        media_type="application/json",
        filename="wall.json",
    )
@app.post("/api/inspect/dummy")
async def generate_dummy():
    if DUMMY_PATH.exists():
        with open(DUMMY_PATH, "r", encoding="utf-8") as f:
            payload = json.load(f)
    else:
        payload = _fallback_dummy_payload()

    now = datetime.now()
    ts = now.strftime("%Y%m%d_%H%M%S_%f")
    history_path = HISTORY_DIR / f"inspect_{ts}.json"

    with open(LATEST_PATH, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)

    with open(history_path, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)

    event_data = {
        "type": "inspect_updated",
        "timestamp": ts,
        "payload": payload,
    }

    dead_queues = []
    for q in subscribers[:]:
        try:
            q.put_nowait(event_data)
        except Exception:
            dead_queues.append(q)

    for q in dead_queues:
        if q in subscribers:
            subscribers.remove(q)

    return {
        "success": True,
        "message": "dummy inspection result loaded",
        "latest_path": str(LATEST_PATH),
        "history_path": str(history_path),
    }


@app.post("/api/inspect/clear")
async def clear_latest_inspection():
    removed = False
    try:
        if LATEST_PATH.exists():
            LATEST_PATH.unlink()
            removed = True
    except Exception as exc:
        return JSONResponse(
            {
                "success": False,
                "message": f"failed to clear latest.json: {exc}",
            },
            status_code=500,
        )

    event_data = {
        "type": "inspect_cleared",
        "timestamp": datetime.now().strftime("%Y%m%d_%H%M%S_%f"),
        "payload": {
            "success": True,
            "message": "no inspection result yet",
            "frame_id": "-",
            "timestamp_sec": 0,
            "wall": None,
            "tiles": [],
        },
    }

    dead_queues = []
    for q in subscribers[:]:
        try:
            q.put_nowait(event_data)
        except Exception:
            dead_queues.append(q)

    for q in dead_queues:
        if q in subscribers:
            subscribers.remove(q)

    return {
        "success": True,
        "message": "inspection cache cleared",
        "removed": removed,
    }


@app.get("/api/inspect/latest")
async def get_latest():
    if LATEST_PATH.exists():
        with open(LATEST_PATH, "r", encoding="utf-8") as f:
            return JSONResponse(json.load(f))

    return JSONResponse(
        {
            "success": False,
            "message": "no inspection result yet",
            "frame_id": "-",
            "timestamp_sec": 0,
            "wall": None,
            "tiles": [],
        },
        status_code=200,
    )
# @app.get("/api/inspect/latest")
# async def get_latest():
#     # if not LATEST_PATH.exists():
#     #     return JSONResponse(
#     #         {
#     #             "success": False,
#     #             "message": "no inspection result yet",
#     #             "frame_id": "-",
#     #             "timestamp_sec": 0,
#     #             "wall": None,
#     #             "tiles": [],
#     #         },
#     #         status_code=200,
#     #     )

#     ###########################################DEBUG MODE##############################33
#     # latest.json 존재하면 그것 사용
#     if LATEST_PATH.exists():
#         with open(LATEST_PATH, "r", encoding="utf-8") as f:
#             data = json.load(f)
#         return JSONResponse(data)

#     # 없으면 dummy.json 사용
#     if DUMMY_PATH.exists():
#         with open(DUMMY_PATH, "r", encoding="utf-8") as f:
#             data = json.load(f)
#         return JSONResponse(data)
#     ####################################################################################

#     with open(LATEST_PATH, "r", encoding="utf-8") as f:
#         data = json.load(f)

#     return JSONResponse(data)


@app.get("/api/inspect/events")
async def inspect_events():
    """
    브라우저가 EventSource로 접속하는 SSE 엔드포인트
    """
    queue: asyncio.Queue = asyncio.Queue()
    subscribers.append(queue)

    async def event_generator():
        try:
            yield "event: connected\ndata: {\"success\": true}\n\n"

            while True:
                msg = await queue.get()
                event_name = msg.get("type", "message")
                yield f"event: {event_name}\ndata: {json.dumps(msg, ensure_ascii=False)}\n\n"
        except asyncio.CancelledError:
            pass
        finally:
            if queue in subscribers:
                subscribers.remove(queue)

    return StreamingResponse(
        event_generator(),
        media_type="text/event-stream",
        headers={
            "Cache-Control": "no-cache",
            "Connection": "keep-alive",
            "X-Accel-Buffering": "no",
        },
    )


@app.post("/api/inspect/result")
async def post_inspect_result(request: Request):
    payload: Dict[str, Any] = await request.json()

    now = datetime.now()
    ts = now.strftime("%Y%m%d_%H%M%S_%f")
    history_path = HISTORY_DIR / f"inspect_{ts}.json"

    with open(LATEST_PATH, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)

    with open(history_path, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)

    event_data = {
        "type": "inspect_updated",
        "timestamp": ts,
        "payload": payload,
    }

    for q in subscribers[:]:
        try:
            q.put_nowait(event_data)
        except Exception:
            if q in subscribers:
                subscribers.remove(q)

    return {
        "success": True,
        "message": "inspection result saved",
        "latest_path": str(LATEST_PATH),
        "history_path": str(history_path),
    }


def main():
    parser = argparse.ArgumentParser(description="Tilemate FastAPI server")
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--reload", action="store_true")
    args = parser.parse_args()

    uvicorn.run(
        "tilemate_web.server:app",
        host=args.host,
        port=args.port,
        reload=args.reload,
        reload_excludes=["keyword_extraction.py", "STT.py"],
    )


if __name__ == "__main__":
    main()

#host="192.168.10.48"