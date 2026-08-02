"""
==============================================================================
DASHBOARD FASTAPI BACKEND SERVER
==============================================================================

Project:  Vision-Based Autonomous Robotic Arm
File:     main.py
Location: dashboard/backend/

PURPOSE:
    Asynchronous web server providing REST endpoints and WebSocket real-time
    communication for the Dashboard UI. Serves the static Warm Cream HTML/CSS/JS
    frontend and interfaces directly with serial_manager.py.

KEY ENDPOINTS:
    - GET  /api/ports             Lists available serial ports
    - POST /api/connect           Connects to a specific serial port
    - POST /api/disconnect        Disconnects from Arduino
    - POST /api/lock90            Locks all 6 servos at 90° for assembly
    - POST /api/home              Moves all servos to Home Position
    - POST /api/estop             Triggers Emergency Stop
    - POST /api/estop/reset       Resets Emergency Stop state
    - WS   /ws                    Real-time WebSocket for telemetry & slider streaming

RELATED DECISIONS:
    - Decision #11: Web-based Dashboard (FastAPI + WebSockets)
    - Decision #15: No authentication required
    - Decision #18: Warm light cream theme support
    - Decision #19: Active physical assembly tool
==============================================================================
"""

import os
import json
import asyncio
import logging
from typing import List, Dict, Any
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

from serial_manager import serial_manager

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger("DashboardBackend")

app = FastAPI(
    title="Robotic Arm Control Dashboard Backend",
    version="1.0.0"
)

# Allow CORS for development
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Track active WebSocket connections
active_connections: List[WebSocket] = []


# Request Models
class ConnectRequest(BaseModel):
    port: str
    baudrate: int = 115200


class ServoAnglesRequest(BaseModel):
    angles: List[int]


CONFIG_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "kinematics_config.json"))
JOURNAL_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "journal_entries.json"))

class KinematicsConfigRequest(BaseModel):
    L1: float = 10.0
    L2: float = 14.0
    L3: float = 12.0
    L4: float = 8.0
    offsets: List[int] = [0, 0, 0, 0, 0, 0]
    gripper_closed: int = 10
    gripper_open: int = 90

class JournalEntriesRequest(BaseModel):
    entries: List[Dict[str, Any]]


@app.get("/api/journal")
async def get_journal_entries():
    """Returns shared master list of journal entries."""
    if os.path.exists(JOURNAL_PATH):
        try:
            with open(JOURNAL_PATH, "r") as f:
                return json.load(f)
        except Exception:
            pass
    return []


@app.post("/api/journal")
async def save_journal_entries(req: JournalEntriesRequest):
    """Saves shared master list of journal entries to backend file."""
    with open(JOURNAL_PATH, "w") as f:
        json.dump(req.entries, f, indent=2)
    return {"status": "saved", "count": len(req.entries)}


@app.get("/api/kinematics")
async def get_kinematics_config():
    """Returns persistent Kinematic Calibration parameters."""
    if os.path.exists(CONFIG_PATH):
        try:
            with open(CONFIG_PATH, "r") as f:
                return json.load(f)
        except Exception:
            pass
    return {
        "L1": 10.0, "L2": 14.0, "L3": 12.0, "L4": 8.0,
        "offsets": [0, 0, 0, 0, 0, 0],
        "gripper_closed": 10, "gripper_open": 90
    }


@app.post("/api/kinematics")
async def save_kinematics_config(req: KinematicsConfigRequest):
    """Saves updated Kinematic Calibration parameters (L1-L4, offsets, gripper angles)."""
    data = req.model_dump()
    with open(CONFIG_PATH, "w") as f:
        json.dump(data, f, indent=2)
    return {"status": "saved", "config": data}


@app.get("/api/ports")
async def list_ports():
    """Lists available serial ports."""
    return {"ports": serial_manager.list_available_ports()}


@app.post("/api/connect")
async def connect_port(req: ConnectRequest):
    """Connects to specified serial port."""
    success, msg = serial_manager.connect(req.port, req.baudrate)
    if not success:
        raise HTTPException(status_code=400, detail=msg)
    await broadcast_status()
    return {"status": "connected", "message": msg}


@app.post("/api/auto-connect")
async def auto_connect_port():
    """Auto-detects and connects to Arduino."""
    success, msg = serial_manager.auto_connect()
    if not success:
        raise HTTPException(status_code=400, detail=msg)
    await broadcast_status()
    return {"status": "connected", "message": msg}


@app.post("/api/disconnect")
async def disconnect_port():
    """Disconnects serial connection."""
    success, msg = serial_manager.disconnect()
    await broadcast_status()
    return {"status": "disconnected", "message": msg}


@app.post("/api/servos")
async def set_servo_angles(req: ServoAnglesRequest):
    """Sets 6 servo angles (0–180)."""
    success, msg = serial_manager.send_angles(req.angles)
    if not success:
        raise HTTPException(status_code=400, detail=msg)
    await broadcast_status()
    return {"status": "success", "angles": serial_manager.current_angles}


@app.post("/api/lock90")
async def lock_all_90():
    """Locks all servos at 90° for assembly (Decision #19)."""
    asyncio.create_task(serial_manager.lock_all_90(broadcast_callback=broadcast_status))
    return {"status": "success", "message": "Locking all servos at 90°"}


@app.post("/api/home")
async def move_home():
    """Moves all servos to Home Position."""
    asyncio.create_task(serial_manager.move_to_home(broadcast_callback=broadcast_status))
    return {"status": "success", "message": "Moving to Home Position"}


@app.post("/api/servos/test/{servo_index}")
async def test_servo(servo_index: int):
    """Performs a solo sweep test on a single servo for assembly diagnostics."""
    asyncio.create_task(serial_manager.test_single_servo(servo_index, broadcast_callback=broadcast_status))
    return {"status": "success", "message": f"Testing Servo {servo_index}"}


@app.post("/api/estop")
async def emergency_stop():
    """Triggers Emergency Stop."""
    success, msg = serial_manager.emergency_stop()
    await broadcast_status()
    return {"status": "estop_active", "message": msg}


@app.post("/api/estop/reset")
async def reset_estop():
    """Resets Emergency Stop state."""
    success, msg = serial_manager.reset_estop()
    await broadcast_status()
    return {"status": "estop_reset", "message": msg}


# WebSocket Handler for Real-Time Telemetry & Slider Control

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    active_connections.append(websocket)
    logger.info("New WebSocket client connected.")
    
    # Send initial status on connect
    await websocket.send_json({"type": "status", "data": serial_manager.get_status()})

    try:
        while True:
            data_str = await websocket.receive_text()
            try:
                data = json.loads(data_str)
                action_type = data.get("type")

                if action_type == "set_angles":
                    angles = data.get("angles", [])
                    serial_manager.send_angles(angles)
                    await broadcast_status()

                elif action_type == "lock90":
                    asyncio.create_task(serial_manager.lock_all_90(broadcast_callback=broadcast_status))

                elif action_type == "home":
                    asyncio.create_task(serial_manager.move_to_home(broadcast_callback=broadcast_status))

                elif action_type == "sweep":
                    asyncio.create_task(serial_manager.run_joint_sweep_test(broadcast_callback=broadcast_status))

                elif action_type == "test_servo":
                    servo_index = int(data.get("index", 0))
                    asyncio.create_task(serial_manager.test_single_servo(servo_index, broadcast_callback=broadcast_status))

                elif action_type == "estop":
                    serial_manager.emergency_stop()
                    await broadcast_status()

                elif action_type == "reset_estop":
                    serial_manager.reset_estop()
                    await broadcast_status()

            except json.JSONDecodeError:
                logger.warning("Received non-JSON WebSocket message.")

    except WebSocketDisconnect:
        active_connections.remove(websocket)
        logger.info("WebSocket client disconnected.")


async def broadcast_status():
    """Broadcasts current status to all connected WebSocket clients."""
    if not active_connections:
        return
    status_data = {"type": "status", "data": serial_manager.get_status()}
    for conn in list(active_connections):
        try:
            await conn.send_json(status_data)
        except Exception:
            if conn in active_connections:
                active_connections.remove(conn)


# Mount static frontend files
FRONTEND_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "../frontend"))
if os.path.exists(FRONTEND_DIR):
    app.mount("/", StaticFiles(directory=FRONTEND_DIR, html=True), name="frontend")


if __name__ == "__main__":
    import uvicorn
    # Use port 8050 to avoid conflicts with macOS AirPlay Receiver (which uses port 8000)
    uvicorn.run("main:app", host="0.0.0.0", port=8050, reload=True)
