from __future__ import annotations

import csv
import io
import json
import os
import re
import socket
import sqlite3
import threading
import time
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional

import serial
from serial.tools import list_ports
from fastapi import FastAPI, HTTPException, Query
from fastapi.responses import FileResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

BASE_DIR = Path(__file__).resolve().parent
STATIC_DIR = BASE_DIR / "static"
DATA_DIR = Path(os.getenv("VENOTRACE_DATA_DIR", str(BASE_DIR / "data")))
DATA_DIR.mkdir(parents=True, exist_ok=True)
DB_PATH = DATA_DIR / "venotrace.db"

app = FastAPI(
    title="VENOTRACE Research Dashboard API",
    version="1.1.0",
    description=(
        "Local data acquisition and exploratory signal monitoring for the "
        "VENOTRACE ESP32 + MAX30102 + MPU6050 research prototype. "
        "Supports USB Serial and Wi-Fi SoftAP TCP streaming."
    ),
)
app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")

DB_LOCK = threading.Lock()
DB = sqlite3.connect(DB_PATH, check_same_thread=False)
DB.row_factory = sqlite3.Row
SERIAL_LOCK = threading.RLock()
STATE_LOCK = threading.RLock()
SERIAL_OBJ: Optional[serial.Serial] = None
SOCKET_OBJ: Optional[socket.socket] = None
READER_THREAD: Optional[threading.Thread] = None
STOP_EVENT = threading.Event()

STATE: dict[str, Any] = {
    "connected": False,
    "connection_type": "serial",
    "port": None,
    "baud": 115200,
    "optical": "UNKNOWN",
    "imu": "UNKNOWN",
    "model": "NO_MODEL",
    "subject": None,
    "site": None,
    "phase": None,
    "label": -1,
    "last_line": None,
    "last_line_at": None,
    "last_data_at": None,
    "last_error": None,
    "samples_total": 0,
    "windows_total": 0,
    "predictions_total": 0,
    "last_q": None,
    "last_optical_status": "WAITING",
    "last_motion_status": "WAITING",
    "last_window_status": "WAITING",
    "last_prediction_status": "NO_MODEL",
}

SITE_LABELS = {
    0: "Left calf",
    1: "Right calf",
    2: "Left ankle",
    3: "Right ankle",
    4: "Left thigh",
    5: "Right thigh",
}
PHASE_LABELS = {
    0: "Rest — supine",
    1: "Rest — sitting",
    2: "Leg elevated",
    3: "Calf pump",
    4: "Standing",
    5: "Walking",
    6: "Recovery",
}


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def init_db() -> None:
    with DB_LOCK:
        DB.executescript(
            """
            CREATE TABLE IF NOT EXISTS samples (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                received_at TEXT NOT NULL,
                payload_json TEXT NOT NULL,
                raw_line TEXT NOT NULL
            );
            CREATE INDEX IF NOT EXISTS idx_samples_received ON samples(id DESC);
            CREATE TABLE IF NOT EXISTS windows (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                received_at TEXT NOT NULL,
                payload_json TEXT NOT NULL,
                raw_line TEXT NOT NULL
            );
            CREATE INDEX IF NOT EXISTS idx_windows_received ON windows(id DESC);
            CREATE TABLE IF NOT EXISTS predictions (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                received_at TEXT NOT NULL,
                payload_json TEXT NOT NULL,
                raw_line TEXT NOT NULL
            );
            CREATE INDEX IF NOT EXISTS idx_predictions_received ON predictions(id DESC);
            CREATE TABLE IF NOT EXISTS logs (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                received_at TEXT NOT NULL,
                level TEXT NOT NULL,
                message TEXT NOT NULL
            );
            CREATE INDEX IF NOT EXISTS idx_logs_received ON logs(id DESC);
            """
        )
        DB.commit()


def parse_number(value: str, cast: type = float) -> Any:
    value = value.strip()
    if value == "" or value.upper() in {"NA", "NAN", "INF", "-INF", "INFINITY"}:
        return None
    try:
        out = cast(value)
        if isinstance(out, float) and (out != out or out in (float("inf"), float("-inf"))):
            return None
        return out
    except (ValueError, TypeError, OverflowError):
        return None


def insert_json_record(table: str, data: dict[str, Any], raw_line: str) -> None:
    if table not in {"samples", "windows", "predictions"}:
        return
    with DB_LOCK:
        DB.execute(
            f"INSERT INTO {table}(received_at, payload_json, raw_line) VALUES (?, ?, ?)",
            (utc_now(), json.dumps(data, allow_nan=False), raw_line),
        )
        DB.commit()


def add_log(message: str, level: str = "INFO") -> None:
    clean = message.strip()
    if not clean:
        return
    with DB_LOCK:
        DB.execute(
            "INSERT INTO logs(received_at, level, message) VALUES (?, ?, ?)",
            (utc_now(), level, clean[:500]),
        )
        DB.commit()
    with STATE_LOCK:
        STATE["last_line"] = clean[:500]
        STATE["last_line_at"] = utc_now()
        if "MAX30102: OK" in clean:
            STATE["optical"] = "OK"
        elif "MAX30102: no response" in clean or "MAX30102: initialization failed" in clean:
            STATE["optical"] = "FAIL"
        if "MPU6050: OK" in clean:
            STATE["imu"] = "OK"
        elif "MPU6050: not found" in clean or "MPU6050: read failed" in clean:
            STATE["imu"] = "FAIL"
        if "MODEL STATUS: NO_MODEL" in clean:
            STATE["model"] = "NO_MODEL"
        match = re.search(r"optical=(\w+)\s+imu=(\w+).*?subj=(-?\d+).*?site=(-?\d+).*?phase=(-?\d+).*?label=(-?\d+)", clean)
        if match:
            STATE["optical"] = match.group(1)
            STATE["imu"] = match.group(2)
            STATE["subject"] = int(match.group(3))
            STATE["site"] = int(match.group(4))
            STATE["phase"] = int(match.group(5))
            STATE["label"] = int(match.group(6))
    lvl = "ERROR" if ("ERROR" in clean or "FAIL" in clean or "not found" in clean) else "INFO"
    # The log inserted above is INFO; severe entries are still visible in UI.


def parse_line(raw_line: str) -> None:
    line = raw_line.strip()
    if not line:
        return

    if line.startswith("#"):
        add_log(line)
        # Also infer device failures from some init messages.
        low = line.lower()
        with STATE_LOCK:
            if "max30102" in low and ("no response" in low or "initialization failed" in low):
                STATE["optical"] = "FAIL"
            if "mpu6050" in low and ("not found" in low or "read failed" in low):
                STATE["imu"] = "FAIL"
            if "model status" in low and "no_model" in low:
                STATE["model"] = "NO_MODEL"
        return

    parts = line.split(",")
    tag = parts[0].strip() if parts else ""

    try:
        if tag == "D":
            if len(parts) < 27:
                add_log(f"Malformed D row: expected 27 fields, received {len(parts)}", "ERROR")
                return
            keys = [
                "type", "t_ms", "seq", "subj", "site", "phase", "label", "mark",
                "red", "ir", "red_base", "ir_base", "red_dev_pct", "ir_dev_pct",
                "ax_ms2", "ay_ms2", "az_ms2", "gx_dps", "gy_dps", "gz_dps",
                "pitch_deg", "roll_deg", "acc_dev_g", "gyro_dps", "temp_c", "motion", "q"
            ]
            int_fields = {"t_ms", "seq", "subj", "site", "phase", "label", "mark", "red", "ir", "motion", "q"}
            float_fields = set(keys) - int_fields - {"type"}
            data: dict[str, Any] = {"type": "D"}
            for i, key in enumerate(keys[1:], start=1):
                if i >= len(parts):
                    data[key] = None
                elif key in int_fields:
                    data[key] = parse_number(parts[i], int)
                elif key in float_fields:
                    data[key] = parse_number(parts[i], float)
            if data.get("motion") is not None:
                data["motion"] = bool(data["motion"])
            data["site_label"] = SITE_LABELS.get(data.get("site"), "Unknown site")
            data["phase_label"] = PHASE_LABELS.get(data.get("phase"), "Unknown phase")
            data["quality_flags"] = quality_flags(data.get("q"))
            insert_json_record("samples", data, line)
            with STATE_LOCK:
                STATE["samples_total"] += 1
                STATE["last_data_at"] = utc_now()
                STATE["last_q"] = data.get("q")
                STATE["last_optical_status"] = optical_quality(data)
                STATE["last_motion_status"] = motion_quality(data)
                STATE["subject"] = data.get("subj")
                STATE["site"] = data.get("site")
                STATE["phase"] = data.get("phase")
                STATE["label"] = data.get("label")
        elif tag == "W":
            if len(parts) < 16:
                add_log(f"Malformed W row: expected 16 fields, received {len(parts)}", "ERROR")
                return
            data = {
                "type": "W",
                "win": parse_number(parts[1], int),
                "t_ms": parse_number(parts[2], int),
                "subj": parse_number(parts[3], int),
                "site": parse_number(parts[4], int),
                "phase": parse_number(parts[5], int),
                "label": parse_number(parts[6], int),
                "n_valid": parse_number(parts[7], int),
                "valid_frac": parse_number(parts[8], float),
                "ir_mean": parse_number(parts[9], float),
                "red_mean": parse_number(parts[10], float),
                "ir_variation_pct": parse_number(parts[11], float),
                "red_variation_pct": parse_number(parts[12], float),
                "red_ir_variation_ratio": parse_number(parts[13], float),
                "relative_change_from_rest_pct": parse_number(parts[14], float),
                "status": parts[15].strip(),
            }
            data["site_label"] = SITE_LABELS.get(data["site"], "Unknown site")
            data["phase_label"] = PHASE_LABELS.get(data["phase"], "Unknown phase")
            insert_json_record("windows", data, line)
            with STATE_LOCK:
                STATE["windows_total"] += 1
                STATE["last_window_status"] = data["status"]
        elif tag == "P":
            if len(parts) < 8:
                add_log(f"Malformed P row: expected 8 fields, received {len(parts)}", "ERROR")
                return
            data = {
                "type": "P",
                "win": parse_number(parts[1], int),
                "t_ms": parse_number(parts[2], int),
                "model": parts[3].strip(),
                "status": parts[4].strip(),
                "score": parse_number(parts[5], float),
                "flag": parse_number(parts[6], int),
                "note": parts[7].strip(),
            }
            insert_json_record("predictions", data, line)
            with STATE_LOCK:
                STATE["predictions_total"] += 1
                STATE["last_prediction_status"] = data["status"]
                STATE["model"] = data["model"] if data["model"] not in {"none", ""} else "NO_MODEL"
        else:
            add_log(f"Ignored unrecognized serial row: {line[:180]}")
    except Exception as exc:
        add_log(f"Parse error for {tag or 'unknown'} row: {exc}", "ERROR")


def quality_flags(q: Optional[int]) -> list[str]:
    if q is None:
        return ["QUALITY_UNKNOWN"]
    flags = []
    mapping = [
        (1, "MAX30102_DOWN"),
        (2, "IMU_DOWN"),
        (4, "NO_CONTACT"),
        (8, "SATURATED"),
        (16, "MOTION"),
        (32, "WARMUP"),
    ]
    for bit, label in mapping:
        if q & bit:
            flags.append(label)
    return flags if flags else ["USABLE"]


def optical_quality(sample: dict[str, Any]) -> str:
    q = sample.get("q")
    if q is None:
        return "UNKNOWN"
    if q & 1:
        return "SENSOR_DOWN"
    if q & 8:
        return "SATURATED"
    if q & 4:
        return "CHECK_CONTACT"
    if q & 32:
        return "WARMUP"
    return "READING_AVAILABLE"


def motion_quality(sample: dict[str, Any]) -> str:
    q = sample.get("q")
    if q is None:
        return "UNKNOWN"
    if q & 2:
        return "IMU_UNAVAILABLE"
    if q & 16:
        return "POSSIBLE_MOTION"
    return "LOW_ACCEL_DEVIATION"


def reader_worker(ser: serial.Serial, stop_event: threading.Event) -> None:
    add_log(f"Serial reader started on {ser.port} at {ser.baudrate} baud")
    while not stop_event.is_set():
        try:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").strip("\x00\r\n ")
            if line:
                parse_line(line)
        except (serial.SerialException, OSError) as exc:
            if not stop_event.is_set():
                add_log(f"Serial reader stopped: {exc}", "ERROR")
                with STATE_LOCK:
                    STATE["last_error"] = str(exc)
            break
        except Exception as exc:
            add_log(f"Unexpected serial reader error: {exc}", "ERROR")
            time.sleep(0.1)
    add_log("Serial reader stopped")


def socket_reader_worker(sock: socket.socket, stop_event: threading.Event) -> None:
    add_log("Wi-Fi TCP reader started")
    sock_file = sock.makefile("r", encoding="utf-8", errors="replace")
    while not stop_event.is_set():
        try:
            line = sock_file.readline()
            if not line:
                if not stop_event.is_set():
                    add_log("Wi-Fi connection closed by ESP32", "WARN")
                break
            clean_line = line.strip("\x00\r\n ")
            if clean_line:
                parse_line(clean_line)
        except Exception as exc:
            if not stop_event.is_set():
                add_log(f"Wi-Fi TCP reader stopped: {exc}", "ERROR")
                with STATE_LOCK:
                    STATE["last_error"] = str(exc)
            break
    add_log("Wi-Fi TCP reader stopped")


def stop_connection() -> None:
    global SERIAL_OBJ, SOCKET_OBJ, READER_THREAD, STOP_EVENT
    with SERIAL_LOCK:
        STOP_EVENT.set()
        old_ser = SERIAL_OBJ
        old_sock = SOCKET_OBJ
        old_thread = READER_THREAD
        SERIAL_OBJ = None
        SOCKET_OBJ = None
        READER_THREAD = None
        if old_ser is not None:
            try:
                old_ser.close()
            except Exception:
                pass
        if old_sock is not None:
            try:
                old_sock.shutdown(socket.SHUT_RDWR)
            except Exception:
                pass
            try:
                old_sock.close()
            except Exception:
                pass
        if old_thread is not None and old_thread.is_alive() and old_thread is not threading.current_thread():
            old_thread.join(timeout=1.5)
        with STATE_LOCK:
            STATE["connected"] = False
            STATE["port"] = None


def stop_serial() -> None:
    stop_connection()


class ConnectRequest(BaseModel):
    mode: str = Field(default="serial")  # "serial" or "wifi"
    port: Optional[str] = Field(default=None, max_length=200)
    baud: Optional[int] = Field(default=115200, ge=9600, le=921600)
    host: Optional[str] = Field(default="192.168.4.1", max_length=100)
    tcp_port: Optional[int] = Field(default=8888, ge=1, le=65535)


class CommandRequest(BaseModel):
    command: str = Field(min_length=1, max_length=48)


@app.on_event("startup")
def startup() -> None:
    init_db()


@app.on_event("shutdown")
def shutdown() -> None:
    stop_connection()
    try:
        DB.close()
    except Exception:
        pass


@app.get("/")
def root() -> FileResponse:
    return FileResponse(STATIC_DIR / "index.html")


@app.get("/api/health")
def health() -> dict[str, Any]:
    return {"ok": True, "service": "VENOTRACE dashboard backend", "time": utc_now()}


@app.get("/api/ports")
def ports() -> dict[str, Any]:
    found = []
    for p in list_ports.comports():
        found.append({
            "device": p.device,
            "description": p.description or "Serial device",
            "manufacturer": p.manufacturer,
        })
    return {"ports": found}


@app.post("/api/connect")
def connect(req: ConnectRequest) -> dict[str, Any]:
    global SERIAL_OBJ, SOCKET_OBJ, READER_THREAD, STOP_EVENT
    stop_connection()

    if req.mode == "wifi":
        host = (req.host or "192.168.4.1").strip()
        port = req.tcp_port or 8888
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5.0)
            sock.connect((host, port))
            sock.settimeout(None)
        except Exception as exc:
            with STATE_LOCK:
                STATE["last_error"] = str(exc)
            raise HTTPException(status_code=400, detail=f"Could not connect to ESP32 Wi-Fi ({host}:{port}): {exc}")

        STOP_EVENT = threading.Event()
        SOCKET_OBJ = sock
        READER_THREAD = threading.Thread(
            target=socket_reader_worker,
            args=(sock, STOP_EVENT),
            daemon=True,
            name="venotrace-wifi-reader",
        )
        READER_THREAD.start()

        with STATE_LOCK:
            STATE["connected"] = True
            STATE["connection_type"] = "wifi"
            STATE["port"] = f"Wi-Fi: {host}:{port}"
            STATE["baud"] = None
            STATE["last_error"] = None
            STATE["optical"] = "UNKNOWN"
            STATE["imu"] = "UNKNOWN"
        add_log(f"Connected to ESP32 Wi-Fi at {host}:{port}")
        return {"ok": True, "message": f"Connected to ESP32 Wi-Fi at {host}:{port}"}

    # Serial mode
    if not req.port:
        raise HTTPException(status_code=400, detail="Serial port name is required for USB serial mode")
    try:
        ser = serial.Serial(
            port=req.port,
            baudrate=req.baud or 115200,
            timeout=1,
            write_timeout=1,
        )
    except Exception as exc:
        with STATE_LOCK:
            STATE["last_error"] = str(exc)
        raise HTTPException(status_code=400, detail=f"Could not open {req.port}: {exc}")

    STOP_EVENT = threading.Event()
    SERIAL_OBJ = ser
    READER_THREAD = threading.Thread(
        target=reader_worker,
        args=(ser, STOP_EVENT),
        daemon=True,
        name="venotrace-serial-reader",
    )
    READER_THREAD.start()

    with STATE_LOCK:
        STATE["connected"] = True
        STATE["connection_type"] = "serial"
        STATE["port"] = req.port
        STATE["baud"] = req.baud
        STATE["last_error"] = None
        STATE["optical"] = "UNKNOWN"
        STATE["imu"] = "UNKNOWN"
    return {"ok": True, "message": f"Connected to {req.port} at {req.baud} baud"}


@app.post("/api/disconnect")
def disconnect() -> dict[str, Any]:
    stop_connection()
    add_log("Connection closed by dashboard")
    return {"ok": True, "message": "Disconnected"}


@app.get("/api/status")
def status() -> dict[str, Any]:
    with STATE_LOCK:
        result = dict(STATE)
    result["site_label"] = SITE_LABELS.get(result.get("site"), "Not set")
    result["phase_label"] = PHASE_LABELS.get(result.get("phase"), "Not set")
    result["reference_label"] = {-1: "Unlabelled", 0: "Control / reference-negative", 1: "Reference-positive"}.get(result.get("label"), "Not set")
    return result


def fetch_records(table: str, limit: int) -> list[dict[str, Any]]:
    if table not in {"samples", "windows", "predictions"}:
        return []
    with DB_LOCK:
        rows = DB.execute(
            f"SELECT id, received_at, payload_json FROM {table} ORDER BY id DESC LIMIT ?",
            (limit,),
        ).fetchall()
    result = []
    for row in reversed(rows):
        try:
            payload = json.loads(row["payload_json"])
        except json.JSONDecodeError:
            payload = {}
        payload["db_id"] = row["id"]
        payload["received_at"] = row["received_at"]
        result.append(payload)
    return result


@app.get("/api/samples")
def samples(limit: int = Query(default=300, ge=1, le=2000)) -> dict[str, Any]:
    return {"samples": fetch_records("samples", limit)}


@app.get("/api/windows")
def windows(limit: int = Query(default=30, ge=1, le=500)) -> dict[str, Any]:
    return {"windows": fetch_records("windows", limit)}


@app.get("/api/predictions")
def predictions(limit: int = Query(default=20, ge=1, le=200)) -> dict[str, Any]:
    return {"predictions": fetch_records("predictions", limit)}


@app.get("/api/logs")
def logs(limit: int = Query(default=30, ge=1, le=200)) -> dict[str, Any]:
    with DB_LOCK:
        rows = DB.execute(
            "SELECT id, received_at, level, message FROM logs ORDER BY id DESC LIMIT ?",
            (limit,),
        ).fetchall()
    return {"logs": [dict(r) for r in rows]}


ALLOWED_SIMPLE_COMMANDS = {"mark", "cal", "rest", "status", "help"}


def validate_command(command: str) -> str:
    cmd = command.strip()
    if cmd.lower() in ALLOWED_SIMPLE_COMMANDS:
        return cmd.lower()
    m = re.fullmatch(r"(subj|site|phase|label)\s+(-?\d+)", cmd, flags=re.IGNORECASE)
    if not m:
        raise HTTPException(status_code=400, detail="Unsupported command. Use subj/site/phase/label, mark, cal, rest, status, or help.")
    name = m.group(1).lower()
    value = int(m.group(2))
    ranges = {
        "subj": (0, 9999),
        "site": (0, 5),
        "phase": (0, 6),
        "label": (-1, 1),
    }
    lo, hi = ranges[name]
    if value < lo or value > hi:
        raise HTTPException(status_code=400, detail=f"Invalid {name} value. Allowed range: {lo} to {hi}.")
    return f"{name} {value}"


@app.post("/api/command")
def send_command(req: CommandRequest) -> dict[str, Any]:
    global SERIAL_OBJ, SOCKET_OBJ
    command = validate_command(req.command)
    with SERIAL_LOCK:
        ser = SERIAL_OBJ
        sock = SOCKET_OBJ
        if (ser is None or not ser.is_open) and sock is None:
            raise HTTPException(status_code=409, detail="ESP32 is not connected. Connect over Serial or Wi-Fi first.")
        try:
            payload = (command + "\n").encode("utf-8")
            if ser is not None and ser.is_open:
                ser.write(payload)
                ser.flush()
            elif sock is not None:
                sock.sendall(payload)
        except Exception as exc:
            with STATE_LOCK:
                STATE["last_error"] = str(exc)
            raise HTTPException(status_code=500, detail=f"Could not send command: {exc}")
    add_log(f"Dashboard command sent: {command}")
    return {"ok": True, "command": command}


def export_csv(table: str, filename: str) -> StreamingResponse:
    if table not in {"samples", "windows", "predictions"}:
        raise HTTPException(status_code=404, detail="Unknown export table")
    with DB_LOCK:
        rows = DB.execute(
            f"SELECT received_at, payload_json, raw_line FROM {table} ORDER BY id ASC"
        ).fetchall()
    stream = io.StringIO()
    writer = csv.writer(stream)
    writer.writerow(["received_at", "parsed_payload_json", "raw_serial_line"])
    for row in rows:
        writer.writerow([row["received_at"], row["payload_json"], row["raw_line"]])
    payload = stream.getvalue().encode("utf-8-sig")
    return StreamingResponse(
        io.BytesIO(payload),
        media_type="text/csv; charset=utf-8",
        headers={"Content-Disposition": f'attachment; filename="{filename}"'},
    )


@app.get("/api/export/samples.csv")
def export_samples() -> StreamingResponse:
    return export_csv("samples", "venotrace_samples.csv")


@app.get("/api/export/windows.csv")
def export_windows() -> StreamingResponse:
    return export_csv("windows", "venotrace_windows.csv")


@app.get("/api/export/predictions.csv")
def export_predictions() -> StreamingResponse:
    return export_csv("predictions", "venotrace_research_model_records.csv")
