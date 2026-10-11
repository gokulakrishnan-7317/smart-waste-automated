"""
100% Wireless Smart Waste Management Robot Controller
=====================================================
- Video Input: Wireless HTTP MJPEG stream from ESP32-CAM
- AI Brain: Trained YOLOv8m on NVIDIA RTX 4060 GPU
- Wireless Control: High-speed UDP command packets sent to ESP32 DevKit
- Actuation: 4-wheel mobile base + 6 DOF robotic arm
"""

import os
import sys
import time
import socket
import argparse
from pathlib import Path
import cv2
import torch
from ultralytics import YOLO

SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent if SCRIPT_DIR.name == "src" else SCRIPT_DIR
ROOT_DIR = PROJECT_ROOT

# ============================================================================
# CONFIGURATION
# ============================================================================
# 1. IP Addresses (Set according to your Laptop Hotspot / Local Network)
ESP32_CAM_STREAM_URL = "http://192.168.137.200:81/stream"  # Fixed Static IP on Hotspot (fallback: 192.168.4.1)
ESP32_DEVKIT_IP      = "192.168.137.204"                 # ESP32 DevKit UDP fallback IP
ESP32_DEVKIT_PORT    = 8888                              # UDP Port on ESP32

# 2. Model Weights
DEFAULT_WEIGHTS_PATH = str(ROOT_DIR / "models" / "best_trash_yolov8m.pt")
CONF_THRESHOLD = 0.35  # Detection confidence threshold (sensitive enough for distant battery)

# 3. Screen Dimensions for Target Tracking
FRAME_WIDTH  = 640
FRAME_HEIGHT = 480
DEADZONE_X   = 55  # Pixel threshold to consider object centered


def find_weights(preferred: str = None) -> str:
    """Find available model weights."""
    candidates = [
        preferred,
        str(ROOT_DIR / "models" / "best_trash_yolov8m.pt"),
        str(ROOT_DIR / "weights" / "best_trash_yolov8m.pt"),
        str(ROOT_DIR / "models" / "best_degradable_yolov8.pt"),
        str(ROOT_DIR / "weights" / "best_degradable_yolov8.pt"),
        str(ROOT_DIR / "runs" / "detect" / "deep_trash_yolov8m" / "weights" / "best.pt"),
        str(ROOT_DIR / "runs" / "detect" / "degradable_trash_yolo" / "weights" / "best.pt"),
        DEFAULT_WEIGHTS_PATH,
        str(ROOT_DIR / "runs" / "detect" / "trash_max_accuracy" / "weights" / "best.pt"),
        str(ROOT_DIR / "runs" / "detect" / "trash_yolov8m_ultra_acc" / "weights" / "best.pt"),
        str(ROOT_DIR / "runs" / "detect" / "trash_finetune" / "weights" / "best.pt"),
        str(ROOT_DIR / "models" / "roboflow_banana_weights.pt"),
        str(ROOT_DIR / "models" / "yolov8n.pt"),
        str(ROOT_DIR / "best.pt"),
        "yolov8m.pt",
        "yolov8n.pt"
    ]
    for c in candidates:
        if c and os.path.exists(c):
            return c
    return "yolov8n.pt"


# ============================================================================
# UDP WIRELESS TRANSMITTER
# ============================================================================
class WirelessRobotClient:
    def __init__(self, ip: str, port: int, dry_run: bool = False):
        self.target = (ip, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.last_sent_cmd = ""
        self.last_sent_time = 0.0
        self.dry_run = dry_run
        self.telemetry = {
            "dist": -1.0,
            "ir_l": 0,
            "ir_r": 0,
            "obst": 0,
            "lat": "0.000000",
            "lon": "0.000000",
            "fix": 0,
            "sats": 0
        }

    def poll_telemetry(self):
        """Read any incoming UDP telemetry packets non-blockingly."""
        if self.dry_run:
            return self.telemetry
        try:
            while True:
                data, _ = self.sock.recvfrom(512)
                text = data.decode('utf-8', errors='ignore').strip()
                if text.startswith("TELEM:"):
                    parts = text[6:].split(',')
                    for p in parts:
                        if '=' in p:
                            k, v = p.split('=', 1)
                            k = k.lower().strip()
                            try:
                                if k == "dist": self.telemetry["dist"] = float(v)
                                elif k == "ir_l": self.telemetry["ir_l"] = int(v)
                                elif k == "ir_r": self.telemetry["ir_r"] = int(v)
                                elif k == "obst": self.telemetry["obst"] = int(v)
                                elif k == "lat": self.telemetry["lat"] = v
                                elif k == "lon": self.telemetry["lon"] = v
                                elif k == "fix": self.telemetry["fix"] = int(v)
                                elif k == "sats": self.telemetry["sats"] = int(v)
                            except ValueError:
                                pass
        except (BlockingIOError, socket.error):
            pass
        return self.telemetry

    def send(self, command: str, force: bool = False):
        """Send command over Wi-Fi UDP without flooding the network."""
        now = time.time()
        # Avoid spamming duplicate commands faster than 100ms unless forced
        if not force and command == self.last_sent_cmd and (now - self.last_sent_time) < 0.1:
            return

        if self.dry_run:
            print(f"[Simulated UDP -> {self.target[0]}:{self.target[1]}] {command}")
            self.last_sent_cmd = command
            self.last_sent_time = now
            return

        try:
            self.sock.sendto(command.encode('utf-8'), self.target)
            self.last_sent_cmd = command
            self.last_sent_time = now
            print(f"📡 [Live Command -> DevKit {self.target[0]}:{self.target[1]}] {command}")
        except Exception as e:
            print(f"[UDP Error] {e}")

    def close(self):
        """Safely close UDP socket."""
        try:
            self.sock.close()
        except Exception:
            pass


# ============================================================================
# BLUETOOTH SERIAL WIRELESS TRANSMITTER (Zero IP Address Dependency)
# ============================================================================
class BluetoothRobotClient:
    def __init__(self, port: str = None, baudrate: int = 115200, dry_run: bool = False):
        self.dry_run = dry_run
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.last_sent_cmd = ""
        self.last_sent_time = 0.0
        self.telemetry = {
            "dist": -1.0, "ir_l": 0, "ir_r": 0, "obst": 0,
            "lat": "0.000000", "lon": "0.000000", "fix": 0, "sats": 0
        }
        if not dry_run and port:
            try:
                import serial
                self.ser = serial.Serial(self.port, self.baudrate, timeout=0.08)
                print(f"✅ [Bluetooth Serial Connected] DevKit active on {self.port} (115200 baud) - Zero IP needed!")
            except Exception as e:
                print(f"⚠️ [Bluetooth Connection Note] Could not open {self.port}: {e}")
                self.ser = None

    def send(self, command: str, force: bool = False):
        now = time.time()
        if not force and command == self.last_sent_cmd and (now - self.last_sent_time) < 0.1:
            return
        if self.dry_run:
            print(f"[Simulated BT -> {self.port}] {command}")
            self.last_sent_cmd = command
            self.last_sent_time = now
            return
        if self.ser and self.ser.is_open:
            try:
                self.ser.write((command.strip() + "\n").encode('utf-8'))
                self.ser.flush()
                self.last_sent_cmd = command
                self.last_sent_time = now
                print(f"📶 [Live Command -> DevKit BT {self.port}] {command}")
            except Exception as e:
                print(f"[BT Serial Send Error] {e}")

    def poll_telemetry(self):
        if not self.ser or not self.ser.is_open or self.dry_run:
            return self.telemetry
        try:
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith("TELEM:"):
                    parts = line[6:].split(',')
                    for p in parts:
                        if '=' in p:
                            k, v = p.split('=', 1)
                            k = k.lower().strip()
                            try:
                                if k == "dist": self.telemetry["dist"] = float(v)
                                elif k == "ir_l": self.telemetry["ir_l"] = int(v)
                                elif k == "ir_r": self.telemetry["ir_r"] = int(v)
                                elif k == "obst": self.telemetry["obst"] = int(v)
                                elif k == "lat": self.telemetry["lat"] = v
                                elif k == "lon": self.telemetry["lon"] = v
                                elif k == "fix": self.telemetry["fix"] = int(v)
                                elif k == "sats": self.telemetry["sats"] = int(v)
                            except ValueError:
                                pass
        except Exception:
            pass
        return self.telemetry

    def close(self):
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass


def find_bluetooth_port(device_hint="SMART_WASTE_ROBOT"):
    """Scans Windows COM ports for paired Bluetooth Serial or USB DevKit ports."""
    try:
        import serial.tools.list_ports as lp
        ports = list(lp.comports())
        # Check ports matching Bluetooth / hint
        for p in ports:
            info = f"{p.device} {p.description} {p.hwid}".upper()
            if "BTHENUM" in info or "BLUETOOTH" in info or device_hint.upper() in info:
                return p.device
        # Standard Serial / Silicon Labs / CH340 / Prolific USB fallbacks
        for p in ports:
            info = f"{p.device} {p.description}".upper()
            if "STANDARD" in info or "CP210" in info or "CH340" in info or "USB-SERIAL" in info:
                return p.device
    except Exception:
        pass
    return None

def auto_detect_hardware(default_cam_url=ESP32_CAM_STREAM_URL, default_devkit_ip=ESP32_DEVKIT_IP):
    """Automatically detects active ESP32-CAM stream and DevKit IP from hotspot ARP table if default is unreachable."""
    import subprocess
    cam_url = default_cam_url
    devkit_ip = default_devkit_ip

    # 1. Quick check if default URL responds on port 81
    try:
        s = socket.socket()
        s.settimeout(0.5)
        host = default_cam_url.split("://")[1].split(":")[0]
        port = int(default_cam_url.split(":")[2].split("/")[0])
        if s.connect_ex((host, port)) == 0:
            s.close()
            return cam_url, devkit_ip
        s.close()
    except Exception:
        pass

    # 1b. Check if direct AP (192.168.4.1) responds on port 81
    try:
        s = socket.socket()
        s.settimeout(0.4)
        if s.connect_ex(("192.168.4.1", 81)) == 0:
            s.close()
            return "http://192.168.4.1:81/stream", devkit_ip
        s.close()
    except Exception:
        pass

    # 2. Check ARP table for connected hotspot devices
    print("[Auto-Discovery] Probing Windows hotspot devices for ESP32...")
    try:
        arp_out = subprocess.check_output("arp -a", shell=True).decode("utf-8", errors="ignore")
        current_iface = None
        hotspot_ips = []
        for line in arp_out.splitlines():
            if "Interface:" in line:
                current_iface = line.strip()
            elif current_iface and "192.168.137.1" in current_iface:
                parts = line.split()
                if len(parts) >= 3:
                    ip = parts[0]
                    if ip.startswith("192.168.137.") and not ip.endswith(".255"):
                        hotspot_ips.append(ip)

        for ip in hotspot_ips:
            try:
                s = socket.socket()
                s.settimeout(0.4)
                if s.connect_ex((ip, 80)) == 0:
                    cam_url = f"http://{ip}/stream"
                    print(f" -> Found ESP32-CAM stream at {cam_url}")
                    s.close()
                    break
                elif s.connect_ex((ip, 81)) == 0:
                    cam_url = f"http://{ip}:81/stream"
                    print(f" -> Found ESP32-CAM stream at {cam_url}")
                    s.close()
                    break
                s.close()
            except Exception:
                pass

        # Identify other hotspot IP as DevKit
        other_ips = [ip for ip in hotspot_ips if ip not in cam_url]
        if other_ips:
            devkit_ip = other_ips[0]
            print(f" -> Found ESP32 DevKit at {devkit_ip}")
    except Exception as e:
        print(f"[Auto-Discovery] {e}")

    return cam_url, devkit_ip


class SmartStreamCapture:
    """
    High-speed, zero-latency, threaded MJPEG stream receiver with auto-recovery.
    Eliminates TCP buffer buildup, memory desync, and the freeze-reconnect cycle.
    """
    def __init__(self, source_str: str):
        import threading
        self.source_str = source_str
        self.is_http = str(source_str).startswith("http://") or str(source_str).startswith("https://")
        self.cap = None
        self.latest_frame = None
        self.last_frame_time = 0.0
        self.stopped = False
        self.lock = threading.Lock()

        if not self.is_http:
            if str(source_str).isdigit():
                self.cap = cv2.VideoCapture(int(source_str), cv2.CAP_DSHOW)
            else:
                self.cap = cv2.VideoCapture(source_str)
            if self.cap is not None:
                self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        else:
            self.thread = threading.Thread(target=self._stream_worker, daemon=True)
            self.thread.start()

    def _stream_worker(self):
        import urllib.request
        import numpy as np

        while not self.stopped:
            stream = None
            try:
                req = urllib.request.Request(self.source_str, headers={'User-Agent': 'Mozilla/5.0'})
                stream = urllib.request.urlopen(req, timeout=3.0)
                buf = bytearray()

                while not self.stopped:
                    chunk = stream.read(8192)
                    if not chunk:
                        break
                    buf.extend(chunk)

                    start = buf.find(b'\xff\xd8')
                    if start != -1:
                        end = buf.find(b'\xff\xd9', start + 2)
                        if end != -1:
                            jpg_bytes = buf[start:end+2]
                            del buf[:end+2]

                            if len(buf) > 131072:
                                buf.clear()

                            frame = cv2.imdecode(np.frombuffer(jpg_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
                            if frame is not None:
                                with self.lock:
                                    self.latest_frame = frame
                                    self.last_frame_time = time.time()
                    elif len(buf) > 65536:
                        buf.clear()
            except Exception:
                time.sleep(0.3)
            finally:
                if stream:
                    try:
                        stream.close()
                    except Exception:
                        pass
                time.sleep(0.3)

    def isOpened(self) -> bool:
        if not self.is_http:
            return self.cap is not None and self.cap.isOpened()
        start_wait = time.time()
        while (time.time() - start_wait) < 2.5:
            with self.lock:
                if self.latest_frame is not None:
                    return True
            time.sleep(0.05)
        with self.lock:
            return self.latest_frame is not None or (time.time() - self.last_frame_time) < 4.0

    def read(self):
        if not self.is_http:
            if self.cap is not None:
                return self.cap.read()
            return False, None

        with self.lock:
            if self.latest_frame is not None and (time.time() - self.last_frame_time) < 4.0:
                return True, self.latest_frame.copy()
            return False, None

    def release(self):
        self.stopped = True
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass


def process_detections(boxes, model, center_x, distance_cm: float = -1.0):
    """
    Evaluate detections and calculate navigation/actuation decisions:
    1. If waste/battery is far away: drive forward with smooth curve steering towards it.
    2. When waste is in grasp range: halt truck, calculate whether waste is on
       LEFT, CENTER, or RIGHT side, and command arm base to turn toward it!
    Returns: (target_found, best_box, status, command, force)
    """
    target_found = False
    best_box = None
    max_area = 0

    for box in boxes:
        cls_id = int(box.cls[0])
        cls_name = model.names[cls_id]
        conf = float(box.conf[0])
        x1, y1, x2, y2 = box.xyxy[0].tolist()
        area = (x2 - x1) * (y2 - y1)

        if area > max_area:
            max_area = area
            best_box = (cls_name, conf, x1, y1, x2, y2)
            target_found = True

    if target_found and best_box:
        cls_name, conf, x1, y1, x2, y2 = best_box
        box_center_x = (x1 + x2) / 2
        box_height = y2 - y1
        diff_x = box_center_x - center_x

        # Check if item is in grabbing zone (front bumper reach)
        # Bounding box bottom y2 >= 370, ultrasonic range <= 18cm, or box height >= 170px
        in_reach = (
            (0.0 < distance_cm <= 18.0) or
            (y2 >= 370) or
            (box_height >= 170)
        )

        if not in_reach:
            # ── DISTANCE IS HIGH: APPROACH THE OBJECT ────────────────────
            # Curve steer if off-center while driving forward
            if diff_x < -DEADZONE_X:
                if y2 < 330:
                    return target_found, best_box, "APPROACH:CURVE_LEFT", "CMD:FORWARD_LEFT", False
                else:
                    return target_found, best_box, "ALIGN:PIVOT_LEFT", "CMD:LEFT", False
            elif diff_x > DEADZONE_X:
                if y2 < 330:
                    return target_found, best_box, "APPROACH:CURVE_RIGHT", "CMD:FORWARD_RIGHT", False
                else:
                    return target_found, best_box, "ALIGN:PIVOT_RIGHT", "CMD:RIGHT", False
            else:
                return target_found, best_box, "APPROACH:FORWARD", "CMD:FORWARD", False

        else:
            # ── IN GRASP RANGE: HALT & AIM ARM BASE TOWARD OBJECT ─────────
            # Determine if battery is on the LEFT side, RIGHT side, or CENTER
            if diff_x < -25:
                aim_dir = "LEFT"
                offset_ms = int(100 + (abs(diff_x) - 25) * 1.5)
                offset_ms = min(400, max(80, offset_ms))
            elif diff_x > 25:
                aim_dir = "RIGHT"
                offset_ms = int(100 + (diff_x - 25) * 1.5)
                offset_ms = min(400, max(80, offset_ms))
            else:
                aim_dir = "CENTER"
                offset_ms = 0

            degradable_names = {"banana", "banana peel", "banana_peel", "organic", "fruit", "food"}
            is_deg = any(d in cls_name.lower().replace("_", " ").strip() for d in degradable_names)
            category = "DEGRADABLE" if is_deg else "NON_DEGRADABLE"

            cmd = f"ARM:PICK_TARGET:{category}:{aim_dir}:{offset_ms}"
            status = f"ARM:LOCK_{aim_dir}_{category} ({cls_name.upper()})"
            return target_found, best_box, status, cmd, True

    return False, None, "SEARCHING_WASTE", "CMD:STOP", False


# ============================================================================
# MAIN WIRELESS VISION & CONTROL LOOP
# ============================================================================
def main():
    parser = argparse.ArgumentParser(description="100% Wireless Smart Waste Vision & Robot Controller")
    parser.add_argument("--source", type=str, default=None, help="ESP32-CAM stream URL, webcam ID ('0'), or video/image path")
    parser.add_argument("--weights", type=str, default=None, help="Path to YOLOv8 weights")
    parser.add_argument("--bt", action="store_true", help="Connect to ESP32 DevKit via Bluetooth Serial (Zero IP needed)")
    parser.add_argument("--port", type=str, default=None, help="COM port for Bluetooth or USB Serial (e.g. COM4)")
    parser.add_argument("--esp-ip", type=str, default=None, help="ESP32 DevKit IP (for UDP mode)")
    parser.add_argument("--esp-port", type=int, default=ESP32_DEVKIT_PORT, help="ESP32 DevKit UDP Port")
    parser.add_argument("--dry-run", action="store_true", help="Simulate without transmitting packets")
    parser.add_argument("--pick-duration", type=float, default=7.0, help="Robotic arm pick execution time in seconds (default: 7.0)")
    parser.add_argument("--test-image", type=str, default=None, help="Run single image test, print decision and exit")
    args = parser.parse_args()

    weights = find_weights(args.weights)
    source_url = args.source
    esp_ip = args.esp_ip

    # Check for Bluetooth Serial port first (zero IP needed!)
    bt_port = args.port
    if not bt_port and (args.bt or args.esp_ip is None):
        bt_port = find_bluetooth_port()

    if source_url is None:
        detected_cam, detected_devkit = auto_detect_hardware(ESP32_CAM_STREAM_URL, ESP32_DEVKIT_IP)
        source_url = detected_cam
        if esp_ip is None and not bt_port:
            esp_ip = detected_devkit
    elif esp_ip is None and not bt_port:
        esp_ip = ESP32_DEVKIT_IP

    print("=" * 65)
    print(" 100% Wireless Smart Waste Vision & Robot Controller")
    print("=" * 65)
    print(f" Source Stream / Input : {args.test_image or source_url}")
    if bt_port:
        print(f" Robot DevKit Link     : BLUETOOTH SERIAL [{bt_port}] (Zero IP Dependency!)")
    else:
        print(f" Robot DevKit Link     : WI-FI UDP [{esp_ip}:{args.esp_port}]")
    print(f" Model Weights         : {weights}")
    print("=" * 65)

    if bt_port:
        robot = BluetoothRobotClient(bt_port, dry_run=args.dry_run)
    else:
        robot = WirelessRobotClient(esp_ip, args.esp_port, dry_run=args.dry_run)

    device = "0" if torch.cuda.is_available() else "cpu"
    print(f"\n[AI] Loading YOLOv8 on {device.upper()}...")
    model = YOLO(weights)
    center_x = FRAME_WIDTH // 2

    # Single Image Test Mode
    if args.test_image:
        if not os.path.exists(args.test_image):
            print(f"[Error] Image not found: {args.test_image}")
            return
        frame = cv2.imread(args.test_image)
        frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))
        results = model.predict(frame, conf=args.conf, device=device, verbose=False)
        target_found, best_box, status, cmd, force = process_detections(results[0].boxes, model, center_x)
        print(f"\n[Test Result] Status: {status} | Dispatched Command: {cmd}")
        if best_box:
            cls_name, conf, x1, y1, x2, y2 = best_box
            print(f"[Target Detected] {cls_name} ({conf*100:.1f}%) BBox: [{x1:.1f}, {y1:.1f}, {x2:.1f}, {y2:.1f}]")
        robot.send(cmd, force=force)
        robot.close()
        return

    # Video / Stream Mode (ESP32-CAM Only - No Webcam Fallback)
    print(f"\n[Vision] Connecting to ESP32-CAM stream: {source_url}...")
    cap = SmartStreamCapture(source_url)
    retry_count = 0
    while not cap.isOpened() and retry_count < 5:
        retry_count += 1
        print(f" -> Connecting to ESP32-CAM... (Attempt {retry_count}/5)")
        time.sleep(1.0)
        cap = SmartStreamCapture(source_url)

    if not cap.isOpened():
        print(f"\n[Error] Could not connect to ESP32-CAM stream at: {source_url}")
        print("Troubleshooting Checklist:")
        print(" 1. Ensure the ESP32-CAM is powered with a stable 5V supply.")
        print(" 2. Make sure GPIO 0 is DISCONNECTED from GND, and press RST.")
        print(" 3. Upload the updated esp32_cam_firmware.ino with PSRAM Enabled in Arduino IDE.")
        robot.close()
        return

    # Autonomous Pick State Machine
    ARM_PICK_DURATION = args.pick_duration
    is_picking = False
    pick_start_time = 0.0
    pick_item = ""
    fps = 0.0
    prev_time = time.time()

    last_reconnect_msg = 0.0

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                now = time.time()
                if now - last_reconnect_msg > 3.0:
                    print(f"[Vision] Waiting for stream from {source_url}... (If camera froze/browned out, press RST button on ESP32-CAM)")
                    last_reconnect_msg = now
                time.sleep(0.2)
                continue

            frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))

            curr_time = time.time()
            fps = 0.9 * fps + 0.1 * (1.0 / max(curr_time - prev_time, 1e-5))
            prev_time = curr_time

            # Poll live sensor & GPS telemetry from ESP32 DevKit
            telem = robot.poll_telemetry()
            current_dist = telem.get("dist", -1.0)

            if is_picking:
                # Truck remains halted while robotic arm performs picking & bin sorting
                elapsed = time.time() - pick_start_time
                if elapsed < ARM_PICK_DURATION:
                    robot.send("CMD:STOP")
                    status = f"ARM:PICKING:{pick_item} ({ARM_PICK_DURATION - elapsed:.1f}s remaining)"
                    annotated_frame = frame.copy()
                else:
                    is_picking = False
                    status = "PICK_FINISHED_RESUMING_SEARCH"
                    print(f"\n[Arm] Pick-and-place completed for {pick_item}. Resuming waste search.")
                    annotated_frame = frame.copy()
            else:
                results = model.predict(frame, conf=args.conf, device=device, verbose=False)
                boxes = results[0].boxes
                annotated_frame = results[0].plot()

                target_found, best_box, status, cmd, force = process_detections(boxes, model, center_x, current_dist)

                if cmd.startswith("ARM:PICK:") or cmd.startswith("ARM:PICK_TARGET:"):
                    is_picking = True
                    pick_start_time = time.time()
                    pick_item = cmd
                    print(f"\n🎯 [Target Locked in Grasp Range!] Halting truck and activating 6-DOF arm: {cmd}")
                    robot.send("CMD:STOP", force=True)
                    time.sleep(0.3)  # Allow chassis momentum to settle completely for 100% grasp accuracy
                    robot.send(cmd, force=True)
                    status = f"ARM:PICKING ({ARM_PICK_DURATION:.1f}s)"
                else:
                    robot.send(cmd, force=force)

                if target_found and best_box:
                    cls_name, conf, x1, y1, x2, y2 = best_box
                    box_center_x = (x1 + x2) / 2
                    box_center_y = (y1 + y2) / 2
                    cv2.circle(annotated_frame, (int(box_center_x), int(box_center_y)), 6, (0, 0, 255), -1)

            hud_text = f"FPS: {fps:.1f} | Status: {status}"
            # Dual color text shadow for crystal clear HUD readability
            cv2.putText(annotated_frame, hud_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 3)
            cv2.putText(annotated_frame, hud_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)

            # Live Sensors & GPS telemetry footer
            dist_str = f"{telem['dist']:.1f}cm" if telem['dist'] > 0 else "N/A"
            telem_text = f"Obstacle: {dist_str} | IR: L={telem['ir_l']} R={telem['ir_r']} | GPS: {telem['lat']}, {telem['lon']} (Fix:{telem['fix']})"
            cv2.putText(annotated_frame, telem_text, (10, FRAME_HEIGHT - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3)
            cv2.putText(annotated_frame, telem_text, (10, FRAME_HEIGHT - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

            cv2.imshow("Smart Waste Robot - Wireless Vision & Manipulation", annotated_frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                robot.send("CMD:STOP", force=True)
                break

    finally:
        robot.send("CMD:STOP", force=True)
        robot.close()
        cap.release()
        cv2.destroyAllWindows()
        print("\n[System] Wireless Controller Stopped.")

if __name__ == "__main__":
    main()
