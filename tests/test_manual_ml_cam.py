"""
Smart Waste AI - Live ESP32-CAM Object Detection & DevKit Instruction Controller
================================================================================
- Live Video Input: Wireless HTTP MJPEG stream from ESP32-CAM
- AI Brain: YOLOv8m (best_trash_yolov8m.pt) on NVIDIA RTX 4060 GPU
- Live DevKit Control: High-speed UDP instructions dispatched to ESP32 DevKit
- Dual Operating Modes:
    1. MANUAL MODE: Interactive keyboard controls (W/A/S/D driving, P to pick)
    2. AUTO-PILOT MODE: AI visual servoing (auto-steers, approaches & sorts waste)
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
PROJECT_ROOT = SCRIPT_DIR.parent if SCRIPT_DIR.name in ("src", "scripts", "tests") else SCRIPT_DIR
ROOT_DIR = PROJECT_ROOT

# ==============================================================================
# DEFAULT HARDWARE CONFIGURATION
# ==============================================================================
DEFAULT_ESP32_CAM_URL = "http://192.168.137.200:81/stream"      # ESP32-CAM Fixed Hotspot Stream URL
DEFAULT_DEVKIT_IP     = "192.168.137.204"                    # ESP32 DevKit IP
DEFAULT_DEVKIT_PORT   = 8888                               # UDP Port on ESP32 DevKit

# Primary Model Weights (Prioritizes newly trained best weights)
candidates = [
    str(ROOT_DIR / "models" / "best_trash_yolov8m.pt"),
    str(ROOT_DIR / "weights" / "best_trash_yolov8m.pt"),
    str(ROOT_DIR / "models" / "best_degradable_yolov8.pt"),
    str(ROOT_DIR / "weights" / "best_degradable_yolov8.pt"),
    str(ROOT_DIR / "runs" / "detect" / "deep_trash_yolov8m" / "weights" / "best.pt"),
    str(ROOT_DIR / "runs" / "detect" / "trash_max_accuracy" / "weights" / "best.pt"),
    str(ROOT_DIR / "runs" / "detect" / "trash_yolov8m_ultra_acc" / "weights" / "best.pt"),
    str(ROOT_DIR / "runs" / "detect" / "trash_finetune" / "weights" / "best.pt"),
    str(ROOT_DIR / "models" / "roboflow_banana_weights.pt"),
    str(ROOT_DIR / "models" / "yolov8n.pt"),
    "yolov8m.pt",
    "yolov8n.pt"
]
WEIGHTS_PATH = next((c for c in candidates if os.path.exists(c)), "yolov8m.pt")

# Waste Sorting Bin Assignment (Dual Dustbin System)
def get_sorting_bin(cls_name: str) -> tuple:
    """Returns (bin_display_str, short_category, color_bgr)"""
    name_upper = cls_name.upper()
    if "BANANA" in name_upper or "ORGANIC" in name_upper or "DEGRADABLE" in name_upper:
        return "RIGHT BIN (180° CW)", "Degradable Waste", (0, 255, 0)
    else:
        return "LEFT BIN (180° CCW)", "Non-Degradable", (0, 165, 255)

class DevKitTransmitter:
    """Sends rate-limited live UDP instructions to the ESP32 DevKit."""
    def __init__(self, ip: str, port: int):
        self.target = (ip, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.last_cmd = "NONE"
        self.last_cmd_time = 0.0

    def send(self, cmd: str, force: bool = False):
        now = time.time()
        # Avoid spamming unchanged movement instructions faster than 100ms
        if not force and cmd == self.last_cmd and (now - self.last_cmd_time) < 0.10:
            return
        try:
            self.sock.sendto(cmd.encode('utf-8'), self.target)
            self.last_cmd = cmd
            self.last_cmd_time = now
            print(f"📡 [Live Instruction -> DevKit {self.target[0]}:{self.target[1]}] {cmd}")
        except Exception as e:
            print(f"❌ [UDP Error] {e}")

    def close(self):
        try:
            self.send("CMD:STOP", force=True)
            self.sock.close()
        except Exception:
            pass

def auto_detect_hardware(default_cam_url=DEFAULT_ESP32_CAM_URL, default_devkit_ip=DEFAULT_DEVKIT_IP):
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
                if s.connect_ex((ip, 81)) == 0:
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

def open_video_source(source_str: str):
    """Open video stream from URL, integer webcam ID, or file."""
    return SmartStreamCapture(source_str)

def main():
    parser = argparse.ArgumentParser(description="Live ESP32-CAM Object Detection & DevKit Instruction Controller")
    parser.add_argument("--source", type=str, default=None, help="ESP32-CAM stream URL or '0' for webcam")
    parser.add_argument("--cam-ip", type=str, default=None, help="ESP32-CAM IP (e.g. 192.168.137.34)")
    parser.add_argument("--devkit-ip", type=str, default=DEFAULT_DEVKIT_IP, help="ESP32 DevKit IP")
    parser.add_argument("--devkit-port", type=int, default=DEFAULT_DEVKIT_PORT, help="ESP32 DevKit UDP Port")
    parser.add_argument("--conf", type=float, default=0.40, help="Detection confidence threshold")
    parser.add_argument("--auto", action="store_true", help="Start directly in Autonomous Instruction mode")
    parser.add_argument("--weights", type=str, default=WEIGHTS_PATH, help="Path to YOLOv8 model weights")
    args = parser.parse_args()

    # Determine camera stream source and DevKit IP
    source = args.source
    devkit_ip = args.devkit_ip
    if source is None:
        if args.cam_ip:
            source = f"http://{args.cam_ip.strip()}:81/stream"
        else:
            source, devkit_ip = auto_detect_hardware(DEFAULT_ESP32_CAM_URL, args.devkit_ip)

    print("=" * 70)
    print(" 🤖 Smart Waste Management: Live Vision & DevKit Instruction System")
    print("=" * 70)
    print(f" Camera Stream Source : {source}")
    print(f" ESP32 DevKit Target  : {devkit_ip}:{args.devkit_port}")
    print(f" YOLOv8 Model Weights : {args.weights}")
    print("=" * 70)

    # Initialize UDP transmitter
    transmitter = DevKitTransmitter(devkit_ip, args.devkit_port)

    # Load YOLOv8 Model on GPU
    device = "0" if torch.cuda.is_available() else "cpu"
    print(f"\n[AI] Loading YOLOv8 on {device.upper()} (NVIDIA GPU acceleration)...")
    model = YOLO(args.weights)

    # Connect to camera stream
    print(f"\n[Vision] Connecting to camera stream: {source}...")
    cap = open_video_source(source)
    retry_count = 0

    while not cap.isOpened() and retry_count < 3:
        retry_count += 1
        print(f" -> Connecting to {source}... (Attempt {retry_count}/3)")
        time.sleep(1.0)
        cap = open_video_source(source)

    # Interactive IP prompt if default unreachable
    if not cap.isOpened():
        print(f"\n⚠️ Could not connect to camera at: {source}")
        print("Tip: If your ESP32-CAM has a different IP on your Wi-Fi/Hotspot, enter it below:")
        try:
            user_input = input("Enter ESP32-CAM IP or URL (or '0' for laptop webcam, or Enter to retry): ").strip()
            if user_input:
                if user_input == "0":
                    source = "0"
                elif user_input.startswith("http://"):
                    source = user_input
                else:
                    source = f"http://{user_input}:81/stream"
                print(f"Retrying with source: {source}...")
                cap = open_video_source(source)
            else:
                cap = open_video_source(source)
        except Exception:
            pass

    if not cap.isOpened():
        print(f"\n❌ [Connection Failed] Unable to stream from {source}")
        print("Checklist:")
        print(" 1. Ensure your Laptop Mobile Hotspot is turned ON.")
        print(" 2. Ensure ESP32-CAM and ESP32 DevKit are powered and connected to the Hotspot.")
        print(" 3. Check Serial Monitor in Arduino IDE to verify the exact IP addresses.")
        transmitter.close()
        return

    print("\n" + "=" * 65)
    print(" 🚀 LIVE OBJECT DETECTION & INSTRUCTION STREAM ACTIVE")
    print("=" * 65)
    print(" Controls:")
    print("   [M]         : Toggle AUTO-PILOT AI instruction streaming (ON/OFF)")
    print("   [W/A/S/D]   : Manual Drive (Forward / Left / Backward / Right)")
    print("   [Space]     : Stop Robot (CMD:STOP)")
    print("   [P]         : Send ARM:PICK for currently detected trash")
    print("   [1 / 2 / 3] : Quick Pick: [1] Plastic | [2] Metal | [3] Glass")
    print("   [+ / -]     : Adjust Confidence Threshold (+/- 0.05)")
    print("   [S]         : Save annotated snapshot photo")
    print("   [Q] / ESC   : Exit and stop robot")
    print("=" * 65 + "\n")

    conf_thresh = args.conf
    auto_mode = args.auto
    prev_time = time.time()
    fps = 0.0
    last_detected_class = None

    # Auto-Pilot State Variables
    ARM_PICK_DURATION = 5.5
    is_picking = False
    pick_start_time = 0.0
    pick_item = ""

    FRAME_W, FRAME_H = 640, 480
    CENTER_X = FRAME_W // 2
    DEADZONE_X = 65

    try:
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                time.sleep(0.05)
                continue

            frame = cv2.resize(frame, (FRAME_W, FRAME_H))

            # FPS measurement
            curr_time = time.time()
            fps = 0.9 * fps + 0.1 * (1.0 / max(curr_time - prev_time, 1e-5))
            prev_time = curr_time

            # Run YOLOv8 ML Inference
            infer_start = time.time()
            results = model.predict(frame, conf=conf_thresh, device=device, verbose=False)
            infer_ms = (time.time() - infer_start) * 1000

            annotated_frame = results[0].plot()
            boxes = results[0].boxes

            detected_summary = []
            best_target = None
            max_area = 0

            for box in boxes:
                cls_id = int(box.cls[0])
                cls_name = model.names[cls_id]
                conf = float(box.conf[0])
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                area = (x2 - x1) * (y2 - y1)

                bin_slot, bin_type, color = get_sorting_bin(cls_name)
                detected_summary.append(f"{cls_name} ({conf*100:.0f}%) -> {bin_slot}")

                if area > max_area:
                    max_area = area
                    best_target = (cls_name, conf, x1, y1, x2, y2)
                    last_detected_class = cls_name

            # ==============================================================
            # AUTONOMOUS INSTRUCTION STREAMING ENGINE
            # ==============================================================
            auto_status_text = "SEARCHING"
            if auto_mode:
                if is_picking:
                    elapsed = time.time() - pick_start_time
                    if elapsed < ARM_PICK_DURATION:
                        transmitter.send("CMD:STOP")
                        auto_status_text = f"ARM:PICKING:{pick_item} ({ARM_PICK_DURATION - elapsed:.1f}s)"
                    else:
                        is_picking = False
                        auto_status_text = "PICK COMPLETE -> RESUMING"
                        print(f"[Arm] Pick finished for {pick_item}. Resuming live search.")
                elif best_target:
                    cls_name, conf, x1, y1, x2, y2 = best_target
                    box_cx = (x1 + x2) / 2
                    box_h = y2 - y1
                    diff_x = box_cx - CENTER_X

                    # Centering Deadzone & Forward Drive
                    if abs(diff_x) > DEADZONE_X:
                        if diff_x < 0:
                            transmitter.send("CMD:LEFT")
                            auto_status_text = "STEERING LEFT ⮌"
                        else:
                            transmitter.send("CMD:RIGHT")
                            auto_status_text = "STEERING RIGHT ⮎"
                    else:
                        if box_h < 220:
                            transmitter.send("CMD:FORWARD")
                            auto_status_text = "APPROACHING FORWARD ⬆"
                        else:
                            # In target range: Trigger pick & place
                            pick_cmd = f"ARM:PICK:{cls_name.upper()}"
                            transmitter.send(pick_cmd, force=True)
                            is_picking = True
                            pick_start_time = time.time()
                            pick_item = cls_name
                            auto_status_text = f"IN RANGE! DISPATCHED {pick_cmd}"
                else:
                    transmitter.send("CMD:STOP")
                    auto_status_text = "NO TARGET -> IDLE STOP"

            # ==============================================================
            # ON-SCREEN HEADS-UP DISPLAY (HUD)
            # ==============================================================
            # Top Status Bar
            hud = annotated_frame.copy()
            cv2.rectangle(hud, (0, 0), (FRAME_W, 60), (18, 18, 18), -1)
            cv2.addWeighted(hud, 0.75, annotated_frame, 0.25, 0, annotated_frame)

            top_l1 = f"YOLOv8m | Latency: {infer_ms:.1f}ms | FPS: {fps:.1f} | Conf: {conf_thresh:.2f}"
            cv2.putText(annotated_frame, top_l1, (12, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

            mode_str = "[AUTO-PILOT ACTIVE]" if auto_mode else "[MANUAL CONTROL]"
            mode_color = (0, 255, 0) if auto_mode else (255, 200, 0)
            top_l2 = f"DevKit: {args.devkit_ip}:{args.devkit_port} | Mode: {mode_str}"
            cv2.putText(annotated_frame, top_l2, (12, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.52, mode_color, 2)

            # Center Aim Guides (if Auto-Pilot is ON)
            if auto_mode:
                cv2.line(annotated_frame, (CENTER_X - DEADZONE_X, 60), (CENTER_X - DEADZONE_X, FRAME_H - 40), (80, 80, 80), 1)
                cv2.line(annotated_frame, (CENTER_X + DEADZONE_X, 60), (CENTER_X + DEADZONE_X, FRAME_H - 40), (80, 80, 80), 1)
                cv2.line(annotated_frame, (CENTER_X, 60), (CENTER_X, FRAME_H - 40), (50, 150, 50), 1)

            # Detected Items & Sorting Destinations
            if detected_summary:
                for idx, text in enumerate(detected_summary[:3]):
                    y_pos = 85 + (idx * 24)
                    cv2.putText(annotated_frame, text, (12, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3)
                    cv2.putText(annotated_frame, text, (12, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

            # Bottom Instruction Dispatch Banner
            bottom_hud = annotated_frame.copy()
            cv2.rectangle(bottom_hud, (0, FRAME_H - 38), (FRAME_W, FRAME_H), (15, 15, 15), -1)
            cv2.addWeighted(bottom_hud, 0.8, annotated_frame, 0.2, 0, annotated_frame)

            last_sent_str = f"Last Cmd: {transmitter.last_cmd}"
            if auto_mode:
                last_sent_str += f" | AI: {auto_status_text}"
            cv2.putText(annotated_frame, last_sent_str, (12, FRAME_H - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 2)

            # Show preview
            cv2.imshow("Smart Waste AI - Live Vision & DevKit Instruction", annotated_frame)
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q') or key == 27:
                break
            elif key == ord('m') or key == ord('M'):
                auto_mode = not auto_mode
                if not auto_mode:
                    transmitter.send("CMD:STOP", force=True)
                print(f"[Mode Switch] Auto-Pilot AI Streaming: {'ENABLED' if auto_mode else 'DISABLED'}")
            elif key == ord('w') or key == ord('W'):
                transmitter.send("CMD:FORWARD", force=True)
            elif key == ord('s') or key == ord('S'):
                transmitter.send("CMD:BACKWARD", force=True)
            elif key == ord('a') or key == ord('A'):
                transmitter.send("CMD:LEFT", force=True)
            elif key == ord('d') or key == ord('D'):
                transmitter.send("CMD:RIGHT", force=True)
            elif key == ord(' ') or key == ord('x') or key == ord('X'):
                transmitter.send("CMD:STOP", force=True)
            elif key == ord('p') or key == ord('P'):
                if last_detected_class:
                    cmd = f"ARM:PICK:{last_detected_class.upper()}"
                    transmitter.send(cmd, force=True)
                else:
                    print("⚠️ No waste object currently in view to pick.")
            elif key == ord('1'):
                transmitter.send("ARM:PICK:PLASTIC BOTTLE", force=True)
            elif key == ord('2'):
                transmitter.send("ARM:PICK:CAN", force=True)
            elif key == ord('3'):
                transmitter.send("ARM:PICK:GLASS BOTTLE", force=True)
            elif key == ord('+') or key == ord('='):
                conf_thresh = min(0.95, round(conf_thresh + 0.05, 2))
                print(f"[Confidence] Threshold set to {conf_thresh:.2f}")
            elif key == ord('-') or key == ord('_'):
                conf_thresh = max(0.10, round(conf_thresh - 0.05, 2))
                print(f"[Confidence] Threshold set to {conf_thresh:.2f}")
            elif key == ord('c') or key == ord('C'):
                snap_path = f"live_detection_{int(time.time())}.jpg"
                cv2.imwrite(snap_path, annotated_frame)
                print(f"[Snapshot] Saved to {snap_path}")

    finally:
        cap.release()
        transmitter.close()
        cv2.destroyAllWindows()
        print("\n[Terminated] Camera stream closed and robot halted safely.")

if __name__ == "__main__":
    main()
