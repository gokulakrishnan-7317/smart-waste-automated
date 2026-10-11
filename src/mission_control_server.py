"""
AI Smart Waste Robot -- Mission Control Server
FastAPI backend serving the Mission Control dashboard UI.
"""

import os, sys, time, socket, asyncio, threading, subprocess
from pathlib import Path
from typing import Optional

import cv2, numpy as np
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", line_buffering=True)
if hasattr(sys.stderr, "reconfigure"):
    sys.stderr.reconfigure(encoding="utf-8", line_buffering=True)
try:
    import torch
    from ultralytics import YOLO
    HAVE_YOLO = True
except ImportError:
    torch = None
    YOLO = None
    HAVE_YOLO = False
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles

SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent if SCRIPT_DIR.name == "src" else SCRIPT_DIR
ROOT_DIR = PROJECT_ROOT

ESP32_CAM_STREAM_URL = "http://192.168.137.71:81/stream"  # Leased/Assigned IP on Hotspot
ESP32_DEVKIT_IP      = "192.168.137.204"                 # ESP32 DevKit UDP fallback IP
ESP32_DEVKIT_PORT    = 8888
DEFAULT_WEIGHTS_PATH = str(ROOT_DIR / "models" / "best_trash_yolov8m.pt")
CONF_THRESHOLD       = 0.35
FRAME_WIDTH          = 640
FRAME_HEIGHT         = 480
DEADZONE_X           = 55

shared_state = {
    "mode": "MANUAL", "status": "STANDBY", "fps": 0.0,
    "telemetry": {"dist":-1.0,"ir_l":0,"ir_r":0,"obst":0,"lat":"0.000000","lon":"0.000000","fix":0,"sats":0},
    "detection": {"found":False,"class":"","conf":0.0,"cx":0,"cy":0,"x1":0,"y1":0,"x2":0,"y2":0,"category":""},
    "picking":False,"pick_item":"","items_picked":0,"degradable":0,"non_degradable":0,"banana_peels":0,"batteries":0,"cups":0,
    "mission_start":time.time(),"cam_url":ESP32_CAM_STREAM_URL,
    "devkit_ip":ESP32_DEVKIT_IP,"cam_online":False,"devkit_online":False,
}
frame_lock   = threading.Lock()
latest_frame = [None]

class RobotSerial:
    def __init__(self, port, baudrate=115200):
        self.port=port; self.baudrate=baudrate; self.ser=None
        self.last_cmd=""; self.last_time=0.0
        self.last_rx=0.0
        self.connect()

    def connect(self):
        try:
            import serial
            if self.ser and self.ser.is_open:
                try: self.ser.close()
                except: pass
            print(f"  [Connecting BT Serial] Opening {self.port} at {self.baudrate} baud...")
            self.ser=serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.last_rx = time.time()
            print(f"  [BT Serial Connected] DevKit active on {self.port} (Zero IP needed!)")
            try:
                self.ser.write(b"CMD:PING\n")
                self.ser.flush()
            except Exception: pass
        except Exception as e:
            print(f"  [BT Connect Warning] Could not open {self.port}: {e}")
            self.ser = None

    @property
    def is_connected(self):
        if not self.ser or not self.ser.is_open:
            return False
        # Heartbeat watchdog: allow up to 8.0s for Bluetooth RFCOMM buffering
        return self.last_rx > 0 and (time.time() - self.last_rx) < 8.0

    def send(self, cmd, force=False):
        now=time.time()
        if not force and cmd==self.last_cmd and (now-self.last_time)<0.09: return
        if self.ser and self.ser.is_open:
            try:
                self.ser.write((cmd.strip()+"\n").encode("utf-8"))
                self.ser.flush()
                self.last_cmd=cmd; self.last_time=now
                print(f"  BT -> {self.port}  {cmd}")
            except Exception as e:
                print(f"  BT send err: {e}")
                try: self.ser.close()
                except: pass
                self.ser = None

    def poll_telemetry(self):
        if not self.ser or not self.ser.is_open: return False
        received = False
        try:
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                if line:
                    self.last_rx = time.time()
                if line.startswith("TELEM:"):
                    received = True
                    for part in line[6:].split(","):
                        if "=" in part:
                            k, v = part.split("=", 1); k = k.lower().strip()
                            try:
                                t = shared_state["telemetry"]
                                if k=="dist": t["dist"]=float(v)
                                elif k=="ir_l": t["ir_l"]=int(v)
                                elif k=="ir_r": t["ir_r"]=int(v)
                                elif k=="obst": t["obst"]=int(v)
                                elif k=="lat": t["lat"]=v
                                elif k=="lon": t["lon"]=v
                                elif k=="fix": t["fix"]=int(v)
                                elif k=="sats": t["sats"]=int(v)
                            except ValueError: pass
        except Exception as e:
            print(f"  BT poll err: {e}")
            try: self.ser.close()
            except: pass
            self.ser = None
        return received

    def close(self):
        try:
            if self.ser and self.ser.is_open: self.ser.close()
        except Exception: pass
        self.ser = None

def find_bluetooth_port(device_hint="SMART_WASTE_ROBOT"):
    try:
        import serial
        import serial.tools.list_ports as lp
        ports = list(lp.comports())

        # 1. First priority: SMART_WASTE_ROBOT Bluetooth port (MAC: B0CBD80F8816)
        for p in ports:
            desc = f"{p.device} {p.description} {p.hwid}".upper()
            if "B0CBD80F8816" in desc:
                print(f"  [Auto-Detect] Found SMART_WASTE_ROBOT on {p.device}")
                return p.device

        # 2. Check for direct USB cable connection (CP210x, CH340, FTDI, etc.)
        for p in ports:
            desc = f"{p.device} {p.description} {p.hwid}".upper()
            if any(chip in desc for chip in ["CP210", "CH340", "USB-SERIAL", "FTDI", "PROLIFIC", "SILICON LABS"]):
                print(f"  [Auto-Detect] Found USB Serial DevKit on {p.device}: {p.description}")
                return p.device

        # 3. Check for COM14 or device hint
        for p in ports:
            desc = f"{p.device} {p.description} {p.hwid}".upper()
            if "COM14" == p.device.upper() or device_hint.upper() in desc:
                return p.device

        # 3. Filter out known audio headsets/earbuds
        exclude_macs = ["12254FB5D5BA", "FEE5E937469C", "B56B2C66887E"]
        valid_bt = []
        for p in ports:
            desc = f"{p.device} {p.description} {p.hwid}".upper()
            if ("BTHENUM" in desc or "BLUETOOTH" in desc) and not any(m in desc for m in exclude_macs):
                if "LOCALMFG&0000" not in desc:
                    valid_bt.append(p.device)

        def port_num(d):
            try: return int(d.upper().replace("COM", ""))
            except: return 0
        valid_bt.sort(key=port_num, reverse=True)
        if valid_bt:
            return valid_bt[0]

    except Exception:
        pass
    return None

class RobotUDP:
    def __init__(self, ip, port):
        self.target=(ip,port); self.sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.sock.setblocking(False); self.last_cmd=""; self.last_time=0.0
        self.last_rx = 0.0

    @property
    def is_connected(self):
        # UDP requires actual incoming packets from the ESP32 within the last 3.5s
        return self.last_rx > 0 and (time.time() - self.last_rx) < 3.5
    def send(self, cmd, force=False):
        now=time.time()
        if not force and cmd==self.last_cmd and (now-self.last_time)<0.09: return
        try:
            self.sock.sendto(cmd.encode("utf-8"),self.target)
            self.last_cmd=cmd; self.last_time=now
            print(f"  UDP -> {self.target[0]}:{self.target[1]}  {cmd}")
        except Exception as e: print(f"  UDP err: {e}")
    def poll_telemetry(self):
        try:
            while True:
                data,_=self.sock.recvfrom(512)
                text=data.decode("utf-8",errors="ignore").strip()
                if text.startswith("TELEM:"):
                    self.last_rx = time.time()
                    for part in text[6:].split(","):
                        if "=" in part:
                            k,v=part.split("=",1); k=k.lower().strip()
                            try:
                                t=shared_state["telemetry"]
                                if k=="dist": t["dist"]=float(v)
                                elif k=="ir_l": t["ir_l"]=int(v)
                                elif k=="ir_r": t["ir_r"]=int(v)
                                elif k=="obst": t["obst"]=int(v)
                                elif k=="lat": t["lat"]=v
                                elif k=="lon": t["lon"]=v
                                elif k=="fix": t["fix"]=int(v)
                            except ValueError: pass
        except (BlockingIOError, socket.error, Exception): pass
    def close(self):
        try: self.sock.close()
        except: pass

robot = None

def stream_worker(url:str):
    import urllib.request
    candidates = [url]
    for fb in ["http://smartwastecam.local:81/stream", "http://smartwastecam.local/stream", "http://192.168.137.71:81/stream", "http://192.168.137.71/stream", "http://192.168.137.145:81/stream", "http://192.168.137.145/stream", "http://192.168.137.129:81/stream", "http://192.168.137.137:81/stream", "http://192.168.137.183:81/stream", "http://192.168.137.200:81/stream", "http://192.168.4.1:81/stream"]:
        if fb not in candidates:
            candidates.append(fb)
    if ":81" in url:
        candidates.append(url.replace(":81", ""))
    elif "http://" in url:
        try:
            p = url.split("/", 3)
            host = p[2].split(":")[0]
            candidates.append(f"http://{host}:81/stream")
        except: pass

    cand_idx = 0
    fail_count = 0
    while True:
        target_url = candidates[cand_idx % len(candidates)]
        stream = None
        try:
            req = urllib.request.Request(target_url, headers={"User-Agent": "Mozilla/5.0"})
            stream = urllib.request.urlopen(req, timeout=3.5)
            buf = bytearray()
            shared_state["cam_online"] = True
            fail_count = 0
            while True:
                chunk = stream.read(4096)
                if not chunk: break
                buf.extend(chunk)

                # Zero-Latency: Always jump to the LATEST complete JPEG frame
                e = buf.rfind(b"\xff\xd9")
                if e != -1:
                    s = buf.rfind(b"\xff\xd8", 0, e)
                    if s != -1:
                        jpg = buf[s:e+2]
                        del buf[:e+2]
                        if len(buf) > 32768: buf.clear()
                        frm = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                        if frm is not None:
                            with frame_lock:
                                latest_frame[0] = frm
        except Exception:
            fail_count += 1
            cand_idx += 1
            if fail_count >= 3:
                shared_state["cam_online"] = False
                with frame_lock:
                    latest_frame[0] = None
            if fail_count % 8 == 0:
                try:
                    new_cam, _ = auto_detect()
                    if new_cam and new_cam not in candidates:
                        candidates.insert(0, new_cam)
                        cand_idx = 0
                except Exception:
                    pass
            time.sleep(0.5)
        finally:
            if stream:
                try: stream.close()
                except: pass
            time.sleep(0.2)

def find_weights():
    candidates = [
        str(ROOT_DIR / "models" / "best_trash_yolov8m.pt"),
        str(ROOT_DIR / "weights" / "best_trash_yolov8m.pt"),
        str(ROOT_DIR / "models" / "best_degradable_yolov8.pt"),
        str(ROOT_DIR / "weights" / "best_degradable_yolov8.pt"),
        str(ROOT_DIR / "runs" / "detect" / "deep_trash_yolov8m" / "weights" / "best.pt"),
        str(ROOT_DIR / "runs" / "detect" / "degradable_trash_yolo" / "weights" / "best.pt"),
        str(ROOT_DIR / "models" / "roboflow_banana_weights.pt"),
        str(ROOT_DIR / "models" / "yolov8n.pt"),
        str(ROOT_DIR / "best.pt"),
        "yolov8n.pt"
    ]
    for c in candidates:
        if c and os.path.exists(c): return c
    return "yolov8n.pt"

DEGRADABLE_NAMES = {"banana", "banana peel", "banana_peel", "organic", "fruit", "food"}

def is_degradable_class(cls_name: str) -> bool:
    cn = str(cls_name).lower().replace("_", " ").strip()
    return any(d in cn for d in DEGRADABLE_NAMES)

def process_detections(boxes,model,dist_cm=-1.0):
    best_box=None; max_area=0
    for box in boxes:
        cls_id=int(box.cls[0]); conf=float(box.conf[0])
        x1,y1,x2,y2=box.xyxy[0].tolist(); area=(x2-x1)*(y2-y1)
        if area>max_area: max_area=area; best_box=(model.names[cls_id],conf,x1,y1,x2,y2)
    if not best_box: return False,None,"SEARCHING_WASTE","CMD:STOP",False
    cls_name,conf,x1,y1,x2,y2=best_box
    cx=(x1+x2)/2; diff_x=cx-(FRAME_WIDTH//2); bh=y2-y1
    in_reach=(0.0<dist_cm<=18.0)or(y2>=370)or(bh>=170)
    category = "DEGRADABLE" if is_degradable_class(cls_name) else "NON_DEGRADABLE"
    if not in_reach:
        if diff_x<-DEADZONE_X: cmd,status=("CMD:FORWARD_LEFT",f"APPROACH:LEFT [{category}]")if y2<330 else("CMD:LEFT",f"ALIGN:LEFT [{category}]")
        elif diff_x>DEADZONE_X: cmd,status=("CMD:FORWARD_RIGHT",f"APPROACH:RIGHT [{category}]")if y2<330 else("CMD:RIGHT",f"ALIGN:RIGHT [{category}]")
        else: cmd,status="CMD:FORWARD",f"APPROACH:CENTER [{category}]"
        return True,best_box,status,cmd,False
    else:
        if diff_x<-25: aim,ms="LEFT",min(400,max(80,int(100+(abs(diff_x)-25)*1.5)))
        elif diff_x>25: aim,ms="RIGHT",min(400,max(80,int(100+(diff_x-25)*1.5)))
        else: aim,ms="CENTER",0
        cmd=f"ARM:PICK_TARGET:{category}:{aim}:{ms}"
        return True,best_box,f"ARM:LOCK_{aim}_{category} ({cls_name.upper()})",cmd,True

def telemetry_worker():
    last_query = 0.0
    while True:
        now = time.time()
        if robot:
            robot.poll_telemetry()
            conn = bool(robot.is_connected)
            shared_state["devkit_online"] = conn
            ser_open = getattr(robot, "ser", None) is not None and robot.ser.is_open
            if ser_open:
                if now - last_query >= 0.8:
                    robot.send("CMD:TELEMETRY")
                    last_query = now
                if not conn and (now - robot.last_rx > 10.0):
                    print("  [DevKit Disconnected] Heartbeat timeout (>10s). Closing port for clean reconnect.")
                    robot.close()
            elif isinstance(robot, RobotUDP):
                if now - last_query >= 0.8:
                    robot.send("CMD:TELEMETRY")
                    last_query = now
        else:
            shared_state["devkit_online"] = False
        time.sleep(0.04)

def reconnect_worker():
    global robot
    while True:
        try:
            if not robot or not robot.is_connected:
                bt_port = find_bluetooth_port()
                if bt_port:
                    if not robot or not isinstance(robot, RobotSerial) or getattr(robot, "port", None) != bt_port:
                        if robot:
                            try: robot.close()
                            except: pass
                        robot = RobotSerial(bt_port)
                    else:
                        robot.connect()
                    shared_state["devkit_online"] = bool(robot and robot.is_connected)
                    shared_state["devkit_ip"] = bt_port
                else:
                    shared_state["devkit_online"] = False
        except Exception:
            pass
        time.sleep(5.0)

def hotspot_watchdog():
    script_path = str(ROOT_DIR / "scripts" / "start_hotspot.ps1")
    while True:
        try:
            if not shared_state.get("cam_online", False):
                arp = subprocess.check_output("ipconfig", shell=True).decode("utf-8", errors="ignore")
                if "192.168.137.1" not in arp and os.path.exists(script_path):
                    print("  [Hotspot Watchdog] Mobile Hotspot offline. Starting AADHIDHYA hotspot...")
                    subprocess.run(["powershell", "-ExecutionPolicy", "Bypass", "-File", script_path],
                                   capture_output=True, timeout=8)
        except Exception:
            pass
        time.sleep(12)

def inference_thread():
    global HAVE_YOLO, torch, YOLO
    model = None
    device = "cpu"
    if HAVE_YOLO and torch is not None and YOLO is not None:
        try:
            device = "0" if torch.cuda.is_available() else "cpu"
            weights = find_weights()
            print(f"[AI] Loading YOLOv8 from: {weights}")
            model = YOLO(weights)
            print("[AI] Model loaded OK")
        except Exception as e:
            print(f"[AI] Could not initialize YOLO: {e}")
            model = None
    prev_time=time.time(); pick_start=0.0; ARM_PICK_DUR=7.0
    while True:
        with frame_lock: frame=latest_frame[0]
        if frame is None:
            shared_state["fps"]=0.0
            time.sleep(0.04)
            continue
        frame=cv2.resize(frame,(FRAME_WIDTH,FRAME_HEIGHT))
        cur=time.time(); shared_state["fps"]=0.9*shared_state["fps"]+0.1*(1.0/max(cur-prev_time,1e-5))
        prev_time=cur
        if robot: robot.poll_telemetry()
        dist=shared_state["telemetry"].get("dist",-1.0)
        if shared_state["picking"]:
            elapsed=time.time()-pick_start
            if elapsed<ARM_PICK_DUR:
                if robot: robot.send("CMD:STOP")
                shared_state["status"]=f"ARM:PICKING ({ARM_PICK_DUR-elapsed:.1f}s left)"
            else:
                shared_state["picking"]=False; shared_state["status"]="PICK_COMPLETE"
                shared_state["items_picked"]+=1
                item_lbl = str(shared_state.get("pick_item", "")).upper()
                if "DEGRADABLE" in item_lbl or "BANANA" in item_lbl:
                    shared_state["degradable"] += 1
                    shared_state["banana_peels"] += 1
                else:
                    shared_state["non_degradable"] += 1
        elif shared_state["mode"]=="AUTO":
            if model is None:
                if not HAVE_YOLO:
                    try:
                        import torch as _torch
                        from ultralytics import YOLO as _YOLO
                        torch = _torch
                        YOLO = _YOLO
                        HAVE_YOLO = True
                    except Exception:
                        pass
                if HAVE_YOLO and torch is not None and YOLO is not None:
                    try:
                        device = "0" if torch.cuda.is_available() else "cpu"
                        weights = find_weights()
                        model = YOLO(weights)
                    except Exception:
                        pass
            if model is not None:
                results=model.predict(frame,conf=CONF_THRESHOLD,device=device,verbose=False)
                boxes=results[0].boxes
                found,best,status,cmd,force=process_detections(boxes,model,dist)
                shared_state["status"]=status
                if found and best:
                    cls_name,conf,x1,y1,x2,y2=best
                    shared_state["detection"]={"found":True,"class":cls_name,"conf":round(conf*100,1),
                        "cx":int((x1+x2)/2),"cy":int((y1+y2)/2),"x1":int(x1),"y1":int(y1),"x2":int(x2),"y2":int(y2)}
                else: shared_state["detection"]["found"]=False
                if cmd.startswith("ARM:PICK"):
                    shared_state["picking"]=True; shared_state["pick_item"]=cmd; pick_start=time.time()
                    if robot: robot.send("CMD:STOP",force=True); time.sleep(0.3); robot.send(cmd,force=True)
                elif robot: robot.send(cmd,force=force)
            else:
                shared_state["status"]="AI INITIALIZING..."
            time.sleep(0.01)
        else:
            shared_state["detection"]["found"]=False; shared_state["status"]="MANUAL CONTROL"
            time.sleep(0.033)

def annotated_frames():
    GOLD_METALLIC = (55, 175, 212)   # #d4af37 in BGR
    GOLD_BRIGHT   = (0, 215, 255)    # #ffd700 in BGR
    GOLD_SOFT     = (160, 205, 235)  # champagne gold in BGR
    BG_OBSIDIAN   = (8, 10, 14)      # deep dark obsidian in BGR

    while True:
        with frame_lock: frame=latest_frame[0]
        if frame is None:
            ph = np.full((FRAME_HEIGHT, FRAME_WIDTH, 3), BG_OBSIDIAN, dtype=np.uint8)
            cv2.rectangle(ph, (8, 8), (FRAME_WIDTH-8, FRAME_HEIGHT-8), (35, 100, 130), 1)
            cv2.rectangle(ph, (12, 12), (FRAME_WIDTH-12, FRAME_HEIGHT-12), GOLD_METALLIC, 1)
            cv2.putText(ph, "MISSION CONTROL · OPTICAL SENSOR", (135, 190), cv2.FONT_HERSHEY_SIMPLEX, 0.55, GOLD_BRIGHT, 1)
            cv2.putText(ph, "AWAITING ESP32-CAM FEED", (145, 235), cv2.FONT_HERSHEY_SIMPLEX, 0.78, GOLD_BRIGHT, 2)
            cv2.putText(ph, f"Target: {shared_state['cam_url']}", (140, 275), cv2.FONT_HERSHEY_SIMPLEX, 0.48, GOLD_SOFT, 1)
            ret, buf = cv2.imencode(".jpg", ph, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ret:
                part = (b"--frame\r\n"
                        b"Content-Type: image/jpeg\r\n"
                        b"Content-Length: " + str(len(buf)).encode() + b"\r\n\r\n" +
                        buf.tobytes() + b"\r\n")
                yield part
            time.sleep(0.1); continue

        frame = cv2.resize(frame.copy(), (FRAME_WIDTH, FRAME_HEIGHT))
        det = shared_state["detection"]
        if det["found"]:
            x1, y1, x2, y2 = det["x1"], det["y1"], det["x2"], det["y2"]
            cv2.rectangle(frame, (x1, y1), (x2, y2), GOLD_METALLIC, 2)

            # High-tech corner markers
            c_len = min(16, max(6, int((x2-x1)*0.18)))
            cv2.line(frame, (x1, y1), (x1+c_len, y1), GOLD_BRIGHT, 3)
            cv2.line(frame, (x1, y1), (x1, y1+c_len), GOLD_BRIGHT, 3)
            cv2.line(frame, (x2, y1), (x2-c_len, y1), GOLD_BRIGHT, 3)
            cv2.line(frame, (x2, y1), (x2, y1+c_len), GOLD_BRIGHT, 3)
            cv2.line(frame, (x1, y2), (x1+c_len, y2), GOLD_BRIGHT, 3)
            cv2.line(frame, (x1, y2), (x1, y2-c_len), GOLD_BRIGHT, 3)
            cv2.line(frame, (x2, y2), (x2-c_len, y2), GOLD_BRIGHT, 3)
            cv2.line(frame, (x2, y2), (x2, y2-c_len), GOLD_BRIGHT, 3)

            label = f"{det['class'].upper()}  {det['conf']:.1f}%"
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.50, 1)
            cv2.rectangle(frame, (x1, y1-24), (x1+tw+10, y1), GOLD_METALLIC, -1)
            cv2.putText(frame, label, (x1+5, y1-7), cv2.FONT_HERSHEY_SIMPLEX, 0.50, BG_OBSIDIAN, 1, cv2.LINE_AA)

            cx, cy = det["cx"], det["cy"]
            cv2.line(frame, (cx-18, cy), (cx+18, cy), GOLD_BRIGHT, 1)
            cv2.line(frame, (cx, cy-18), (cx, cy+18), GOLD_BRIGHT, 1)
            cv2.circle(frame, (cx, cy), 6, GOLD_BRIGHT, 1)
            cv2.circle(frame, (cx, cy), 18, GOLD_METALLIC, 1)

        frame[0:FRAME_HEIGHT:4, :] = (frame[0:FRAME_HEIGHT:4, :] * 0.88).astype(np.uint8)

        hud = f"FPS {shared_state['fps']:04.1f}  |  {shared_state['mode']}  |  {shared_state['status']}"
        cv2.rectangle(frame, (0, 0), (FRAME_WIDTH, 30), BG_OBSIDIAN, -1)
        cv2.line(frame, (0, 30), (FRAME_WIDTH, 30), GOLD_METALLIC, 1)
        cv2.putText(frame, hud, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.50, GOLD_BRIGHT, 1, cv2.LINE_AA)

        t = shared_state["telemetry"]
        dist_s = f"{t['dist']:.1f}cm" if t["dist"] > 0 else "N/A"
        bot = f"SONAR:{dist_s}  IR L:{t['ir_l']} R:{t['ir_r']}  GPS:{t['lat']},{t['lon']}"
        cv2.rectangle(frame, (0, FRAME_HEIGHT-24), (FRAME_WIDTH, FRAME_HEIGHT), BG_OBSIDIAN, -1)
        cv2.line(frame, (0, FRAME_HEIGHT-24), (FRAME_WIDTH, FRAME_HEIGHT-24), GOLD_METALLIC, 1)
        cv2.putText(frame, bot, (10, FRAME_HEIGHT-7), cv2.FONT_HERSHEY_SIMPLEX, 0.42, GOLD_SOFT, 1, cv2.LINE_AA)

        ret, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
        if ret:
            part = (b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n"
                    b"Content-Length: " + str(len(buf)).encode() + b"\r\n\r\n" +
                    buf.tobytes() + b"\r\n")
            yield part
        time.sleep(0.015)

def auto_detect():
    bt_port = find_bluetooth_port()
    devkit_ip = bt_port if bt_port else shared_state["devkit_ip"]
    cam_url = shared_state["cam_url"]

    # 1. Probe static/fixed IPs first
    probe_ips = ["192.168.137.71", "192.168.137.145", "192.168.137.129", "192.168.137.137", "192.168.137.183", "192.168.137.200", "192.168.4.1"]
    try:
        arp=subprocess.check_output("arp -a",shell=True).decode("utf-8",errors="ignore")
        in_hs=False
        for line in arp.splitlines():
            if "Interface:" in line: in_hs="192.168.137.1" in line
            elif in_hs:
                parts=line.split()
                if len(parts)>=3 and parts[0].startswith("192.168.137.") and not parts[0].endswith(".255"):
                    if parts[0] not in probe_ips:
                        probe_ips.append(parts[0])
    except: pass

    for ip in probe_ips:
        try:
            s=socket.socket(); s.settimeout(0.3)
            if s.connect_ex((ip,81))==0:
                cam_url=f"http://{ip}:81/stream"; s.close(); break
            elif s.connect_ex((ip,80))==0:
                cam_url=f"http://{ip}/stream"; s.close(); break
            s.close()
        except: pass

    shared_state["cam_url"]=cam_url; shared_state["devkit_ip"]=devkit_ip
    return cam_url,devkit_ip

app=FastAPI(title="Smart Waste Management System Using AI Vision and Robotic Manipulation")
ui_dir = ROOT_DIR / "web_ui"
static_dir = ui_dir / "static"
if static_dir.exists():
    app.mount("/static", StaticFiles(directory=str(static_dir)), name="static")

@app.get("/", response_class=HTMLResponse)
async def index():
    html_file = ui_dir / "index.html"
    with open(html_file, "r", encoding="utf-8") as f:
        return HTMLResponse(content=f.read())

@app.get("/video_feed")
def video_feed():
    return StreamingResponse(annotated_frames(),media_type="multipart/x-mixed-replace; boundary=frame")

@app.get("/api/state")
def get_state():
    return shared_state

@app.websocket("/ws")
async def ws_endpoint(ws:WebSocket):
    await ws.accept(); print("[WS] Client connected")
    async def sender():
        while True:
            t=shared_state["telemetry"]
            payload={"type":"state","mode":shared_state["mode"],"status":shared_state["status"],
                "fps":round(shared_state["fps"],1),"picking":shared_state["picking"],
                "pick_item":shared_state["pick_item"],"items_picked":shared_state["items_picked"],
                "degradable":shared_state["degradable"],"non_degradable":shared_state["non_degradable"],
                "banana_peels":shared_state["banana_peels"],"batteries":shared_state["batteries"],"cups":shared_state["cups"],
                "mission_s":int(time.time()-shared_state["mission_start"]),
                "cam_online":shared_state["cam_online"],"devkit_online":shared_state["devkit_online"],
                "cam_url":shared_state["cam_url"],"devkit_ip":shared_state["devkit_ip"],
                "detection":shared_state["detection"],
                "telemetry":{"dist":f"{t['dist']:.1f}" if t["dist"]>0 else "-1",
                    "ir_l":t["ir_l"],"ir_r":t["ir_r"],"obst":t["obst"],
                    "lat":t["lat"],"lon":t["lon"],"fix":t["fix"],"sats":t["sats"]}}
            try: await ws.send_json(payload)
            except: break
            await asyncio.sleep(0.12)
    async def receiver():
        global robot
        async for msg in ws.iter_json():
            mtype=msg.get("type","")
            if mtype=="cmd":
                cmd=msg.get("cmd","")
                if cmd=="SET_MODE_AUTO": shared_state["mode"]="AUTO"
                elif cmd=="SET_MODE_MANUAL": shared_state["mode"]="MANUAL"
                elif cmd=="ESTOP":
                    shared_state["mode"]="MANUAL"; shared_state["picking"]=False
                    if robot: robot.send("CMD:STOP",force=True)
                elif cmd=="RESET_MISSION":
                    shared_state["items_picked"]=0; shared_state["batteries"]=0
                    shared_state["cups"]=0; shared_state["mission_start"]=time.time()
                elif cmd=="AUTO_DETECT":
                    cam_url,devkit_ip=auto_detect()
                    threading.Thread(target=stream_worker,args=(cam_url,),daemon=True).start()
                    if robot: robot.close()
                    if str(devkit_ip).upper().startswith("COM"):
                        robot=RobotSerial(devkit_ip)
                    else:
                        robot=RobotUDP(devkit_ip,ESP32_DEVKIT_PORT)
                    shared_state["devkit_online"] = bool(robot and robot.is_connected)
                else:
                    if cmd == "CMD:REVERSE":
                        cmd = "CMD:BACKWARD"
                    elif cmd in ("CMD:BACKWARD_LEFT", "CMD:REVERSE_LEFT"):
                        cmd = "CMD:BACK_LEFT"
                    elif cmd in ("CMD:BACKWARD_RIGHT", "CMD:REVERSE_RIGHT"):
                        cmd = "CMD:BACK_RIGHT"
                    is_urgent = (cmd.startswith("ARM:") or 
                                 cmd.startswith("CMD:SPEED:") or 
                                 cmd.startswith("CMD:THROTTLE:") or 
                                 cmd.startswith("CMD:STOP") or
                                 cmd.startswith("CMD:BACK_"))
                    if robot: robot.send(cmd, force=is_urgent)
            elif mtype=="config":
                if "cam_url" in msg:
                    shared_state["cam_url"]=msg["cam_url"]
                    threading.Thread(target=stream_worker,args=(msg["cam_url"],),daemon=True).start()
                if "devkit_ip" in msg:
                    target = msg["devkit_ip"].strip()
                    shared_state["devkit_ip"] = target
                    if robot: robot.close()
                    if target.upper().startswith("COM"):
                        robot = RobotSerial(target)
                    else:
                        robot = RobotUDP(target, ESP32_DEVKIT_PORT)
                    shared_state["devkit_online"] = bool(robot and robot.is_connected)
    try: await asyncio.gather(sender(),receiver())
    except: print("[WS] Client disconnected")

@app.on_event("startup")
async def startup():
    global robot
    bt_port = find_bluetooth_port()
    if bt_port:
        print(f"[Server] Auto-detected ESP32 DevKit Bluetooth Serial on {bt_port} (Zero IP!)")
        robot = RobotSerial(bt_port)
        shared_state["devkit_ip"] = bt_port
    else:
        print(f"[Server] Connecting to DevKit via UDP at {ESP32_DEVKIT_IP}:{ESP32_DEVKIT_PORT}")
        robot = RobotUDP(ESP32_DEVKIT_IP, ESP32_DEVKIT_PORT)
    shared_state["devkit_online"] = bool(robot and robot.is_connected)
    threading.Thread(target=stream_worker,args=(ESP32_CAM_STREAM_URL,),daemon=True).start()
    threading.Thread(target=telemetry_worker,daemon=True).start()
    threading.Thread(target=reconnect_worker,daemon=True).start()
    threading.Thread(target=hotspot_watchdog,daemon=True).start()
    threading.Thread(target=inference_thread,daemon=True).start()
    print("[Server] Mission Control started at http://localhost:8000")

if __name__=="__main__":
    uvicorn.run("mission_control_server:app",host="0.0.0.0",port=8000,reload=False)
