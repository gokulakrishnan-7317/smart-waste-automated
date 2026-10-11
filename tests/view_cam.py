import cv2
import socket
import urllib.request
import numpy as np
import subprocess

# ==============================================================================
# ESP32-CAM Video Stream URL
# ==============================================================================
DEFAULT_CAM_IP = "192.168.137.200"

def get_cam_stream_url(default_ip=DEFAULT_CAM_IP):
    """Auto-detects ESP32-CAM stream (port 80 or 81) or falls back to default IP."""
    for test_ip in [default_ip, "192.168.4.1"]:
        for port, path in [(81, ":81/stream"), (80, "/stream")]:
            try:
                s = socket.socket()
                s.settimeout(0.4)
                if s.connect_ex((test_ip, port)) == 0:
                    s.close()
                    return f"http://{test_ip}{path}"
                s.close()
            except Exception:
                pass

    # Probe ARP table
    try:
        arp_out = subprocess.check_output("arp -a", shell=True).decode("utf-8", errors="ignore")
        current_iface = None
        for line in arp_out.splitlines():
            if "Interface:" in line:
                current_iface = line.strip()
            elif current_iface and "192.168.137.1" in current_iface:
                parts = line.split()
                if len(parts) >= 3 and parts[0].startswith("192.168.137.") and not parts[0].endswith(".255"):
                    ip = parts[0]
                    for port, path in [(80, "/stream"), (81, ":81/stream")]:
                        try:
                            s = socket.socket()
                            s.settimeout(0.3)
                            if s.connect_ex((ip, port)) == 0:
                                s.close()
                                return f"http://{ip}{path}"
                            s.close()
                        except Exception:
                            pass
    except Exception:
        pass
    return f"http://{default_ip}/stream"

class SmartStreamCapture:
    """
    High-speed, zero-latency, threaded MJPEG stream receiver with auto-recovery.
    Eliminates TCP buffer buildup, memory desync, and the freeze-reconnect cycle.
    """
    def __init__(self, source_url: str):
        import threading
        self.source_url = source_url
        self.latest_frame = None
        self.last_frame_time = 0.0
        self.stopped = False
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self._stream_worker, daemon=True)
        self.thread.start()

    def _stream_worker(self):
        while not self.stopped:
            stream = None
            try:
                req = urllib.request.Request(self.source_url, headers={'User-Agent': 'Mozilla/5.0'})
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
        start_wait = time.time()
        while (time.time() - start_wait) < 2.5:
            with self.lock:
                if self.latest_frame is not None:
                    return True
            time.sleep(0.05)
        with self.lock:
            return self.latest_frame is not None or (time.time() - self.last_frame_time) < 4.0

    def read(self):
        with self.lock:
            if self.latest_frame is not None and (time.time() - self.last_frame_time) < 4.0:
                return True, self.latest_frame.copy()
            return False, None

    def release(self):
        self.stopped = True

def main():
    stream_url = get_cam_stream_url()
    print(f"Connecting to ESP32-CAM stream at: {stream_url}")
    print("Press 'q' or 'ESC' inside the video window to quit.")

    cap = SmartStreamCapture(stream_url)
    if not cap.isOpened():
        print(f"\n[Error] Could not open video stream from: {stream_url}")
        print("Please check that ESP32-CAM is powered on and connected to Hotspot.")
        return

    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            print("[Warning] Reconnecting to stream...")
            continue

        cv2.imshow("ESP32-CAM Live Stream", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:
            break

    cap.release()
    cv2.destroyAllWindows()
    print("Stream closed.")

if __name__ == "__main__":
    main()
