"""
ESP32-CAM Connection Tester & Video Stream Diagnostic Tool
==========================================================
Usage:
  1. Auto-test default or specific IP:
     python test_cam_connection.py --ip 192.168.137.10

  2. Test with full stream URL:
     python test_cam_connection.py --url http://192.168.137.10:81/stream

  3. Auto-scan local network / hotspot for ESP32-CAM:
     python test_cam_connection.py --scan
"""

import sys
import os
import time
import socket
import argparse
import subprocess
from pathlib import Path
import urllib.request
import cv2

def ping_ip(ip: str, timeout_ms: int = 1000) -> bool:
    """Check if IP address is reachable with ICMP ping."""
    param = "-n" if sys.platform.lower().startswith("win") else "-c"
    timeout_param = "-w" if sys.platform.lower().startswith("win") else "-W"
    cmd = ["ping", param, "1", timeout_param, str(timeout_ms), ip]
    try:
        res = subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return res.returncode == 0
    except Exception:
        return False

def test_tcp_port(ip: str, port: int, timeout: float = 2.0) -> bool:
    """Test if TCP port is open on target IP."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(timeout)
    try:
        result = sock.connect_ex((ip, port))
        sock.close()
        return result == 0
    except Exception:
        return False

def find_candidate_ips():
    """Extract candidate IPv4 addresses from local ARP table."""
    candidates = []
    try:
        output = subprocess.check_output("arp -a", shell=True, text=True)
        for line in output.splitlines():
            parts = line.strip().split()
            if len(parts) >= 2:
                ip = parts[0]
                # Check for standard IPv4 pattern
                octets = ip.split(".")
                if len(octets) == 4 and all(o.isdigit() for o in octets):
                    # Filter out broadcast/multicast
                    if not (ip.endswith(".255") or ip.startswith("224.") or ip.startswith("239.") or ip.startswith("127.")):
                        candidates.append(ip)
    except Exception as e:
        print(f"[ARP Warning] Could not read ARP table: {e}")
    return candidates

def test_camera_stream(url: str, preview: bool = True):
    """
    Test opening the camera stream with OpenCV, verify frame reception,
    and display live preview with diagnostic HUD.
    """
    print(f"\n[Vision Test] Attempting to open video stream at: {url}")
    print("Connecting... (Press Ctrl+C or 'q' in preview window to exit)")

    cap = cv2.VideoCapture(url)
    
    # Wait up to 5 seconds to connect
    start_time = time.time()
    opened = False
    while time.time() - start_time < 5.0:
        if cap.isOpened():
            opened = True
            break
        time.sleep(0.3)

    if not opened:
        print(f"\n[FAILED] Could not open video stream at: {url}")
        print("\nTroubleshooting Checklist:")
        print(" 1. Check if the ESP32-CAM is powered ON (Red power LED should be steady).")
        print(" 2. Confirm ESP32-CAM is connected to the same Wi-Fi / Laptop Hotspot.")
        print(" 3. Open Serial Monitor in Arduino IDE (115200 baud) to verify the assigned IP address.")
        print(" 4. ESP32-CAM brownout: Ensure you provide stable 5V (at least 1.5A - 2A). Weak USB power causes reboot loops.")
        print(" 5. Test opening the stream directly in your browser: " + url)
        return False

    print("\n[SUCCESS] Stream opened successfully! Reading live frames...")
    
    frame_count = 0
    fps_start = time.time()
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                print("[WARN] Dropped frame or empty response from stream.")
                time.sleep(0.1)
                continue

            frame_count += 1
            h, w = frame.shape[:2]
            elapsed = max(time.time() - fps_start, 1e-4)
            current_fps = frame_count / elapsed

            if preview:
                # Overlay diagnostic info on stream
                hud = f"ESP32-CAM ACTIVE | {w}x{h} | {current_fps:.1f} FPS"
                cv2.putText(frame, hud, (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3)
                cv2.putText(frame, hud, (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                cv2.imshow("ESP32-CAM Connection Test", frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:
                    print("\n[Preview Closed] User terminated test.")
                    break
            else:
                # Headless test: grab 15 frames to verify stability
                if frame_count >= 15:
                    print(f"[OK] Verified: Received {frame_count} frames at {w}x{h} ({current_fps:.1f} FPS).")
                    break

    finally:
        cap.release()
        cv2.destroyAllWindows()

    return True

def scan_for_esp32_cam():
    """Scan local network devices for ESP32-CAM stream."""
    print("=" * 60)
    print(" Scanning local network for ESP32-CAM...")
    print("=" * 60)
    
    candidates = find_candidate_ips()
    # Add fixed static and common hotspot IPs
    common_ips = ["192.168.137.200", "192.168.4.1", "192.168.137.10", "192.168.1.100"]
    for cip in common_ips:
        if cip not in candidates:
            candidates.append(cip)

    print(f"Found {len(candidates)} candidate IP addresses to probe.")
    
    found_urls = []
    for ip in candidates:
        # Check port 81 (standard ESP32-CAM MJPEG) and port 80
        for port in [81, 80]:
            if test_tcp_port(ip, port, timeout=0.3):
                test_url = f"http://{ip}:{port}/stream" if port == 81 else f"http://{ip}/stream"
                print(f" -> Found open web port on {ip}:{port}! Testing stream: {test_url}")
                found_urls.append(test_url)

    if not found_urls:
        print("\n❌ No active ESP32-CAM stream ports detected automatically.")
        print("Please specify the exact IP manually, for example:")
        print("  python test_cam_connection.py --ip <YOUR_ESP32_CAM_IP>")
        return None

    print(f"\nDiscovered {len(found_urls)} potential stream URLs:")
    for u in found_urls:
        print(f"  • {u}")
    return found_urls[0]

def main():
    parser = argparse.ArgumentParser(description="Test ESP32-CAM connection and live video stream")
    parser.add_argument("--ip", type=str, default=None, help="ESP32-CAM IP address (e.g. 192.168.137.10)")
    parser.add_argument("--port", type=int, default=81, help="Stream HTTP port (default: 81)")
    parser.add_argument("--url", type=str, default=None, help="Direct stream URL (e.g. http://192.168.137.10:81/stream)")
    parser.add_argument("--scan", action="store_true", help="Auto-scan local network for ESP32-CAM")
    parser.add_argument("--headless", action="store_true", help="Run without opening GUI preview window")
    args = parser.parse_args()

    print("=" * 65)
    print(" ESP32-CAM Connection & Live Stream Diagnostic Tool")
    print("=" * 65)

    stream_url = None

    if args.url:
        stream_url = args.url
    elif args.ip:
        ip = args.ip.strip()
        print(f"\n[1/3] Testing Network Ping to {ip}...")
        if ping_ip(ip):
            print(f"  [OK] Device at {ip} is responding to ping!")
        else:
            print(f"  [WARN] No ICMP ping response from {ip}. (Continuing test...)")

        print(f"\n[2/3] Checking HTTP Stream Port ({args.port}) on {ip}...")
        if test_tcp_port(ip, args.port, timeout=2.0):
            print(f"  [OK] Port {args.port} is OPEN and listening!")
        else:
            print(f"  [WARN] Port {args.port} did not accept TCP connection within 2 seconds.")

        stream_url = f"http://{ip}:{args.port}/stream"
    elif args.scan:
        stream_url = scan_for_esp32_cam()
    else:
        # Prompt user or default to config
        print("\nNo IP address or URL specified.")
        user_input = input("Enter your ESP32-CAM IP address (or press Enter to scan network): ").strip()
        if user_input:
            if user_input.startswith("http://"):
                stream_url = user_input
            else:
                stream_url = f"http://{user_input}:81/stream"
        else:
            stream_url = scan_for_esp32_cam()

    if not stream_url:
        print("\nExiting. Run with: python test_cam_connection.py --ip <IP_ADDRESS>")
        return

    print(f"\n[3/3] Testing Video Stream Feed: {stream_url}")
    test_camera_stream(stream_url, preview=not args.headless)

if __name__ == "__main__":
    main()
