"""
PCA9685 6-DOF Robotic Arm Joint Diagnostic Tester
=================================================
Tests each servo joint individually to verify power, wiring, and motion limits.
Target ESP32 DevKit: 192.168.137.204:8888
"""

import sys
import time
import socket

# Ensure UTF-8 output on Windows terminals (prevents UnicodeEncodeError)
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
if hasattr(sys.stderr, "reconfigure"):
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")

DEVKIT_IP = "192.168.137.204"
DEVKIT_PORT = 8888
ser_conn = None
is_serial = False

target_arg = sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith("-") else None
if not target_arg or "--bt" in sys.argv:
    try:
        import serial.tools.list_ports as lp
        for p in lp.comports():
            info = f"{p.device} {p.description}".upper()
            if "BTHENUM" in info or "BLUETOOTH" in info or "SMART_WASTE" in info or "STANDARD" in info:
                target_arg = p.device
                break
    except Exception:
        pass

if target_arg and target_arg.upper().startswith("COM"):
    try:
        import serial
        ser_conn = serial.Serial(target_arg, 115200, timeout=1.0)
        is_serial = True
        print(f"📶 [Bluetooth Serial Connected] DevKit arm active on {target_arg}!")
    except Exception as e:
        print(f"❌ [Serial Error on {target_arg}] {e}")
elif target_arg:
    DEVKIT_IP = target_arg

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
target = (DEVKIT_IP, DEVKIT_PORT)

def send_cmd(cmd: str):
    if is_serial and ser_conn and ser_conn.is_open:
        try:
            ser_conn.write((cmd.strip() + "\n").encode('utf-8'))
            ser_conn.flush()
            print(f"📶 [BT -> DevKit] --> {cmd}")
        except Exception as e:
            print(f"❌ [BT Error] {e}")
    else:
        try:
            sock.sendto(cmd.encode('utf-8'), target)
            print(f"📡 [Sent to ESP32 DevKit] --> {cmd}")
        except Exception as e:
            print(f"❌ [Error] {e}")

JOINTS = {
    0: ("Base (MG996R)", [45, 90, 135, 90]),
    1: ("Shoulder (MG996R)", [80, 100, 120, 100]),
    2: ("Elbow (MG996R)", [70, 90, 110, 90]),
    3: ("Wrist Pitch (MG90S/MG996R)", [70, 90, 110, 90]),
    4: ("Wrist Roll (MG90S)", [60, 90, 120, 90]),
    5: ("Gripper Claw (MG90S)", [30, 95, 30, 95, 30])
}

def test_joint(ch: int):
    if ch not in JOINTS:
        print(f"Unknown joint: {ch}")
        return
    name, angles = JOINTS[ch]
    print(f"\n🔄 Testing Channel {ch}: {name}")
    for ang in angles:
        print(f"   -> Moving to {ang}°...")
        send_cmd(f"ARM:SERVO:{ch}:{ang}")
        time.sleep(1.0)
    print(f"✅ Channel {ch} ({name}) test completed.\n")

def main():
    print("=" * 65)
    print("      PCA9685 6-DOF ROBOTIC ARM SERVO DIAGNOSTIC TESTER")
    print(f"      Target ESP32: {DEVKIT_IP}:{DEVKIT_PORT}")
    print("=" * 65)
    print(" Options:")
    print("   [0] Test Channel 0: Base Rotation (MG996R)")
    print("   [1] Test Channel 1: Shoulder Joint (MG996R)")
    print("   [2] Test Channel 2: Elbow Joint (MG996R)")
    print("   [3] Test Channel 3: Wrist Pitch")
    print("   [4] Test Channel 4: Wrist Roll")
    print("   [5] Test Channel 5: Claw Gripper (Open / Clamp)")
    print("   [A] Test ALL Joints in sequence")
    print("   [H] Reset to Home Position")
    print("   [P] Run Full Pick Sequence (Plastic)")
    print("   [C] Custom: Send custom channel and angle (format: <ch> <angle>)")
    print("   [Q] Quit")
    print("=" * 65)

    send_cmd("ARM:HOME")

    while True:
        try:
            choice = input("\nEnter joint to test (0-5 / A / H / P / C / Q): ").strip().upper()
            if not choice:
                continue

            if choice in ['0', '1', '2', '3', '4', '5']:
                test_joint(int(choice))
            elif choice == 'A':
                print("\n🚀 Testing all 6 joints in sequence...")
                for ch in range(6):
                    test_joint(ch)
                    time.sleep(0.5)
                send_cmd("ARM:HOME")
            elif choice == 'H':
                print("Sending Arm Home command...")
                send_cmd("ARM:HOME")
            elif choice == 'P':
                print("Executing Pick Sequence...")
                send_cmd("ARM:PICK:PLASTIC BOTTLE")
            elif choice == 'C':
                raw = input("Enter channel and angle (e.g. 5 30): ").strip()
                parts = raw.split()
                if len(parts) == 2 and parts[0].isdigit() and parts[1].isdigit():
                    send_cmd(f"ARM:SERVO:{parts[0]}:{parts[1]}")
                else:
                    print("Invalid format! Use: <channel> <angle>")
            elif choice == 'Q':
                print("Exiting...")
                break
            else:
                print("Invalid option.")
        except KeyboardInterrupt:
            break

    sock.close()

if __name__ == "__main__":
    main()
