"""
Interactive Wireless Robot & 6-DOF Robotic Arm Remote Control Tester
=====================================================================
Supports FULL drive combinations:

 MOVEMENT (type the keys then press Enter):
   [W]        All 4 wheels drive FORWARD
   [S]        All 4 wheels drive BACKWARD (REVERSE)
   [A]        Pivot steer LEFT in place
   [D]        Pivot steer RIGHT in place
   [WA]       Forward + Curve LEFT  (right wheels full, left wheels stop)
   [WD]       Forward + Curve RIGHT (left wheels full, right wheels stop)
   [SA]       Backward + Curve LEFT
   [SD]       Backward + Curve RIGHT
   [X]        STOP all motors
   [Space]    STOP all motors

 6-DOF ARM:
   [1]        Pick & Sort PLASTIC (Bin 1 - 165 deg Left)
   [2]        Pick & Sort METAL / CAN (Bin 2 - 135 deg Right)
   [3]        Pick & Sort GLASS / GENERAL (Bin 3 - 15 deg Center)
   [H]        Reset Arm to HOME position
   [J]        Manual Joint Test  (enter: <channel> <angle>)

 SENSORS:
   [T]        Query Live Telemetry (Ultrasonic distance)
   [I]        I2C Scan (Check PCA9685 Servo Driver at 0x40)
   [Q]        Quit
"""

import socket
import time
import sys

DEVKIT_IP   = "192.168.137.204"
DEVKIT_PORT = 8888
ser_conn    = None
is_serial   = False

# Check for Bluetooth / Serial override or auto-detection
target_arg = sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith("-") else None
if "--bt" in sys.argv:
    try:
        import serial.tools.list_ports as lp
        for p in lp.comports():
            info = f"{p.device} {p.description}".upper()
            if "BTHENUM" in info or "BLUETOOTH" in info or "SMART_WASTE" in info or "STANDARD" in info:
                target_arg = p.device
                break
    except Exception: pass

if target_arg and target_arg.upper().startswith("COM"):
    try:
        import serial
        ser_conn = serial.Serial(target_arg, 115200, timeout=1.5)
        is_serial = True
        print(f"  📶 [Bluetooth Serial Connected] DevKit active on {target_arg} (Zero IP needed!)")
    except Exception as e:
        print(f"  ❌ [Serial Connect Error on {target_arg}] {e}")
        ser_conn = None
elif target_arg:
    DEVKIT_IP = target_arg

sock   = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
target = (DEVKIT_IP, DEVKIT_PORT)

# ─────────────────────────────────────────────────────────────
# Communications helpers (Bluetooth Serial OR UDP)
# ─────────────────────────────────────────────────────────────
def send_cmd(cmd: str, silent: bool = False):
    if is_serial and ser_conn and ser_conn.is_open:
        try:
            ser_conn.write((cmd.strip() + "\n").encode("utf-8"))
            ser_conn.flush()
            if not silent:
                print(f"  📶 [Sent via BT {ser_conn.port}]  {cmd}")
        except Exception as e:
            print(f"  ❌ [BT Serial Error] {e}")
    else:
        try:
            sock.sendto(cmd.encode("utf-8"), target)
            if not silent:
                print(f"  📡 [Sent -> {DEVKIT_IP}:{DEVKIT_PORT}]  {cmd}")
        except Exception as e:
            print(f"  ❌ [UDP Error] {e}")

def get_telemetry():
    send_cmd("CMD:TELEMETRY", silent=True)
    if is_serial and ser_conn and ser_conn.is_open:
        try:
            time.sleep(0.1)
            msg = ser_conn.readline().decode("utf-8", errors="ignore").strip()
            if msg:
                print(f"\n  📊 [TELEMETRY via BT]\n     {msg}\n")
            else:
                print("  ⏳ No response – is DevKit powered on?")
        except Exception as e:
            print(f"  ❌ [Error] {e}")
    else:
        sock.settimeout(1.5)
        try:
            data, addr = sock.recvfrom(1024)
            msg = data.decode("utf-8", errors="ignore")
            print(f"\n  📊 [TELEMETRY from {addr[0]}]\n     {msg}\n")
        except socket.timeout:
            print("  ⏳ No response – is the DevKit powered on and on the hotspot?")
        except Exception as e:
            print(f"  ❌ [Error] {e}")
        finally:
            sock.settimeout(None)

def i2c_scan():
    send_cmd("CMD:I2C_SCAN", silent=True)
    if is_serial and ser_conn and ser_conn.is_open:
        try:
            time.sleep(0.15)
            msg = ser_conn.readline().decode("utf-8", errors="ignore").strip()
            print(f"\n  🔌 [I2C via BT]\n     {msg}\n")
        except Exception as e:
            print(f"  ❌ [Error] {e}")
    else:
        sock.settimeout(1.5)
        try:
            data, addr = sock.recvfrom(1024)
            msg = data.decode("utf-8", errors="ignore")
            print(f"\n  🔌 [I2C from {addr[0]}]\n     {msg}\n")
        except socket.timeout:
            print("  ⏳ No I2C response – is the DevKit connected?")
        except Exception as e:
            print(f"  ❌ [Error] {e}")
        finally:
            sock.settimeout(None)

# ─────────────────────────────────────────────────────────────
# Full drive command map (order-insensitive combos)
# ─────────────────────────────────────────────────────────────
DRIVE_CMDS = {
    # Single key
    "W":  ("CMD:FORWARD",        "All 4 wheels FORWARD"),
    "S":  ("CMD:BACKWARD",       "All 4 wheels REVERSE"),
    "A":  ("CMD:LEFT",           "Pivot steer LEFT in place (FR+RR FWD, FL+RL REV)"),
    "D":  ("CMD:RIGHT",          "Pivot steer RIGHT in place (FL+RL FWD, FR+RR REV)"),
    "Q":  ("CMD:FORWARD_LEFT",   "Forward Left [Q] (FR+RR rotate forward, FL+RL stop)"),
    "E":  ("CMD:FORWARD_RIGHT",  "Forward Right [E] (FL+RL rotate forward, FR+RR stop)"),
    "Z":  ("CMD:BACK_LEFT",      "Reverse Left [Z] (FR+RR rotate reverse, FL+RL stop)"),
    "C":  ("CMD:BACK_RIGHT",     "Reverse Right [C] (FL+RL rotate reverse, FR+RR stop)"),
    "X":  ("CMD:STOP",           "STOP all motors"),
    " ":  ("CMD:STOP",           "STOP all motors"),

    # Combos – WA and AW both mean the same thing
    "WA": ("CMD:FORWARD_LEFT",   "Forward + Curve LEFT  (FR+RR forward, FL+RL stop)"),
    "AW": ("CMD:FORWARD_LEFT",   "Forward + Curve LEFT  (FR+RR forward, FL+RL stop)"),
    "WD": ("CMD:FORWARD_RIGHT",  "Forward + Curve RIGHT (FL+RL forward, FR+RR stop)"),
    "DW": ("CMD:FORWARD_RIGHT",  "Forward + Curve RIGHT (FL+RL forward, FR+RR stop)"),
    "SA": ("CMD:BACK_LEFT",      "Reverse + Curve LEFT  (FR+RR reverse, FL+RL stop)"),
    "AS": ("CMD:BACK_LEFT",      "Reverse + Curve LEFT  (FR+RR reverse, FL+RL stop)"),
    "SD": ("CMD:BACK_RIGHT",     "Reverse + Curve RIGHT (FL+RL reverse, FR+RR stop)"),
    "DS": ("CMD:BACK_RIGHT",     "Reverse + Curve RIGHT (FL+RL reverse, FR+RR stop)"),
}

ARM_CMDS = {
    "1": ("ARM:PICK:DEGRADABLE",         "Pick Ground Object → Deposit in RIGHT Bin (Degradable / Banana Peel · 180° Right)"),
    "2": ("ARM:PICK:NON_DEGRADABLE",     "Pick Ground Object → Deposit in LEFT Bin (Non-Degradable · 180° Left)"),
    "BR": ("ARM:BIN:RIGHT",              "Rotate 180° Right (CW) → Deposit in Degradable Bin"),
    "BL": ("ARM:BIN:LEFT",               "Rotate 180° Left (CCW) → Deposit in Non-Degradable Bin"),
    "L": ("ARM:PICK_TARGET:NON_DEGRADABLE:LEFT:200",  "Test Pick LEFT side item → Deposit in LEFT Bin (180° Left)"),
    "R": ("ARM:PICK_TARGET:DEGRADABLE:RIGHT:200",    "Test Pick RIGHT side item → Deposit in RIGHT Bin (180° Right)"),
    "H": ("ARM:HOME",                    "Reset 6-DOF Arm to HOME pose"),
}

claw_is_open = True

# ─────────────────────────────────────────────────────────────
def print_header():
    print("\n" + "=" * 66)
    print("  🚛  Wireless Smart Waste Truck - Dual Dustbin Controller")
    print(f"  Target DevKit : {DEVKIT_IP}:{DEVKIT_PORT}")
    print("=" * 66)
    print("  ── DRIVE (Single & Combos) ──────────────────────────────")
    print("   W  = Forward             S  = Reverse")
    print("   A  = Pivot Left          D  = Pivot Right")
    print("   WA = Forward + Left      WD = Forward + Right")
    print("   SA = Reverse + Left      SD = Reverse + Right")
    print("   X  = STOP (Space also stops)")
    print()
    print("  ── DUAL DUSTBIN ROBOTIC ARM ──────────────────────────────")
    print("   1  = Pick & Deposit in RIGHT Bin (Degradable / Banana Peel · 180° Right)")
    print("   2  = Pick & Deposit in LEFT Bin  (Non-Degradable / Plastic · 180° Left)")
    print("   BR = Empty into RIGHT Dustbin (180° Right CW)")
    print("   BL = Empty into LEFT Dustbin  (180° Left CCW)")
    print("   L  = Pick from Left Ground -> Left Bin")
    print("   R  = Pick from Right Ground -> Right Bin")
    print("   H  = Reset Arm to HOME Driving Position")
    print("   C  = Toggle Gripper Claw (Open / Close)")
    print("   B  = Test 360° Base Rotation (SG996R)")
    print("   K  = Calibrate 180° Bin Turn Time (ms)")
    print("   J  = Manual Joint Test [CH0: speed -100..+100 | CH1-5: 0..180°]")
    print()
    print("  ── SYSTEM & SENSORS ──────────────────────────────────────")
    print("   P = Set Motor PWM Speed (0-255) [default 180, ~70%]")
    print("   T = Live Ultrasonic Distance Telemetry")
    print("   I = I2C Bus Scan (Check PCA9685 at 0x40)")
    print("   Q = Quit & Stop All Motors")
    print("=" * 66)

# ─────────────────────────────────────────────────────────────
def main():
    print_header()
    send_cmd("CMD:STOP")   # safety stop on connect

    while True:
        try:
            raw = input("\n  Enter key(s): ").strip().upper()
            if not raw:
                continue

            # ── Drive commands ───────────────────────────────
            if raw in DRIVE_CMDS:
                cmd, desc = DRIVE_CMDS[raw]
                print(f"  🔸 {desc}")
                send_cmd(cmd)

            # ── Arm preset sequences ─────────────────────────
            elif raw in ARM_CMDS:
                cmd, desc = ARM_CMDS[raw]
                print(f"  🦾 {desc}")
                send_cmd(cmd)

            # ── Claw Open / Close Toggle ─────────────────────
            elif raw == "C":
                claw_is_open = not claw_is_open
                angle = 30 if claw_is_open else 105
                action = "OPEN" if claw_is_open else "CLOSED (Clamped)"
                print(f"  🦀 Claw {action} (CH5 → {angle}°)")
                send_cmd(f"ARM:SERVO:5:{angle}")

            # ── Calibrate 180° Rear Turn Duration ────────────
            elif raw == "K":
                val = input("  Enter 180° rotation duration in milliseconds (e.g. 1100): ").strip()
                if val.isdigit() and int(val) > 100:
                    ms = int(val)
                    print(f"  ⏱️ Calibrating 180° turn duration → {ms} ms")
                    send_cmd(f"ARM:CALIB:180:{ms}")
                else:
                    print("  ❌ Enter a valid millisecond number (e.g. 1000, 1100, 1200)")

            # ── Manual joint test ────────────────────────────
            elif raw == "J":
                joints = input(
                    "  CH0=base speed(-100..+100), CH1-5=angle(0..180). Enter: channel value: "
                ).strip()
                parts = joints.split()
                if len(parts) == 2:
                    ch, val = int(parts[0]), int(parts[1])
                    cmd = f"ARM:SERVO:{ch}:{val}"
                    label = f"speed {val}%" if ch == 0 else f"angle {val}°"
                    print(f"  🦾 CH{ch} → {label}")
                    send_cmd(cmd)
                else:
                    print("  ❌ Format: <channel> <value>   e.g.  0 50  or  2 90")

            # ── 360° Base rotation test ─────────────────────
            elif raw == "B":
                print("  🔄 Base 360° test  (SG996R continuous rotation)")
                print("     CW <speed> <ms>    rotate clockwise for duration")
                print("     CCW <speed> <ms>   rotate counter-clockwise")
                print("     STOP               halt immediately")
                print("     Example: CW 60 1000  = 60% speed, 1 second")
                inp = input("  Enter: ").strip().upper()
                parts = inp.split()
                if not parts:
                    pass
                elif parts[0] == "STOP":
                    send_cmd("ARM:BASE:STOP")
                elif len(parts) >= 3 and parts[0] in ("CW", "CCW"):
                    print(f"  🔄 Base {parts[0]} at {parts[1]}% for {parts[2]} ms")
                    send_cmd(f"ARM:BASE:{parts[0]}:{parts[1]}:{parts[2]}")
                elif len(parts) == 2 and parts[0] in ("CW", "CCW"):
                    print(f"  🔄 Base continuous {parts[0]} at {parts[1]}% (B STOP to halt)")
                    send_cmd(f"ARM:BASE:{parts[0]}:{parts[1]}")
                else:
                    print("  ❌ Usage:  CW 60 1000  |  CCW 60 500  |  STOP")

            # ── Sensors ──────────────────────────────────────
            elif raw == "T":
                get_telemetry()
            elif raw == "I":
                i2c_scan()

            # ── Speed control ────────────────────────────────
            elif raw == "P":
                val = input("  Enter speed (0-255): ").strip()
                if val.isdigit() and 0 <= int(val) <= 255:
                    spd = int(val)
                    pct = round(spd * 100 / 255)
                    print(f"  ⚙️  Setting motor speed → {spd}/255 ({pct}%)")
                    send_cmd(f"CMD:SPEED:{spd}")
                else:
                    print("  ❌ Enter a number between 0 and 255")

            # ── Quit ─────────────────────────────────────────
            elif raw == "Q":
                print("  Sending STOP and exiting...")
                send_cmd("CMD:STOP")
                break

            else:
                print(f"  ❓ Unknown: '{raw}'  –  valid drive keys: W S A D X WA WD SA SD")

        except KeyboardInterrupt:
            send_cmd("CMD:STOP")
            print("\n  Interrupted – motors stopped.")
            break

    sock.close()


if __name__ == "__main__":
    main()
