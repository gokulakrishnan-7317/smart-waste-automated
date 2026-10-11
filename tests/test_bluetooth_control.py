"""
Bluetooth Diagnostic & Wireless Control Tool
===========================================
Connects directly to the ESP32 DevKit ("SMART_WASTE_ROBOT") via Bluetooth Serial (SPP).
Zero IP Address Configuration Required!
"""

import sys
import time
import serial
import serial.tools.list_ports as lp

# Ensure UTF-8 output on Windows terminals (prevents UnicodeEncodeError)
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
if hasattr(sys.stderr, "reconfigure"):
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")

def scan_ports():
    print("\n🔍 Scanning available Serial / Bluetooth COM ports...")
    ports = list(lp.comports())
    if not ports:
        print("  ❌ No COM ports found in Windows.")
        print("     To connect via Bluetooth:")
        print("     1. Turn on ESP32 DevKit.")
        print("     2. Open Windows Settings -> Bluetooth & devices -> Add device.")
        print("     3. Select 'SMART_WASTE_ROBOT' and click Pair (PIN: 1234 or none).")
        return []
    
    found = []
    for p in ports:
        desc = f"{p.device}: {p.description} [{p.hwid}]"
        print(f"  • {desc}")
        found.append(p.device)
    return found

def connect(port_name=None):
    if port_name is None:
        ports = scan_ports()
        if not ports:
            return None
        # Prefer Bluetooth ports
        bt_ports = [p.device for p in lp.comports() if "BTHENUM" in p.hwid.upper() or "BLUETOOTH" in p.description.upper()]
        port_name = bt_ports[0] if bt_ports else ports[0]

    print(f"\n🔌 Attempting to open Bluetooth Serial on {port_name} (115200 baud)...")
    try:
        ser = serial.Serial(port_name, 115200, timeout=1.5)
        time.sleep(0.5)
        print(f"✅ Successfully connected to {port_name}!")
        return ser
    except Exception as e:
        print(f"❌ Failed to open {port_name}: {e}")
        return None

def main():
    print("=" * 65)
    print(" Smart Waste Robot - Bluetooth Serial Diagnostic Tool")
    print(" DevKit Target Name: 'SMART_WASTE_ROBOT' (Zero IP Needed)")
    print("=" * 65)

    target_port = sys.argv[1] if len(sys.argv) > 1 else None
    ser = connect(target_port)

    if not ser:
        print("\n[Pairing Guide]:")
        print(" 1. Go to Windows Settings -> Bluetooth & devices.")
        print(" 2. Turn Bluetooth ON and click 'Add device' -> 'Bluetooth'.")
        print(" 3. Choose 'SMART_WASTE_ROBOT'.")
        print(" 4. Once paired, run this script again: python test_bluetooth_control.py")
        sys.exit(1)

    print("\n📡 Testing connection with Ping & Telemetry...")
    try:
        ser.write(b"CMD:PING\n")
        ser.flush()
        time.sleep(0.3)
        reply = ser.readline().decode('utf-8', errors='ignore').strip()
        print(f"  Ping Response: {reply or '(no response)'}")

        ser.write(b"CMD:TELEMETRY\n")
        ser.flush()
        time.sleep(0.3)
        reply = ser.readline().decode('utf-8', errors='ignore').strip()
        print(f"  Telemetry Response: {reply or '(no response)'}")
    except Exception as e:
        print(f"  Communication error: {e}")

    print("\nControls:")
    print(" [W] Forward   [S] Backward   [A] Turn Left   [D] Turn Right")
    print(" [1] Test Right Bin (Degradable · 180° Right)")
    print(" [2] Test Left Bin  (Non-Degradable · 180° Left)")
    print(" [P] Pick Degradable (Banana)   [N] Pick Non-Degradable")
    print(" [Space] Stop   [H] Arm Home    [Q] Quit")
    print("-" * 65)

    try:
        while True:
            cmd = input("Enter command (or W/S/A/D/1/2/P/N/H/Q): ").strip().upper()
            if not cmd:
                continue
            if cmd == "Q":
                break
            elif cmd == "W":
                ser.write(b"CMD:FORWARD\n")
            elif cmd == "S":
                ser.write(b"CMD:BACKWARD\n")
            elif cmd == "A":
                ser.write(b"CMD:LEFT\n")
            elif cmd == "D":
                ser.write(b"CMD:RIGHT\n")
            elif cmd in (" ", "STOP"):
                ser.write(b"CMD:STOP\n")
            elif cmd in ("1", "R"):
                ser.write(b"ARM:BIN:RIGHT\n")
            elif cmd in ("2", "L"):
                ser.write(b"ARM:BIN:LEFT\n")
            elif cmd == "P":
                ser.write(b"ARM:PICK:DEGRADABLE\n")
            elif cmd == "N":
                ser.write(b"ARM:PICK:NON_DEGRADABLE\n")
            elif cmd == "H":
                ser.write(b"ARM:HOME\n")
            else:
                ser.write((cmd + "\n").encode('utf-8'))
            ser.flush()
            time.sleep(0.1)
            while ser.in_waiting > 0:
                print("  DevKit ->", ser.readline().decode('utf-8', errors='ignore').strip())
    except KeyboardInterrupt:
        pass
    finally:
        ser.write(b"CMD:STOP\n")
        ser.close()
        print("\nClosed Bluetooth Serial connection.")

if __name__ == "__main__":
    main()
