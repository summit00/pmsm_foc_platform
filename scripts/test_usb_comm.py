import sys
import time
import struct
import json
import os
import serial
import serial.tools.list_ports

def find_stm32_port():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        # Check VID/PID 0483:5740 or description
        if p.vid == 0x0483 and p.pid == 0x5740:
            return p.device
        if "STMicroelectronics" in p.description or "Virtual COM" in p.description:
            return p.device
    if ports:
        return ports[0].device
    return None

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_stm32_port()
    if not port:
        print("[-] No serial port found. Please connect the STM32 board via USB.")
        sys.exit(1)

    print(f"[+] Opening serial port: {port} ...")
    try:
        ser = serial.Serial(port, 115200, timeout=0.1)
    except Exception as e:
        print(f"[-] Failed to open {port}: {e}")
        sys.exit(1)

    # Load telemetry scale definitions
    script_dir = os.path.dirname(os.path.abspath(__file__))
    json_path = os.path.join(os.path.dirname(script_dir), "app", "telemetry_registry.json")
    id_map = {}
    if os.path.exists(json_path):
        with open(json_path, "r") as f:
            data = json.load(f)
            for entry in data.get("telemetry", []):
                id_map[entry["id"]] = entry

    print("[+] Selecting telemetry channels (including errorState, temp_C, Udc_V, etc.) ...")
    # SELECT_MAGIC (0xABCE): Dynamic ID selection (10 x uint16_t + 20 bytes padding = 44 bytes)
    selected_ids = [1, 2, 3, 4, 7, 8, 20, 21, 22, 0] # 1:Udc, 2:demandSpeed, 3:feedbackSpeed, 4:encoder, 7:Id, 8:Iq, 20:temp, 21:errorState, 22:autoSetup
    select_frame = struct.pack("<HH10H20s", 0xABCE, 1, *selected_ids, b"\x00" * 20)
    ser.write(select_frame)
    time.sleep(0.05)

    print("[+] Sending command frame to MCU: Enable=1, TargetSpeed=1500 RPM, Accel=500, IsAbs=2000 mA ...")
    # Command frame: RX_MAGIC (0xABCD), seq, 10 int32 payload values
    # Payload format: [enable, mode, targetSpeed*100, accel*100, isAbs*10, 0, 0, 0, 0, 0]
    payload = [
        1,              # Enable
        0,              # Mode
        int(1500.0 * 100),  # Target Speed (scaled x100)
        int(500.0 * 100),   # Accel (scaled x100)
        int(2000.0 * 10),   # Current limit (scaled x10)
        0, 0, 0, 0, 0
    ]
    cmd_frame = struct.pack("<HH10i", 0xABCD, 2, *payload)
    ser.write(cmd_frame)

    print("[+] Listening for Rx Telemetry Stream from MCU (magic 0xDCBA)...")
    start_time = time.time()
    rx_frames = 0
    samples_received = {}
    buf = b""

    while time.time() - start_time < 3.0:
        chunk = ser.read(1024)
        if not chunk:
            continue
        buf += chunk

        while len(buf) >= 6:
            idx = buf.find(struct.pack("<H", 0xDCBA))
            if idx == -1:
                buf = buf[-1:]
                break
            if idx > 0:
                buf = buf[idx:]
                if len(buf) < 6:
                    break

            magic, seq, count = struct.unpack_from("<HHH", buf, 0)
            if not (1 <= count <= 240):
                buf = buf[2:]
                continue

            expected_len = 6 + count * 3
            if len(buf) < expected_len:
                break

            rx_frames += 1
            for i in range(count):
                sid, val = struct.unpack_from("<Bh", buf, 6 + i * 3)
                samples_received[sid] = val

            buf = buf[expected_len:]

    ser.close()

    print(f"\n[+] Received {rx_frames} telemetry packets in ~3 seconds!")
    if samples_received:
        print("\n[+] Decoded live telemetry values:")
        for sid, raw_val in sorted(samples_received.items()):
            meta = id_map.get(sid, {})
            name = meta.get("name", f"ID_{sid}")
            scale = meta.get("scale", 1.0)
            unit = meta.get("unit", "")
            phys_val = raw_val / scale if scale != 0 else raw_val
            print(f"    - {name:<20}: {phys_val:10.2f} {unit}")

        if 21 in samples_received:
            err = samples_received[21]
            if err == 0:
                print("\n[+] Gate Driver (DRV8353) Check: OK (All SPI registers verified, nFAULT PB12 is HIGH, errorState = 0)")
            elif err == 1:
                print(f"\n[-] Gate Driver (DRV8353) Check: ERROR (SPI readback mismatch, errorState = {err})")
            elif err == 2:
                print(f"\n[-] Gate Driver (DRV8353) Check: HARDWARE FAULT (DRV_Fault PB12 pin is LOW, errorState = {err})")
            else:
                print(f"\n[-] Gate Driver (DRV8353) Check: UNKNOWN ERROR (errorState = {err})")

        print("\n[SUCCESS] Bi-directional USB communication is fully operational!")
    else:
        print("\n[-] No valid telemetry packets received. Please verify board is flashed and connected.")

if __name__ == "__main__":
    main()
