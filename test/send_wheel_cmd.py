#!/usr/bin/env python3
import argparse
import json
import sys
import time

import serial

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0", help="ESP32 MAIN serial port (e.g. /dev/ttyUSB0, /dev/ttyACM0)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--hz", type=float, default=50.0)
    ap.add_argument("--fl", type=float, default=0.0)
    ap.add_argument("--fr", type=float, default=0.0)
    ap.add_argument("--rl", type=float, default=0.0)
    ap.add_argument("--rr", type=float, default=0.0)
    ap.add_argument("--duration", type=float, default=5.0, help="seconds; 0 = run forever")
    ap.add_argument("--ramp", type=float, default=0.0, help="seconds ramp from 0 to target")
    ap.add_argument("--print_rx", action="store_true", help="print lines received from ESP32")
    args = ap.parse_args()

    target = [args.fl, args.fr, args.rl, args.rr]

    ser = serial.Serial(args.port, args.baud, timeout=0.0)
    time.sleep(0.2)
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    dt = 1.0 / args.hz
    t0 = time.time()
    next_t = t0

    def scale_target(alpha: float):
        return [alpha * v for v in target]

    print(f"[TX] port={args.port} baud={args.baud} hz={args.hz} target={target} duration={args.duration}s ramp={args.ramp}s")
    print("[TX] Format: {\"wheel_cmd\":[v_fl,v_fr,v_rl,v_rr]} (m/s) newline-delimited")

    try:
        while True:
            now = time.time()
            elapsed = now - t0

            if args.duration > 0 and elapsed >= args.duration:
                break

            # ramp
            if args.ramp > 0:
                alpha = min(1.0, elapsed / args.ramp)
                cmd = scale_target(alpha)
            else:
                cmd = target

            msg = json.dumps({"wheel_cmd": cmd}, separators=(",", ":")) + "\n"
            ser.write(msg.encode("utf-8"))

            # optional RX print (non-blocking)
            if args.print_rx:
                line = ser.readline()
                if line:
                    try:
                        sys.stdout.write(line.decode("utf-8", errors="replace"))
                        sys.stdout.flush()
                    except Exception:
                        pass

            next_t += dt
            sleep_s = next_t - time.time()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                # ถ้าหลุดเวลา ให้รีเซ็ต schedule
                next_t = time.time()

    except KeyboardInterrupt:
        pass
    finally:
        # ส่ง stop 10 ครั้งกันหลุด (เพราะ cmd timeout/serial jitter)
        stop = json.dumps({"wheel_cmd":[0.0,0.0,0.0,0.0]}, separators=(",", ":")) + "\n"
        for _ in range(10):
            ser.write(stop.encode("utf-8"))
            time.sleep(0.02)
        ser.close()
        print("\n[TX] stop sent, done.")

if __name__ == "__main__":
    main()