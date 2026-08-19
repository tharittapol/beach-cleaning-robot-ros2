#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any

try:
    import serial
except ImportError as exc:
    sys.stderr.write(
        'pyserial is required.\n'
        'Install it with:\n'
        '  python3 -m pip install pyserial\n'
    )
    raise SystemExit(2) from exc

THIS_DIR = Path(__file__).resolve().parent
DEFAULT_SPEC_PATH = THIS_DIR / 'kp_pass_stage1.json'
DEFAULT_LOG_DIR = THIS_DIR / 'logs'


def timestamp() -> str:
    return datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]


def print_status(message: str) -> None:
    print(f'[runner] {message}')


def log_line(log_fh, prefix: str, text: str, echo: bool = False) -> None:
    line = f'{timestamp()} {prefix} {text}'
    log_fh.write(line + '\n')
    log_fh.flush()
    if echo:
        print(line)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Run a sequence of ESP32 PID/STEP wheel tests and save the raw serial log.'
    )
    parser.add_argument('--port', default='/dev/ttyUSB0', help='ESP32 main USB serial port')
    parser.add_argument('--baud', type=int, default=115200, help='serial baud rate')
    parser.add_argument(
        '--spec',
        default=str(DEFAULT_SPEC_PATH),
        help='path to JSON array of wheel test steps',
    )
    parser.add_argument('--log', help='optional explicit log file path')
    parser.add_argument('--timeout', type=float, default=0.10, help='serial readline timeout in seconds')
    parser.add_argument('--warmup-s', type=float, default=0.50, help='wait after opening serial port')
    parser.add_argument('--command-gap-s', type=float, default=0.20, help='gap between transmitted commands')
    parser.add_argument('--post-roll-s', type=float, default=0.80, help='extra logging after wheel_test becomes inactive')
    parser.add_argument('--dbgrate-ms', type=int, default=100, help='DBGRATE value to send before the run')
    parser.add_argument('--print-rx', action='store_true', help='echo received serial lines to the terminal')
    return parser.parse_args()


def build_log_path(spec_path: Path, explicit_log: str | None) -> Path:
    if explicit_log:
        return Path(explicit_log).expanduser().resolve()

    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    return DEFAULT_LOG_DIR / f'{spec_path.stem}_{stamp}.log'


def load_steps(path: Path) -> list[dict[str, Any]]:
    data = json.loads(path.read_text(encoding='utf-8'))
    if not isinstance(data, list) or not data:
        raise ValueError(f'{path} must contain a non-empty JSON array')

    steps: list[dict[str, Any]] = []
    for index, item in enumerate(data, start=1):
        if not isinstance(item, dict):
            raise ValueError(f'step {index} in {path} must be a JSON object')

        vmax_value = item.get('vmax_mps', None)
        step = {
            'label': str(item.get('label', f'step_{index}')),
            'wheel': int(item['wheel']),
            'kp': float(item['kp']),
            'ki': float(item.get('ki', 0.0)),
            'kd': float(item.get('kd', 0.0)),
            'vmax_mps': None if vmax_value is None else float(vmax_value),
            'target_mps': float(item['target_mps']),
            'duration_ms': int(item['duration_ms']),
            'closed_loop': 1 if int(item.get('closed_loop', 1)) != 0 else 0,
        }

        if step['wheel'] < 1 or step['wheel'] > 4:
            raise ValueError(f'step {index} uses invalid wheel {step["wheel"]}; expected 1..4')
        if step['duration_ms'] <= 0:
            raise ValueError(f'step {index} duration_ms must be > 0')

        for key in ('kp', 'ki', 'kd', 'target_mps'):
            if not math.isfinite(step[key]):
                raise ValueError(f'step {index} field {key} must be finite')
        if step['vmax_mps'] is not None and not math.isfinite(step['vmax_mps']):
            raise ValueError(f'step {index} field vmax_mps must be finite when provided')

        steps.append(step)

    return steps


def send_command(ser: serial.Serial, log_fh, command: str, delay_s: float) -> None:
    line = command.rstrip('\n') + '\n'
    ser.write(line.encode('utf-8'))
    ser.flush()
    log_line(log_fh, '[TX]', command, echo=True)
    if delay_s > 0.0:
        time.sleep(delay_s)


def read_one_line(ser: serial.Serial) -> str | None:
    raw = ser.readline()
    if not raw:
        return None

    text = raw.decode('utf-8', errors='replace').rstrip('\r\n')
    return text if text else None


def drain_lines(ser: serial.Serial, log_fh, seconds: float, print_rx: bool) -> None:
    deadline = time.monotonic() + max(0.0, seconds)
    while time.monotonic() < deadline:
        text = read_one_line(ser)
        if text is None:
            time.sleep(0.01)
            continue
        log_line(log_fh, '[RX]', text, echo=print_rx)


def parse_json_line(text: str) -> dict[str, Any] | None:
    if not text.startswith('{'):
        return None
    try:
        data = json.loads(text)
    except json.JSONDecodeError:
        return None
    return data if isinstance(data, dict) else None


def extract_wheel_test(data: dict[str, Any] | None) -> dict[str, Any] | None:
    if not isinstance(data, dict):
        return None

    debug_obj = data.get('debug')
    if isinstance(debug_obj, dict):
        wheel_test = debug_obj.get('wheel_test')
        if isinstance(wheel_test, dict):
            return wheel_test

    wheel_test = data.get('wheel_test')
    if isinstance(wheel_test, dict):
        return wheel_test

    return None


def run_step(
    ser: serial.Serial,
    log_fh,
    step: dict[str, Any],
    step_index: int,
    step_total: int,
    command_gap_s: float,
    post_roll_s: float,
    print_rx: bool,
) -> None:
    wheel = step['wheel']
    kp = step['kp']
    ki = step['ki']
    kd = step['kd']
    target_mps = step['target_mps']
    duration_ms = step['duration_ms']
    closed_loop = step['closed_loop']
    label = step['label']
    vmax_mps = step.get('vmax_mps')

    status = (
        f'[{step_index}/{step_total}] {label}: '
        f'PID {wheel} {kp:.3f} {ki:.3f} {kd:.3f}'
    )
    if vmax_mps is not None:
        status += f' | VMAX {wheel} {vmax_mps:.3f}'
    status += f' | STEP {wheel} {target_mps:.3f} {duration_ms} {closed_loop}'
    print_status(status)

    if vmax_mps is not None:
        send_command(ser, log_fh, f'VMAX {wheel} {vmax_mps:.3f}', command_gap_s)
    send_command(ser, log_fh, f'PID {wheel} {kp:.3f} {ki:.3f} {kd:.3f}', command_gap_s)
    send_command(ser, log_fh, f'STEP {wheel} {target_mps:.3f} {duration_ms} {closed_loop}', 0.0)

    hard_deadline = time.monotonic() + (duration_ms / 1000.0) + post_roll_s + 5.0
    saw_active = False
    stop_after: float | None = None

    while True:
        now = time.monotonic()
        if stop_after is not None and now >= stop_after:
            break
        if now >= hard_deadline:
            break

        text = read_one_line(ser)
        if text is None:
            continue

        log_line(log_fh, '[RX]', text, echo=print_rx)

        wheel_test = extract_wheel_test(parse_json_line(text))
        if wheel_test is None:
            continue

        try:
            active = bool(int(wheel_test.get('active', 0)))
        except (TypeError, ValueError):
            active = bool(wheel_test.get('active'))

        try:
            active_wheel = int(wheel_test.get('wheel', 0) or 0)
        except (TypeError, ValueError):
            active_wheel = 0

        if active and active_wheel == wheel:
            saw_active = True
        elif saw_active and not active:
            stop_after = time.monotonic() + max(0.0, post_roll_s)

    if saw_active:
        print_status(f'Wheel {wheel} capture complete.')
    else:
        print_status(f'Wheel {wheel} timed out before a matching active wheel_test frame was seen.')


def main() -> int:
    args = parse_args()
    spec_path = Path(args.spec).expanduser().resolve()
    log_path = build_log_path(spec_path, args.log)
    log_path.parent.mkdir(parents=True, exist_ok=True)

    steps = load_steps(spec_path)

    ser: serial.Serial | None = None
    with log_path.open('w', encoding='utf-8') as log_fh:
        log_line(log_fh, '[INFO]', f'spec={spec_path}', echo=True)
        log_line(log_fh, '[INFO]', f'log={log_path}', echo=True)
        log_line(log_fh, '[INFO]', f'port={args.port} baud={args.baud}', echo=True)

        try:
            ser = serial.Serial(args.port, args.baud, timeout=args.timeout, write_timeout=args.timeout)
            print_status(f'Connected to {args.port} @ {args.baud}')
            time.sleep(max(0.0, args.warmup_s))
            ser.reset_input_buffer()
            ser.reset_output_buffer()

            send_command(ser, log_fh, 'STOP', args.command_gap_s)
            send_command(ser, log_fh, 'DBG 1', args.command_gap_s)
            send_command(ser, log_fh, f'DBGRATE {args.dbgrate_ms}', args.command_gap_s)
            send_command(ser, log_fh, 'STATUS', args.command_gap_s)
            drain_lines(ser, log_fh, 0.50, args.print_rx)

            for index, step in enumerate(steps, start=1):
                run_step(
                    ser=ser,
                    log_fh=log_fh,
                    step=step,
                    step_index=index,
                    step_total=len(steps),
                    command_gap_s=max(0.0, args.command_gap_s),
                    post_roll_s=max(0.0, args.post_roll_s),
                    print_rx=args.print_rx,
                )

            send_command(ser, log_fh, 'STOP', args.command_gap_s)
            drain_lines(ser, log_fh, max(0.0, args.post_roll_s), args.print_rx)

        except KeyboardInterrupt:
            print_status('Interrupted by user; sending STOP and closing serial.')
            if ser is not None and ser.is_open:
                try:
                    send_command(ser, log_fh, 'STOP', 0.0)
                except Exception:
                    pass
            return 130
        except serial.SerialException as exc:
            print_status(f'Serial error: {exc}')
            return 1
        finally:
            if ser is not None and ser.is_open:
                ser.close()

    print_status(f'Raw log saved to {log_path}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
