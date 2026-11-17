#!/usr/bin/env python3
"""Drive the rail over the serial port: calibrate, command velocity or position.

usage: drive.py [-p PORT] <command> [args]

  cal                     dis + cal, then wait for the sweep (about 2-2.5 min)
  wait                    wait for a running calibration to finish
  res                     print the calibration result pages (res, res h, res f)
  hold                    send sp 0.0 and print the reply
  dis                     disable (calibration is lost)
  vel <mm/s> [seconds]    stream sp at 30 ms for [seconds] (default 3), then
                          ramp to zero. Positive is toward far, negative home.
  move <mm> <mm/s>        move a relative distance and stop on target
                          (positive toward far). Must already be calibrated.

Each sp request is answered by one "ack <pos_mm> <vel> <acc>". The firmware
watchdog stops the carriage if sp is not repeated within 50 ms, so vel and move
keep streaming. Any request other than cal/dis/res/kd aborts a calibration
with "err cal", so do not run other commands while cal/wait is running.
"""
import argparse
import sys
import time

import serial

# Distance before the target at which the stop is requested, fitted to moves at
# 10, 20 and 30 mm/s: a constant-lag term plus a braking term growing with speed^2.
EARLY_STOP_A = 0.2825
EARLY_STOP_B = 0.01025
MAX_SPEED = 32.0
CAL_TIMEOUT_S = 240.0

s = None


def read_lines(window):
    end = time.time() + window
    buf = b""
    while time.time() < end:
        buf += s.read(256)
    return [l.strip() for l in buf.decode(errors="replace").split("\n") if l.strip()]


def ack_fields(lines):
    for line in reversed(lines):
        if line.startswith("ack "):
            return [float(x) for x in line.split()[1:4]]
    return None


def send(command, window=0.3):
    s.write((command + "\n").encode())
    lines = read_lines(window)
    print(f"{command!r} -> {lines}")
    return lines


def wait_calibration():
    t0 = time.time()
    while time.time() - t0 < CAL_TIMEOUT_S:
        line = s.readline().decode(errors="replace").strip()
        if not line:
            continue
        print(f"{time.time() - t0:6.1f}s {line}", flush=True)
        if line.startswith("ack"):
            print("calibration done")
            return 0
        if line.startswith("err"):
            print("calibration failed")
            return 1
    print("timed out waiting for calibration")
    return 1


def cmd_cal(_):
    send("dis")
    lines = send("cal")
    if not any(l == "cal" for l in lines):
        print("firmware did not start calibration")
        return 1
    return wait_calibration()


def cmd_wait(_):
    return wait_calibration()


def cmd_res(_):
    for page in ("res", "res h", "res f"):
        send(page)
    return 0


def cmd_hold(_):
    return 0 if ack_fields(send("sp 0.0")) else 1


def cmd_dis(_):
    send("dis")
    return 0


def ramp_down():
    # Each request gets exactly one reply, so keep asking while the carriage settles.
    lines = []
    end = time.time() + 2.5
    while time.time() < end:
        s.write(b"sp 0.0\n")
        lines += read_lines(0.1)
    return ack_fields(lines), lines


def cmd_vel(args):
    if abs(args.speed) > MAX_SPEED:
        print(f"speed limited to +-{MAX_SPEED} mm/s")
        return 1
    end = time.time() + args.seconds
    while time.time() < end:
        s.write(f"sp {args.speed:.1f}\n".encode())
        for line in read_lines(0.03):
            print(line)
            if line.startswith("err"):
                return 1
    final, lines = ramp_down()
    print(f"final {final}" if final else f"no final ack: {lines[-3:]}")
    return 0 if final else 1


def cmd_move(args):
    distance = args.distance
    speed = abs(args.speed)
    if speed > MAX_SPEED or speed == 0.0:
        print(f"speed must be in (0, {MAX_SPEED}] mm/s")
        return 1
    sign = 1.0 if distance >= 0 else -1.0
    command = sign * speed
    early_stop_mm = EARLY_STOP_A * speed + EARLY_STOP_B * speed ** 2

    s.write(b"sp 0.0\n")
    lines = read_lines(0.3)
    fields = ack_fields(lines)
    if fields is None:
        print(f"ABORT: not active and calibrated ({lines[-1] if lines else 'no reply'})")
        return 1

    start = fields[0]
    target = start + distance
    print(f"start {start:.1f} mm -> target {target:.1f} mm at {command:+.1f} mm/s "
          f"(stop begins {early_stop_mm:.1f} mm early)", flush=True)

    begin = time.time()
    limit = abs(distance) / speed + 15.0
    reason = None
    while time.time() - begin < limit and reason is None:
        s.write(f"sp {command:.1f}\n".encode())
        for line in read_lines(0.03):
            if line.startswith("err"):
                reason = line
            elif line.startswith("ack "):
                position = float(line.split()[1])
                if (target - position) * sign <= early_stop_mm:
                    reason = "reached stopping point"
                    break

    final, lines = ramp_down()
    print(f"stopped because: {reason}", flush=True)
    if final is None:
        print(f"no final ack; last lines: {lines[-3:]}")
        return 1
    print(f"final reported position {final[0]:.1f} mm, velocity {final[1]:.1f} mm/s")
    print(f"moved {final[0] - start:+.1f} mm (asked for {distance:+.1f}); "
          f"error vs target {final[0] - target:+.1f} mm")
    return 0


def main():
    global s
    parser = argparse.ArgumentParser(description="Rail drive helper")
    parser.add_argument("-p", "--port", default="/dev/ttyACM0")
    sub = parser.add_subparsers(dest="command", required=True)
    for name, func in (("cal", cmd_cal), ("wait", cmd_wait), ("res", cmd_res),
                       ("hold", cmd_hold), ("dis", cmd_dis)):
        sub.add_parser(name).set_defaults(func=func)
    vel = sub.add_parser("vel")
    vel.add_argument("speed", type=float)
    vel.add_argument("seconds", type=float, nargs="?", default=3.0)
    vel.set_defaults(func=cmd_vel)
    move = sub.add_parser("move")
    move.add_argument("distance", type=float)
    move.add_argument("speed", type=float)
    move.set_defaults(func=cmd_move)
    args = parser.parse_args()

    s = serial.Serial(args.port, 115200, timeout=0.005)
    time.sleep(0.2)
    s.reset_input_buffer()
    try:
        return args.func(args)
    finally:
        s.close()


if __name__ == "__main__":
    sys.exit(main())
