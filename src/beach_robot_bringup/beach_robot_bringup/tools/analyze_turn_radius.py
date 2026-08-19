"""Offline analysis of turn_radius_test trajectory CSVs.

Pass one or more `turn_*.csv` files (the rich per-tick logs written by
turn_radius_test). Recomputes the slip-immune headline metrics, builds a
cross-run comparison table, and picks the tightest *clean* radius per direction.

Headline metrics: yaw-rate fidelity from the gyro and
inside-front encoder speed (>= 0.12 = clean). Odom radius is derived (v/w) with a
circle-fit cross-check; treat it as biased by translation slip until anchored
against one tape-measured 360 arc.

Usage:
  python3 analyze_turn_radius.py run1.csv run2.csv ...
  ros2 run beach_robot_bringup turn_radius_analyze run*.csv
"""

import argparse
import csv
import math

CLEAN_INSIDE_FRONT_MPS = 0.12
PIVOT_MIN_WHEEL_MPS = 0.05
MIXER_MIN_TURN_RADIUS = 0.65
MIXER_MAX_W = 0.35


def to_float(v):
    try:
        x = float(v)
        return x if math.isfinite(x) else None
    except (TypeError, ValueError):
        return None


def mean(values):
    vals = [v for v in values if v is not None]
    return sum(vals) / len(vals) if vals else None


def fabs(x):
    """None-safe abs of a CSV cell (blank/non-numeric -> None)."""
    f = to_float(x)
    return abs(f) if f is not None else None


def circle_fit_radius(points):
    if len(points) < 5:
        return None
    n = len(points)
    sx = sy = sxx = syy = sxy = sxz = syz = sz = 0.0
    for x, y in points:
        z = x * x + y * y
        sx += x; sy += y; sxx += x * x; syy += y * y; sxy += x * y
        sxz += x * z; syz += y * z; sz += z
    m = [[sxx, sxy, sx, sxz], [sxy, syy, sy, syz], [sx, sy, float(n), sz]]
    for col in range(3):
        piv = max(range(col, 3), key=lambda r: abs(m[r][col]))
        if abs(m[piv][col]) < 1e-12:
            return None
        m[col], m[piv] = m[piv], m[col]
        pv = m[col][col]
        m[col] = [x / pv for x in m[col]]
        for r in range(3):
            if r != col:
                fr = m[r][col]
                m[r] = [a - fr * b for a, b in zip(m[r], m[col])]
    a, b, c = m[0][3], m[1][3], m[2][3]
    cx, cy = a / 2.0, b / 2.0
    r2 = c + cx * cx + cy * cy
    return math.sqrt(r2) if r2 > 0 else None


def analyze_file(path, settle_deg, mixer_min_radius, mixer_max_w):
    rows = []
    with open(path, newline="") as fh:
        reader = csv.DictReader(fh)
        # Skip the per-run summary file (turn_summary.csv) and anything that is not
        # a trajectory log (no per-tick gyro/odom columns).
        if not reader.fieldnames or "gyro_w" not in reader.fieldnames:
            return None
        for r in reader:
            rows.append(r)
    if not rows:
        return None

    mode = (rows[0].get("mode") or "arc").strip().lower()
    pivot = mode == "pivot"
    cum = [to_float(r.get("cum_yaw")) for r in rows]
    cum = [c for c in cum if c is not None]
    swept_deg = math.degrees(abs(cum[-1])) if cum else 0.0
    # commanded values: take the steady (post-ramp) command
    cmd_w = mean(fabs(r.get("cmd_w")) for r in rows
                 if to_float(r.get("ramp")) and to_float(r.get("ramp")) >= 0.99)
    cmd_v = mean(to_float(r.get("cmd_v")) for r in rows
                 if to_float(r.get("ramp")) and to_float(r.get("ramp")) >= 0.99)
    direction = "left" if (cmd_w and any(
        (to_float(r.get("cmd_w")) or 0) > 0 for r in rows)) else "right"

    settle = math.radians(settle_deg)
    steady = [r for r in rows if (to_float(r.get("cum_yaw")) is not None
              and abs(to_float(r.get("cum_yaw"))) >= settle)]
    if len(steady) < 5:
        steady = rows

    gyro_w = mean(fabs(r.get("gyro_w")) for r in steady)
    odom_v = mean(to_float(r.get("odom_v")) for r in steady)
    yaw_fidelity = (gyro_w / cmd_w) if gyro_w and cmd_w else None
    r_derived = (odom_v / gyro_w) if gyro_w and odom_v and gyro_w > 1e-6 else None
    pts = [(to_float(r.get("odom_x")), to_float(r.get("odom_y"))) for r in steady]
    pts = [(x, y) for x, y in pts if x is not None and y is not None]
    r_fit = circle_fit_radius(pts)

    inside = "fl" if direction == "left" else "fr"
    inside_vals = [abs(to_float(r.get(f"enc_vel_{inside}"))) for r in steady
                   if to_float(r.get(f"enc_vel_{inside}")) is not None]
    inside_min = min(inside_vals) if inside_vals else None
    inside_mean = mean(inside_vals) if inside_vals else None
    # rear (outside helps spot front/rear fighting): mean |enc| front vs rear
    front_enc = mean(abs(to_float(r.get(f"enc_vel_{w}")))
                     for r in steady for w in ("fl", "fr")
                     if to_float(r.get(f"enc_vel_{w}")) is not None)
    rear_enc = mean(abs(to_float(r.get(f"enc_vel_{w}")))
                    for r in steady for w in ("rl", "rr")
                    if to_float(r.get(f"enc_vel_{w}")) is not None)
    # min |enc| across all 4 wheels (pivot gate: every wheel must turn)
    wheel_mins = []
    for w in ("fl", "fr", "rl", "rr"):
        vals = [abs(to_float(r.get(f"enc_vel_{w}"))) for r in steady
                if to_float(r.get(f"enc_vel_{w}")) is not None]
        if vals:
            wheel_mins.append(min(vals))
    min_wheel = min(wheel_mins) if wheel_mins else None
    # center drift: net displacement start->end (pivot: lower = tighter)
    xs = [(to_float(r.get("odom_x")), to_float(r.get("odom_y"))) for r in rows]
    xs = [(x, y) for x, y in xs if x is not None and y is not None]
    drift = math.hypot(xs[-1][0] - xs[0][0], xs[-1][1] - xs[0][1]) if len(xs) >= 2 else None

    r_cmd = (cmd_v / cmd_w) if cmd_v and cmd_w else None
    if pivot:
        clamped = cmd_w is not None and cmd_w > mixer_max_w
        clean = min_wheel is not None and min_wheel >= PIVOT_MIN_WHEEL_MPS
    else:
        clamped = (r_cmd is not None and r_cmd < mixer_min_radius) or \
                  (cmd_w is not None and cmd_w > mixer_max_w)
        clean = inside_min is not None and inside_min >= CLEAN_INSIDE_FRONT_MPS

    return {
        "file": path.split("/")[-1],
        "mode": mode,
        "pivot": pivot,
        "dir": direction,
        "R_cmd": r_cmd,
        "v_cmd": cmd_v,
        "w_cmd": cmd_w,
        "swept": swept_deg,
        "w_gyro": gyro_w,
        "fidelity": yaw_fidelity,
        "R_deriv": r_derived,
        "R_fit": r_fit,
        "inside": inside,
        "in_min": inside_min,
        "in_mean": inside_mean,
        "front_enc": front_enc,
        "rear_enc": rear_enc,
        "min_wheel": min_wheel,
        "drift": drift,
        "clean": clean,
        "clamped": clamped,
    }


def fmt(v, d=3):
    return f"{v:.{d}f}" if isinstance(v, (int, float)) else "-"


def main():
    ap = argparse.ArgumentParser(description="Analyze turn_radius_test trajectory CSVs (arc + pivot).")
    ap.add_argument("csv_paths", nargs="+")
    ap.add_argument("--settle-deg", type=float, default=20.0)
    ap.add_argument("--mixer-min-radius", type=float, default=MIXER_MIN_TURN_RADIUS,
                    help="active mixer min_turn_radius when the runs were recorded (clamp flagging)")
    ap.add_argument("--mixer-max-w", type=float, default=MIXER_MAX_W,
                    help="active mixer max_w when the runs were recorded")
    args = ap.parse_args()

    results = []
    for path in args.csv_paths:
        r = analyze_file(path, args.settle_deg, args.mixer_min_radius, args.mixer_max_w)
        if r:
            results.append(r)
    if not results:
        print("No usable rows in the given CSVs.")
        return

    header = ["file", "mode", "dir", "R_cmd", "w_cmd", "clamp", "w_gyro", "fidelity",
              "R_deriv", "min_wh", "in_min", "drift", "f/r", "clean"]
    table = [header]
    for r in sorted(results, key=lambda x: (x["pivot"], x["dir"], x["R_cmd"] or 0)):
        fr_ratio = (r["front_enc"] / r["rear_enc"]) if r["front_enc"] and r["rear_enc"] else None
        table.append([
            r["file"], r["mode"], r["dir"], fmt(r["R_cmd"], 2), fmt(r["w_cmd"]),
            "Y" if r["clamped"] else "", fmt(r["w_gyro"]), fmt(r["fidelity"], 2),
            fmt(r["R_deriv"], 2), fmt(r["min_wheel"]), fmt(r["in_min"]),
            fmt(r["drift"], 2), fmt(fr_ratio, 2), "OK" if r["clean"] else "BAD",
        ])
    widths = [max(len(str(row[i])) for row in table) for i in range(len(header))]
    for i, row in enumerate(table):
        print("  ".join(str(c).ljust(widths[j]) for j, c in enumerate(row)))
        if i == 0:
            print("  ".join("-" * widths[j] for j in range(len(header))))

    arcs = [r for r in results if not r["pivot"]]
    if arcs:
        print(f"\nTightest CLEAN arc per direction (inside front >= {CLEAN_INSIDE_FRONT_MPS} m/s, "
              "not mixer-clamped):")
        for d in ("left", "right"):
            clean = [r for r in arcs if r["dir"] == d and r["clean"]
                     and not r["clamped"] and r["R_cmd"]]
            if clean:
                best = min(clean, key=lambda x: x["R_cmd"])
                print(f"  {d:5}: R_cmd={best['R_cmd']:.2f} m  fidelity={fmt(best['fidelity'],2)} "
                      f"inside_min={fmt(best['in_min'])}  -> use this as coverage turn_radius")
            elif any(r["dir"] == d for r in arcs):
                print(f"  {d:5}: no clean un-clamped arc in this set "
                      "(loosen R, raise turn_gain_front, or lower mixer min_turn_radius)")

    pivots = [r for r in results if r["pivot"]]
    if pivots:
        print(f"\nPivots (clean = every wheel >= {PIVOT_MIN_WHEEL_MPS} m/s; lower drift = tighter):")
        for r in sorted(pivots, key=lambda x: (x["dir"], x["w_cmd"] or 0)):
            tag = "CLEAN" if r["clean"] else "STALL"
            print(f"  {r['dir']:5} w_cmd={fmt(r['w_cmd'])}: {tag}  min_wheel={fmt(r['min_wheel'])} "
                  f"drift={fmt(r['drift'],2)} m  fidelity={fmt(r['fidelity'],2)}")

    print("\nHints:")
    hinted = False
    for r in results:
        if r["clamped"]:
            lim = (f"w>{args.mixer_max_w}" if r["pivot"]
                   else f"R<{args.mixer_min_radius} or w>{args.mixer_max_w}")
            print(f"- {r['file']}: hit a mixer clamp ({lim}); achieved value is the clamp, not slip. "
                  "Launch with the loosened probe config (or pass matching --mixer-* here).")
            hinted = True
        if not r["pivot"] and (r["R_deriv"] and r["R_cmd"] and not r["clamped"]
                               and r["R_deriv"] > 1.3 * r["R_cmd"]):
            print(f"- {r['file']}: achieved radius {r['R_deriv']:.2f} m >> commanded "
                  f"{r['R_cmd']:.2f} m (robot under-turns); widen R or raise turn_gain_front. "
                  "(fidelity is a diagnostic, not a 1.0 target.)")
            hinted = True
        if not r["clean"] and not r["pivot"]:
            print(f"- {r['file']}: inside front {r['inside'].upper()} stalls "
                  f"(min {fmt(r['in_min'])} < {CLEAN_INSIDE_FRONT_MPS}); arc degenerating to pivot "
                  "-> raise turn_gain_front or widen R.")
            hinted = True
        if not r["clean"] and r["pivot"]:
            print(f"- {r['file']}: a wheel stalls in the pivot (min_wheel {fmt(r['min_wheel'])} < "
                  f"{PIVOT_MIN_WHEEL_MPS}); raise turn_gain_*_in_place / in_place floor.")
            hinted = True
    if not hinted:
        print("- all runs clean and un-clamped; pick the smallest clean R_cmd (arc) "
              "or lowest-drift pivot.")


if __name__ == "__main__":
    main()
