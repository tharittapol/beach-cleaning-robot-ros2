# Sand Field-Tuning Guide

Bench/indoor tuning does **not** transfer to a sandy beach. Sand is soft, slips, and has
high rolling resistance, so the wheels move less than the encoders think (odometry drift),
stall more easily (need higher motor floors), and turn wider than commanded. This guide
lists the parameters that matter, the **one test** that isolates each, and the **single log
to send back** so the new value can be computed instead of guessed.

> Workflow per parameter: run the named test → it writes a CSV → send me that CSV → I
> return the new value → you edit the file and `colcon build`.

## How to send a log

All test tools write CSVs under `~/beach_robot_logs/...`. After a test:

```bash
# zip the run and copy it off the Jetson
cd ~/beach_robot_logs
tar czf /tmp/sand_run.tgz <the run folder or csv>
# then scp /tmp/sand_run.tgz to the laptop and attach it here
```

For coverage runs, record the bag (command in `CLAUDE.md` → "Coverage bag recording") and
run `coverage_bag_report` on it; send the generated `*.csv` files.

---

## Before anything: effective rolling radius on sand

Everything downstream (distance, speed, turn radius) depends on the **effective rolling
radius** of the wheels. On sand the wheels sink, so the real radius shrinks from the
bench values:

- `TRACK_R_EFF_M = 0.115` and `ROUND_WHEEL_R_M = 0.1601` in `firmware/include/encoder.h`.

**Field check:** mark a wheel, push the robot in a straight line over a measured 5 m of
sand, count wheel revolutions. `radius = distance / (2π × revolutions)`. If it differs
from the bench value by more than ~5 %, tell me the measured distance + revolution count
per wheel type (track vs round) and I'll give corrected radii. Do this **first** — it
shifts `linear_scale` and `turn_radius` below.

---

## Priority order

| # | Parameter (file) | Current | Sand symptom | Test → send |
|---|---|---|---|---|
| 1 | `linear_scale` (`beach_cleaning_bringup.launch.py:64`) | 1.45 | drives short/long vs commanded distance | `drive_straight_odom` + tape → straight-run CSV |
| 2 | `ACTIVE_U_FLOOR`, `SPIN_U_FLOOR`, `TURN_U_FLOOR_*` (`firmware/src/main_esp.cpp:31-37`) | tracks 0.22/0.40, rounds 0.06/0.14 | wheels stall / won't start in sand | `wheel_response_test` → wheel-response CSV |
| 3 | `angular_scale` + spin balance (`:65`) | 1.0 | heading drifts, under-rotates | `wheel_response_test` spin → spin CSV |
| 4 | `max_accel_mps2` (`mixer.yaml:56`) | 0.6 | wheelspin / dig-in on start | wheel-response CSV (cmd vs enc) |
| 5 | `turn_radius` + `regulated_linear_scaling_min_radius` (`nav2_params_keepout.yaml:50`) | 0.30 | arc turns slip wide | coverage bag → `turn_tracking.csv` |
| 6 | `desired_linear_vel`, `min_approach_linear_velocity` (`:35,36`) | 0.17 / 0.05 | stalls on approach / too fast | coverage bag → `coverage_summary.csv` |
| 7 | `xy_goal_tolerance` (`:29`) | 0.35 | waypoints reached early/late | coverage bag → `lane_tracking.csv` |

---

## 1 — `linear_scale` (odometry distance)  🔴

**Symptom:** the robot stops before/after the real target; lanes look short or long.

**Test** (drive a straight 10 m on the actual sand, then tape-measure the ground distance):

```bash
source install/setup.bash
ros2 run beach_robot_bringup drive_straight_odom \
  --odom-topic /odometry/local --distance-m 10.0 --speed-mps 0.12
# tool stops when ODOM thinks it went 10 m. Now tape-measure the REAL ground distance.
```

**Send me:** the printed summary CSV path **and** the tape-measured real distance.
**I compute:** `new_linear_scale = linear_scale × (real_distance / 10.0)`.

---

## 2 — Motor floors (stall) 🔴

**Symptom:** a wheel buzzes/stalls instead of turning; the robot won't start moving or
won't pivot. Worst on the front tracks in soft sand.

**Test** (run the standard per-wheel/maneuver step test on sand, wheels loaded):

```bash
ros2 run beach_robot_bringup wheel_response_test \
  --label sand --tests forward,backward,spin_left,spin_right,curve_left,curve_right
# writes CSVs to ~/beach_robot_logs/wheel_response/
ros2 run beach_robot_bringup wheel_response_analyze \
  ~/beach_robot_logs/wheel_response/<the_forward_csv>.csv
```

**Send me:** the whole `~/beach_robot_logs/wheel_response/` run.
**I compute:** raised `ACTIVE_U_FLOOR` / `SPIN_U_FLOOR` / `TURN_U_FLOOR_*` per wheel
(typically +30…100 % on sand) so each wheel breaks static friction and tracks its target.
These are firmware constants → reflash, **or** push live via the
`{"active_u_floor":[…]}` / `{"spin_u_floor":[…]}` JSON commands the ESP32 already accepts.

---

## 3 — `angular_scale` + spin balance 🔴

**Symptom:** heading drifts during straight lanes, or in-place/turning rotation is short of
commanded; front vs rear wheels fight in a pivot.

**Test:** the spin maneuvers from the wheel-response run above, then the spin analyzer:

```bash
ros2 run beach_robot_bringup spin_tune_analyze \
  ~/beach_robot_logs/wheel_response/<the_spin_left_csv>.csv
```

**Send me:** the spin_left / spin_right CSVs (or the analyzer's printout).
**I compute:** front/rear spin floor balance and a corrected `angular_scale`. (Confirm #1
first — a wrong linear radius pollutes the yaw estimate.)

---

## 4 — `max_accel_mps2` 🟡

**Symptom:** wheels spin up and dig into sand on each start; jerky lane starts.

**Source:** `max_accel_mps2: 0.6` in `src/beach_wheel_mixer/config/mixer.yaml:56`.
Read it from the wheel-response CSV (commanded vs encoder velocity at each step start): if
the encoder lags far behind command at start = slip. **Send** the `forward` CSV.
**Typical fix:** lower to ~0.4 m/s² for traction; no reflash (mixer YAML + rebuild mixer).

---

## 5 — `turn_radius` / arc tracking 🟠

**Symptom:** the S-curve between lanes slips wide; the robot overshoots the next lane's y.

**Test:** a short coverage run on sand (see CLAUDE.md Step 2, small area), record the bag,
then:

```bash
ros2 run beach_robot_bringup coverage_bag_report <bag_path> --out ~/beach_robot_logs/reports/sand1
```

**Send me:** `turn_tracking.csv` (planned vs actual y after each turn).
**I compute:** a larger `turn_radius` (and matching `regulated_linear_scaling_min_radius`
in `nav2_params_keepout.yaml`) so commanded arcs land on the lane. Remember
`lane_spacing ≥ 2 × turn_radius` or arcs degrade to skid turns
(`auto_widen_lanes_for_turn:=true` forces this).

---

## 6 — Coverage speeds 🟠

**Symptom:** robot crawls/stalls on final approach (`min_approach_linear_velocity` too low
for sand rolling resistance), or `desired_linear_vel` feels too fast/slow.

**Send me:** `coverage_summary.csv` + `cmd_vel.csv` from the same coverage bag report.
**I compute:** adjusted `desired_linear_vel` / `min_approach_linear_velocity` in
`nav2_params_keepout.yaml`. (Keep these ≤ teleop's `max_linear=0.17` for consistency.)

---

## 7 — `xy_goal_tolerance` 🟡

**Symptom:** waypoints declared "reached" with the robot visibly short, or it loops trying
to hit a too-tight tolerance under odom drift.

**Send me:** `lane_tracking.csv` (y error per lane).
**I compute:** a tolerance consistent with the measured sand odom drift.

---

## First 30 minutes on sand — ordered checklist

Run in this order; each step's result feeds the next. Always start with a
`preflight_check` and have the joystick E-STOP (Y) ready.

```bash
ros2 run beach_robot_bringup preflight_check        # all topics alive?
```

1. **Rolling radius** check (push test) — fix `TRACK_R_EFF_M` / `ROUND_WHEEL_R_M` if off.
2. **`linear_scale`** — `drive_straight_odom` 10 m + tape. → send straight-run CSV.
3. **Motor floors** — `wheel_response_test --label sand …`. → send the run.
4. **`angular_scale` / spin** — `spin_tune_analyze` on the spin CSVs. → send spin CSVs.
5. **One slow coverage lane** — small area, record bag, `coverage_bag_report`. → send the
   `*.csv` set (turn/lane/summary) for radius, speed, and tolerance tuning.

After I return values, edit the listed file, `colcon build --symlink-install
--packages-select <pkg>`, `source install/setup.bash`, and re-test the single changed item.

---

## Field notes for the two new behaviours (this deployment)

**Obstacle stop (auto mode):** the coverage node now stops + beeps when the ZED sees an
object within **2.0 m** in the forward cone, and resumes the lane after **3 s** clear.
Sand-relevant knobs (launch args on `beach_cleaning_bringup.launch.py`):
`obstacle_stop_distance`, `obstacle_cone_half_width`, `obstacle_clear_time_sec`, and the
node param `obstacle_stop.min_z` (default 0.12 m). If the robot stops for flat sand (ground
false-positives when it pitches on a dune), **raise `obstacle_stop.min_z`** (e.g. 0.15–0.20).
If it ignores low obstacles, lower it (but not below ~0.08, the cloud filter floor). Test
standalone first: `ros2 run beach_robot_coverage_nav2 obstacle_detector` and walk in/out of
the cone while watching the log.

> Run the coverage area with `area_yaw:=0` and `area_origin_x/y:=0` (map-X along the shore),
> as every documented test does. The lane re-send on resume — like the existing turn logic —
> treats odometry x/y as local-area coordinates, which only holds when origin and yaw are 0.

**Vibration motor (manual mode):** X toggles the sieve; it now also force-stops on E-STOP
(Y). Verify before the run:

```bash
ros2 topic echo /vibration_enable    # press X in manual → alternates true/false
                                     # hit E-STOP while ON → expect a single false
```
