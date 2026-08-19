# Engineering Notes — Beach-Cleaning Robot

The working technical reference for this robot: every tuned parameter together with the test that
produced it, the full topic map, the field test procedures, and the known failure modes.

> The identical content also lives in [`CLAUDE.md`](../CLAUDE.md) at the repository root, where it
> doubles as project instructions for the Claude Code CLI.

## Project Overview

Autonomous beach-cleaning robot running **ROS2** on a **Jetson Nano Super**.
The robot has 4 wheels (front: track/caterpillar tyres, narrow; rear: round tyres, wider), a scoop under the chassis, and a vibration-motor sieve to separate trash from sand.

**Development workflow**: edit code on laptop → `git push` → SSH into Jetson → `git pull` → `colcon build` → test.

---

## Repository Layout

```
beach_robot_ws/
├── src/                        # ROS2 packages (build with colcon)
│   ├── beach_robot_bringup/        # Hardware orchestrator + analysis tools
│   ├── beach_robot_coverage_nav2/  # Coverage planner + Nav2 bringup
│   ├── beach_robot_localization/   # Wheel odom + EKF + GNSS fusion
│   ├── beach_wheel_mixer/          # cmd_vel → wheel_cmd (C++)
│   ├── beach_robot_esp32_bridge/   # Serial bridge to ESP32
│   ├── beach_robot_teleop/         # Joystick teleop
│   ├── beach_robot_gnss/           # UM982 GNSS driver + RTK/NTRIP
│   ├── beach_robot_sim/            # Gazebo simulation launcher
│   ├── beach_robot_description/    # URDF / robot description
│   ├── beach_robot_interfaces/     # Custom messages / actions
│   ├── zed_nav2_cloud_filter/      # ZED depth → obstacle layer (C++)
│   └── zed-ros2-wrapper/           # External submodule (Stereolabs)
└── firmware/                   # Version-tracked ESP32 source (reference only)
    └── src/main_esp.cpp            # Master ESP32 firmware (~1160 lines)
```

> `firmware/` is **not built here**. Actual flashing uses PlatformIO in a separate repo.

---

## Hardware

| Component | Detail |
|-----------|--------|
| Compute | Jetson Nano Super |
| MCU | 2× ESP32 (motor/sensor control via serial) |
| GNSS | UM982 (RTK-capable) |
| IMU | BNO055 (via ESP32) |
| Depth cam + IMU | ZED Mini |
| Ultrasonic | 3× HC-SR04 (left/middle/right) |
| Joystick | 2.4 GHz receiver on `/dev/input/js_joy` |
| ESP32 serial | `/dev/ttyESP32` @ 230400 baud |

### Wheel geometry (important — asymmetric robot)
| Wheel | Type | Track width | Encoder scale |
|-------|------|------------|---------------|
| FL / FR (front) | Caterpillar | 0.734 m | 1.0 / 1.0 |
| RL / RR (rear) | Round | 1.179 m | 0.64 / 0.64 |

The rear encoder scale of **0.64** compensates for chain drive reduction.

---

## Tuned Parameters (do not change without re-testing)

| Parameter | Value | Tuned in |
|-----------|-------|---------|
| `linear_scale` (**indoor tile**) | **1.625** | **Tile 2026-06-19** (4.87 m tape @ odom 5.0 m, converged from 1.19; sand 1.19 under-reported ~40% on tile → "lost pose" in coverage. Now the default in `localization_full_test.launch.py` + `beach_cleaning_bringup.launch.py`) |
| `linear_scale` (**sand**) | 1.19 | **Sand field tune 2026-06-10** (10 m odom vs 8.245 m tape ≈ 21% slip; was 1.45 indoor) |
| `angular_scale` | 1.0 (leave) | **Tile circle test 2026-06-19** — do NOT tune. wheel-odom yaw error is slip-driven & L/R-asymmetric (would need 1.055 left but 1.166 right; right circle wheel-odom path missed closure by 1.09 m while fusion_bno closed to 0.25 m). No scalar fixes it; EKF down-weights wheel yaw under BNO. BNO is the heading authority |
| ESP32 PID Kp | [0.15, 0.18, 0.04, 0.03] | On-air test 2026-03-21 |
| ESP32 PID Ki | [0.08, 0.08, 0.05, 0.05] | **Sand field tune 2026-06-09** (was [0.01,0.01,0,0]; raised for hill/sustained-load climb) |
| ESP32 `ACTIVE_U_FLOOR` | [0.12, 0.12, 0.06, 0.06] | **Sand field tune 2026-06-09** (front was 0.22; lowered to kill low-speed stick-slip stutter) |
| `turn_gain_front` (mixer, **sand**) | 1.6 | **Sand 2026-06-10** (2.45 collapsed inside front→pivot/stall; 1.3→~2.0 m turn; **1.6→~1.75 m clean turn**, inside front ≥0.12) |
| Coverage `turn_radius` (**sand**) | 1.8 m | **Sand 2026-06-10** (rear round wheels slip in turns → robot under-rotates; tightest clean radius ~1.08 m only at v=0.5; 1.8 m works at slow safe speed) |
| `turn_gain_front` (mixer, **indoor tile**) | 1.5 | **Tile 2026-06-17** (`mixer_turn06.yaml`; gives R=0.6 m at v=0.2: L 0.611 / R 0.599. With min_turn_radius 0.50, max_w 0.45. Sand `mixer.yaml` untouched) |
| Coverage `turn_radius` (**indoor tile**) | 0.6 m | **Tile 2026-06-17** (lane_spacing 1.2 = 2×R; rolling-arc floor on tile, measured with `turn_radius_test`. Tighter only via skid (rear scrub) or pivot) |
| Wheel V_MAX | [1.25, 1.10, 9.70, 8.60] m/s | Hardware characterisation |

---

## Key Topics

| Topic | Type | Producer |
|-------|------|---------|
| `/cmd_vel` | Twist | Nav2 controller / teleop |
| `/wheel_cmd` | Float32MultiArray | beach_wheel_mixer |
| `/enc_vel` | Float32MultiArray | esp32_bridge |
| `/wheel/odom` | Odometry | wheel_odometry_node |
| `/odometry/fusion_bno` | Odometry | EKF (wheel + BNO055) — **the topic Nav2 uses** (`odom_topic` in `nav2_params_*.yaml`) and the publisher of TF `odom→base_link` |
| `/odometry/gps` | Odometry | navsat_transform (only if use_gnss=true) |
| `/imu/data` | Imu | BNO055 via ESP32 |
| `/zed/filtered_cloud` | PointCloud2 | zed_nav2_cloud_filter |
| `/ultrasonic/{left,middle,right}` | Range | ESP32 bridge |
| `/gps/fix` | NavSatFix | UM982 bridge |
| `/coverage/path` | Path | coverage_follow_waypoints (preview) |

---

## Package Details

### `beach_robot_coverage_nav2`
Coverage planner + full Nav2 bringup.

**Main files:**
- `launch/beach_cleaning_bringup.launch.py` — master launch
- `beach_robot_coverage_nav2/coverage_follow_waypoints.py` — generates waypoints, sends to Nav2 `follow_waypoints` action
- `config/nav2_params_keepout.yaml` — Nav2 tuning
- `config/keepout_mask.yaml` + `keepout_mask.pgm` — boundary / keepout map

**Coverage node key parameters:**

| Parameter | Default | Notes |
|-----------|---------|-------|
| `pattern` | `boustrophedon` | also: `spiral` |
| `area.width / height` | 30 / 10 m | long axis parallel to beach |
| `lane_spacing` | 0.60 m | **in-pass** spacing; keep ≥ 2×turn_radius for arc turns |
| `turn_radius` | 0.30 m | min arc radius; warn if < lane_spacing/2 |
| `auto_widen_lanes_for_turn` | false | auto-expand lane_spacing to 2×turn_radius |
| `num_passes` | 1 | interleaved passes for 100% coverage (launch default **3**) |
| `deadhead_style` | `outside` | between-pass reposition: `outside` loop, `direct` (straight + in-place rotates), or `rounded` (R=turn_radius arcs + straight, stays inside; needs `coverage_path_mode:=teardrop`). `direct`/`rounded` only take effect when `coverage_path_mode != same_direction_loops` |
| `deadhead_clearance` | 0.9 m | how far outside the area the loop runs (≥ turn_radius; unused by `direct`/`rounded`) |
| `coverage_path_mode` | `same_direction_loops` | between-lane geometry: `same_direction_loops` (every transition loops outside), `teardrop` (in-pass arc turns + per-`deadhead_style` between-pass), `multipass_boustrophedon` |
| `boundary_margin` | 0.30 m | shrinks effective area |
| `waypoint_step` | 0.30 m (launch default; node default 0.50) | dense waypoints along each lane. **Tile 2026-06-19: dropped 0.50→0.30** so Nav2 NavfnPlanner can't diagonal-offset the `/plan` off the lane between sparse poses (saw up to 0.3 m parallel offset) |
| `turn_style` | `arc` | also: `corner` |
| `autostart` | true | set false to preview path only |
| `start_delay_sec` | 15.0 s | wait for Nav2 to stabilise |

**Multipass coverage (sand 2026-06-10):** on sand the clean turn radius is **1.8 m** (rear
round wheels slip in turns → robot under-rotates; see Tuned Parameters). So **lock
`turn_radius:=1.8` and `lane_spacing:=3.6` (= 2×turn_radius)** — these keep the in-pass arc
turns feasible and should not change. `num_passes` is then the **coverage-density knob**: fine
lanes are laid at `lane_spacing/num_passes`, interleaved by one tool width.
- **`num_passes:=6` → fine spacing 0.6 m = tool width → 100% coverage** (the full-coverage value).
- Fewer passes trade coverage for fewer deadheads/faster runs: **coverage ≈ (num_passes / 6) × 100%**
  (P=5→83%, P=4→67%, P=3→50%). Gaps are acceptable for quick tests; **`turn_radius` stays 1.8**.

Within a pass: arc turns (r = lane_spacing/2 = 1.8). Between passes: a deadhead that loops
**outside** the work area (needs no keepout boundary — see `use_keepout` below). See
`docs/sand_tuning_guide.md`.

**Keepout:** `use_keepout:=false` (default) runs without the keepout boundary
(`nav2_params_nokeepout.yaml`, no mask servers) so the outside-loop deadheads can plan.
`use_keepout:=true` restores `nav2_params_keepout.yaml` + the mask servers.

**Nav2 controller (RegulatedPurePursuit):**

| Parameter | Value |
|-----------|-------|
| `desired_linear_vel` | 0.25 m/s (sand; nokeepout) — keeps turns ≥0.18 so inside front track drives |
| `min_approach_linear_velocity` | 0.20 m/s (sand) — don't crawl into waypoints/turns |
| `lookahead_dist` | 0.40 m (nokeepout) |
| `rotate_to_heading_angular_vel` | 0.6 rad/s |
| `controller_frequency` | 20 Hz |
| `stop_on_failure` | false (waypoint follower) |

**Nav2 costmap sources:**
- Global: keepout mask (static layer) + inflation 0.6 m
- Local 12×12 m rolling: ZED point cloud (`/zed/filtered_cloud`) + ultrasonics + inflation

### `beach_robot_localization`
- `localization_full_test.launch.py` — full stack (ESP32 + EKF + optional GNSS)
- `localization_imu_compare.launch.py` — EKF with static TFs; used by coverage bringup
- `wheel_odometry_node.py` — integrates per-wheel velocities with geometry; publishes `/wheel/odom`
- EKF config in `config/ekf_*.yaml`; `/odometry/fusion_bno` is the Nav2-facing topic
  (`/odometry/local` is only produced by the older `localization.launch.py`, not by the coverage stack)

### `beach_wheel_mixer` (C++)
Converts `/cmd_vel` Twist → `/wheel_cmd` Float32MultiArray (m/s for 4 wheels).
Config: `config/mixer.yaml`. Skid-steer kinematics, accounting for asymmetric track widths.

### `beach_robot_esp32_bridge`
- Serial JSON protocol to/from ESP32 @ 230400 baud
- Encoder filter: drops sample if any wheel > 3.0 m/s or step > 1.0 m/s
- Wheel command send rate: 30 Hz (configurable via `wheel_cmd_send_rate_hz`)
- Stale cmd timeout: 0.5 s → sends zeros

---

## Coverage Test Procedure

### Prerequisites
1. Robot connected: ESP32 on `/dev/ttyESP32`, ZED USB, joystick USB
2. Workspace built: `colcon build --symlink-install`
3. Environment sourced: `source install/setup.bash`

### Step 1 — Path preview only (no movement), sand 5×20, full coverage (6-pass)
```bash
ros2 launch beach_robot_coverage_nav2 beach_cleaning_bringup.launch.py \
  start_coverage:=false use_keepout:=false num_passes:=6 \
  coverage_pattern:=boustrophedon \
  area_origin_x:=0.0 area_origin_y:=0.0 \
  area_width:=20.0 area_height:=5.0 area_yaw:=0.0 \
  lane_spacing:=3.60 tool_width:=0.60 turn_radius:=1.80 deadhead_clearance:=1.80 \
  boundary_margin:=0.30 angular_scale:=1.0
```
Long axis (20 m) carries the lanes → few long straight lanes, few turns. Open RViz2 and
visualise `/coverage/path_viz` (type: Path, frame: map, republish 3 sec, QoS: volatile).
Verify long 20 m lanes, arc turns (r=1.8) within a pass, loops outside the rectangle between
passes. The node log prints `passes=6 lanes=N`. Needs **~24×9 m clear sand** (lanes 20 m +
deadhead loops 1.8 m past each end).

**Fewer passes / faster:** lower `num_passes` only and **keep `lane_spacing:=3.60
turn_radius:=1.80`** — coverage ≈ (num_passes/6)×100% (gaps below 6, but arc turns stay valid).

### Step 2 — Autonomous run (sand 5×20, 6-pass)
```bash
ros2 launch beach_robot_coverage_nav2 beach_cleaning_bringup.launch.py \
  start_coverage:=true use_keepout:=false num_passes:=6 \
  coverage_pattern:=boustrophedon \
  area_origin_x:=0.0 area_origin_y:=0.0 \
  area_width:=20.0 area_height:=5.0 area_yaw:=0.0 \
  lane_spacing:=3.60 tool_width:=0.60 turn_radius:=1.80 deadhead_clearance:=1.80 \
  boundary_margin:=0.30 angular_scale:=1.0 use_gnss:=true
```
Keep `area_yaw:=0` and `area_origin_*:=0` (map-X along the shore) — lane/deadhead geometry
assumes the area frame is axis-aligned with map. `use_gnss:=true` so `/gps/fix` +
`/odometry/gps` are published for the bag (drop it if RTK/NTRIP is not set up). Lower
`num_passes` for partial coverage (keep lane_spacing/turn_radius).

### Flags when ZED is not available
```bash
  use_zed:=false
```
This removes ZED from the costmap — ultrasonics only for obstacle detection.

### Coverage bag recording
```bash
STAMP=$(date +%Y%m%d_%H%M%S)
ros2 bag record --include-hidden-topics \
  -o ~/beach_robot_logs/coverage/sand_run_${STAMP} \
  /cmd_vel /wheel_cmd /enc_vel \
  /wheel/odom /odometry/fusion_bno /odometry/local \
  /odometry/gps /gps/fix \
  /imu/data \
  /coverage/path \
  /plan /local_plan \
  /follow_waypoints/_action/feedback \
  /navigate_through_poses/_action/feedback \
  /local_costmap/costmap /global_costmap/costmap \
  /tf /tf_static
```
`/odometry/gps` + `/gps/fix` need the run launched with `use_gnss:=true`. Add
`/ultrasonic/{left,middle,right}` to capture obstacle-stop events; `/zed/filtered_cloud` is
large — record it only when debugging the ZED obstacle layer.

---

## Build & Deploy (on Jetson via SSH)

```bash
cd ~/beach_robot_ws
git pull
colcon build --symlink-install --packages-select <pkg>
source install/setup.bash
```

Full rebuild (slow):
```bash
colcon build --symlink-install
```

---

## Common Failure Modes

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| Nav2 not active after launch | TF `odom→base_link` missing | Check ESP32 bridge connected, EKF publishing |
| Coverage node exits immediately | `follow_waypoints` action server not ready | Increase `start_delay_sec` |
| Robot doesn't turn (arc turns fail) | `lane_spacing < 2×turn_radius` | Add `auto_widen_lanes_for_turn:=true` or increase `lane_spacing` |
| ZED costmap errors | ZED not connected | Pass `use_zed:=false` |
| `/odometry/fusion_bno` missing | Localization not launched | Verify `use_robot_stack:=true` (default) |
| Waypoints skipped silently | `stop_on_failure: false` in nav2_params | Normal; Nav2 continues to next waypoint |
| Encoder spike drops | Wheel velocity > 3 m/s or > 1 m/s step | Expected filter behaviour; increase thresholds if needed |

---

## Analysis Tools (in `beach_robot_bringup`)

| Tool | Purpose |
|------|---------|
| `preflight_check.py` | Checks all topics alive before running |
| `localization_pose_report.py` | Post-process bag → pose accuracy CSV/SVG |
| `drive_straight_odom.py` | Commanded straight-drive + odom validation |
| `analyze_spin_tune.py` | Angular velocity tuning analysis |
| `wheel_response_test.py` | Step-response per wheel |
| `turn_radius_test.py` | Drive one arc → measure achieved turn radius / yaw-rate fidelity |
| `analyze_turn_radius.py` | Cross-run compare arcs → tightest *clean* radius per direction |
| `coverage_bag_report.py` | Post-process coverage bag → CSV report tables |

### Turn-radius tuning (find tightest clean arc — e.g. indoor tile)

Drivetrain turn params (`turn_gain_front`, coverage `turn_radius`) are surface-specific.
On sand the rear round wheels slip → under-rotation (radius 1.8 m). On high-friction tile the
failure mode flips to **inside-front stall / front-rear fighting**, so `turn_gain_front` and
mixer `min_turn_radius` become the knobs (not coverage `turn_radius`). Workflow:

```bash
# Step 0 — bring up the drivetrain WITHOUT Nav2 (mixer + esp32_bridge + EKF + /imu/data +
# /odometry/local). Keep esp32_debug_enabled false while driving; keep joystick E-stop in hand.
ros2 launch beach_robot_localization localization_full_test.launch.py

# Step 1 — sweep radii, both directions; each run appends a row to turn_summary.csv.
ros2 run beach_robot_bringup turn_radius_test --radius 1.2 --speed 0.25 --direction left  --label tile_R12
ros2 run beach_robot_bringup turn_radius_test --radius 1.0 --speed 0.25 --direction right --label tile_R10
# Anchor odom-radius bias once with a full circle + tape-measure the diameter:
ros2 run beach_robot_bringup turn_radius_test --radius 1.0 --speed 0.25 --sweep-deg 360 --label tile_360

# Step 2 — compare runs (run on laptop on the returned CSVs — no robot/colcon needed):
python3 src/beach_robot_bringup/beach_robot_bringup/tools/analyze_turn_radius.py \
  ~/beach_robot_logs/turn_tune/turn_*.csv
```
The tool reads `/odometry/fusion_bno` (EKF wheel+BNO) by default — that is what
`localization_full_test` publishes — and it is also the topic Nav2 consumes in the coverage bringup.
Yaw comes from the BNO gyro either way.
**Decision metrics:** (1) **inside-front encoder speed** (FL for left turns, FR for right; **clean ≥
0.12 m/s** — below = pivot-degenerate/stall) and (2) **R_derived** (`R = v̄/w̄`, achieved radius) vs
commanded. Pick the **smallest R whose inside-front stays ≥ 0.12**. Both use the gyro (`/imu/data`)
and `/enc_vel`, so wheel/translation slip can't corrupt them. *yaw fidelity* (`w_gyro/w_cmd`) is a
**diagnostic, not a 1.0 target** — it shifts with `turn_gain_front` (gain > 1 makes w_gyro > w_cmd),
so don't chase fidelity = 1. Circle-fit radius is a cross-check only (odom x/y carries translation
slip); anchor it once against the **taped 360° circle**. Arcs go through `/cmd_vel`, so real mixer
clamps (`min_turn_radius 0.65`, `max_w/max_v 0.35`) are included and the analyzer **flags a clamp**
so it is not mistaken for slip. First run: eyeball `odom_v` in the CSV (≈ commanded speed; if ~0 the
EKF isn't filling twist → R_derived is wrong) and confirm a left turn gives positive `gyro_w` and
rising `cum_yaw`.

#### Pushing toward a tight target (≤ 0.65 m, e.g. the 0.30 m default)

**Geometry reality check:** R is measured to robot centre. Half-track is **0.367 m front / 0.59 m
rear**, so any R below those means the inside wheel(s) must *reverse* → it is a heavy-skid near-pivot,
not a clean rolling arc (the rear round wheels scrub hard). 0.30 m is below both. Two ways to get it,
both testable with the same tool:

**(A) Rolling-arc probe** — needs the clamps loosened. Use the **non-destructive probe config**
(`beach_wheel_mixer/config/mixer_probe_tight.yaml`: `min_turn_radius 0.30`, `max_w 0.85`; the sand
`mixer.yaml` is untouched). At R=0.30, v=0.25 needs w=0.83 rad/s (hence max_w 0.85); keep v low or the
rear scrubs more. Tell the tool the active clamps so it does not mis-flag:
```bash
ros2 launch beach_robot_localization localization_full_test.launch.py \
  mixer_params_file:=$(ros2 pkg prefix beach_wheel_mixer)/share/beach_wheel_mixer/config/mixer_probe_tight.yaml
# Sweep IN from a known-good radius; stop where inside-front drops < 0.12:
for R in 0.60 0.50 0.40 0.30; do
  ros2 run beach_robot_bringup turn_radius_test --radius $R --speed 0.25 --direction left \
    --mixer-min-radius 0.30 --mixer-max-w 0.85 --label tileA_R${R/./}
done
```

**(B) Corner / in-place pivot** — likely more realistic for this wide-rear robot than a 0.30 m rolling
arc. `turn_radius_test` has a **pivot mode** (`--pivot --omega <rad/s>`, v=0); it commands pure yaw
through the mixer in-place path and reports **min |enc| across all 4 wheels** (clean ≥ 0.05 — every
wheel must turn; inside front reverses so it is *not* the gate) and **center drift** (lower = tighter).
```bash
ros2 run beach_robot_bringup turn_radius_test --pivot --omega 0.6 --direction left  --label tileB_piv_L
ros2 run beach_robot_bringup turn_radius_test --pivot --omega 0.6 --direction right --label tileB_piv_R
```
If pivots are clean on tile, coverage can use `turn_style:=corner` (pivot at lane ends) instead of a
tight rolling `turn_radius`. The analyzer reports arcs and pivots in separate sections.

#### Result — indoor tile, 2026-06-17

Rolling-arc radius **floors at ~0.5–0.6 m** on tile (commanding tighter just under-rotates: the rear
round wheels free-spin/scrub and cap yaw at ~0.7 rad/s). Allowing inside-wheel reversal
(`min_curve_inner_mps −0.30`, skid-turn) reached ~0.42 m at v=0.25 but with heavy rear scrub; 0.30 m
is **geometrically a pivot** (R < half-track 0.367 front / 0.59 rear). Chosen production turn:
**R = 0.6 m at v ≈ 0.2 m/s** via `beach_wheel_mixer/config/mixer_turn06.yaml` (`turn_gain_front 1.5`,
`min_turn_radius 0.50`, `max_w 0.45`; sand `mixer.yaml` untouched). Confirmed achieved R: left 0.611,
right 0.599. Launch with `mixer_params_file:=$(ros2 pkg prefix beach_wheel_mixer)/share/beach_wheel_mixer/config/mixer_turn06.yaml`.
The robot is **not L/R symmetric** (right turns ~0.04 m tighter at gain 1.6; shrank to 0.012 m at
1.5) — tune `front_right_scale`/`rear_*_scale` later if exact symmetry is needed.

### `coverage_bag_report` — usage

```bash
# After sourcing workspace (sand runs: --turn-thresh 0.08, see note below):
ros2 run beach_robot_bringup coverage_bag_report \
  ~/beach_robot_logs/coverage/sand_run_<STAMP> \
  --out ~/beach_robot_logs/reports/sand_cov1 --turn-thresh 0.08

# Or run directly (no colcon build needed):
python3 src/beach_robot_bringup/beach_robot_bringup/tools/coverage_bag_report.py \
  <bag_path> [--out <output_dir>] [--min-lane-sec 2.0] [--turn-thresh 0.08]
```
> **`--turn-thresh` (sand):** phase split uses `|angular_z|` (default 0.15 rad/s). Sand arcs at
> `turn_radius 1.8`, `desired_linear_vel 0.25` turn at `w = v/R ≈ 0.14 rad/s` — below 0.15, so
> turns get misclassified as straight lanes. Use **`--turn-thresh 0.08`** for these runs.
> Note: this tool reports **coverage metrics only** (lane/turn tracking from `/odometry/fusion_bno`);
> it does **not** compare imu/gps/encoder separately — use `localization_pose_report.py` for that.

**Output CSV files:**

| File | Contents |
|------|---------|
| `pose_trajectory.csv` | time, x, y, yaw, phase (straight_forward/backward/turn/idle), cmd_vel |
| `cmd_vel.csv` | time series of linear_x and angular_z commands |
| `wheel_speeds.csv` | wheel_cmd and enc_vel per wheel (FL/FR/RL/RR) |
| `lane_tracking.csv` | per lane: planned_y, actual_y start/end/mean, y errors, timing, x range |
| `turn_tracking.csv` | per turn: y before/after, planned vs actual y_end, y error, duration |
| `coverage_summary.csv` | totals: duration, lanes, mean/max/RMS y error, mean vel, area |

**Phase detection**: uses `|angular_z| > 0.15 rad/s` to classify straight vs turn phases. Short straight blips (< 2s) are merged into adjacent turns. Adjust `--min-lane-sec` if lanes are misclassified.
