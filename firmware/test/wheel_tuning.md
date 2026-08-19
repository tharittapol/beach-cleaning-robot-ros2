# Wheel Control Test And Tuning

This project uses two ESP32 boards:

- `esp32_enc`: reads the 4 wheel encoders and sends JSON over `Serial2`
- `esp32_main`: receives Jetson commands, closes the loop, and drives the motors

## Flash

```bash
pio run -e esp32_main -t upload
pio run -e esp32_enc -t upload
```

Open the main ESP32 USB serial monitor at `115200` baud.

## USB test commands

Send these commands to the `esp32_main` USB serial port:

```text
HELP
DBG 1
DBGRATE 100
STATUS
STOP
PID 1 0.25 0.08 0.00
VMAX 1 1.20
CLOSED 1 1
ENC_SIGN 1 1
MOTOR_SIGN 1 1
OPEN 1 0.20 1500
STEP 1 0.30 3000 0
STEP 1 0.30 3000 1
```

Meaning:

- `OPEN <wheel> <motor_u> <duration_ms>`: direct motor command in `[-1.0, 1.0]`
- `STEP <wheel> <target_mps> <duration_ms> [closed_loop]`: one-wheel speed step test
- `PID <wheel|ALL> <kp> <ki> <kd>`: update gains live
- `VMAX <wheel|ALL> <mps>`: update feedforward scaling live
- `CLOSED <wheel|ALL> <0|1>`: switch open-loop or closed-loop live
- `ENC_SIGN <wheel|ALL> <-1|1>`: fix encoder polarity live
- `MOTOR_SIGN <wheel|ALL> <-1|1>`: fix motor direction live

Wheel numbering is `1..4 = FL, FR, RL, RR`.

## Jetson JSON commands

Normal wheel command:

```json
{"wheel_cmd":[0.2,0.2,0.2,0.2]}
```

One-wheel test from Jetson:

```json
{"wheel_test":{"wheel":1,"target_mps":0.30,"duration_ms":3000,"closed_loop":true}}
```

Open-loop direct motor test from Jetson:

```json
{"wheel_test":{"wheel":1,"motor_u":0.20,"duration_ms":1500}}
```

Live tuning from Jetson:

```json
{"pid_kp":[0.30,0.30,0.20,0.20],"pid_ki":[0.08,0.08,0.05,0.05],"pid_kd":[0.0,0.0,0.0,0.0]}
```

## Recommended tuning order

1. Lift the robot so one wheel can spin safely.
2. Turn on debug: `DBG 1` and `DBGRATE 100`.
3. Check motor direction with `OPEN`.
   If positive command spins the wrong direction, fix `MOTOR_SIGN`.
4. Check encoder direction in `enc_vel_corr`.
   If forward spin reads negative, fix `ENC_SIGN`.
5. Estimate each wheel's usable top speed and set `VMAX`.
   Increase `OPEN` in steps and note the resulting `enc_vel_corr` near full speed.
6. Test feedforward only with `STEP ... 0`.
   If speed is far from target, fix `VMAX` before pushing PID gains.
7. Raise `Kp` in small steps until the wheel reaches target quickly but does not oscillate.
8. Add `Ki` slowly until steady-state error is small.
9. Leave `Kd` at `0` unless you still need damping after `Kp` and `Ki` are close.


## Automatic USB runner

If you want to run the current `Kp`-only pass automatically and save one raw log file to send back,
use the helper script in this folder.

Install the one Python dependency:

```bash
python3 -m pip install pyserial
```

Run the default four-wheel pass:

```bash
python3 /home/orin_nano/Documents/PlatformIO/Projects/low_level_firmware_v2/test/run_wheel_tuning.py --port /dev/ttyUSB0
```

Optional live serial echo while it runs:

```bash
python3 /home/orin_nano/Documents/PlatformIO/Projects/low_level_firmware_v2/test/run_wheel_tuning.py --port /dev/ttyUSB0 --print-rx
```

Default step file:

```text
low_level_firmware_v2/test/kp_pass_stage1.json
```

That default profile runs:

- `PID 1 0.18 0 0` then `STEP 1 0.30 3000 1`
- `PID 2 0.18 0 0` then `STEP 2 0.30 3000 1`
- `PID 3 0.08 0 0` then `STEP 3 1.20 3000 1`
- `PID 4 0.05 0 0` then `STEP 4 0.80 3000 1`

Logs are written under:

```text
low_level_firmware_v2/test/logs/
```

The log contains both transmitted commands (`[TX]`) and received serial lines (`[RX]`), so you can send the file back directly for tuning review.

## What to watch in debug JSON

- `enc_vel_corr`: measured wheel speed
- `u_ff`: feedforward output from `wheel_cmd / vmax_mps`
- `u_pid`: PID correction
- `motor_u`: final output sent to the motor driver
- `pid_kp`, `pid_ki`, `pid_kd`: live gains
- `vmax_mps`: live feedforward scale
- `wheel_test`: active one-wheel test info

## Mechanical notes

Low-speed control also depends on the breakaway tuning in `low_level_firmware_v2/src/motor.cpp`:

- `PWM_START[4]`
- `PWM_HOLD[4]`
- `KICK_MS`

If a wheel sticks at low command, tune those values before increasing PID gains too much.

## Starting gains

Start from the current defaults:

- Front wheels: `Kp=0.25`, `Ki=0.08`, `Kd=0.00`
- Rear wheels: `Kp=0.20`, `Ki=0.05`, `Kd=0.00`

Final gains still need real hardware tuning because battery voltage, traction, and drivetrain friction change the result.
