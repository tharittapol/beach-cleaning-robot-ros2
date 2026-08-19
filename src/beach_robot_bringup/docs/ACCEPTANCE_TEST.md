# Acceptance Test (Robot can do work)

## A) Preflight (PASS/FAIL)
PASS if:
- /enc_vel present and changes when wheel moves
- /imu/data present
- TF map->base_link and odom->base_link available

## B) Manual drive
PASS if:
- Robot drives forward/back reliably
- E-stop stops movement immediately

## C) Localization
PASS if:
- odometry/local stable
- odometry/global doesn't jump wildly (outdoors)

## D) Nav2 goal
PASS if:
- Can reach a 5m goal safely
- Avoids simple obstacle

## E) Coverage (minimum)
PASS if:
- Completes 3 lanes continuously inside boundary
- Stops safely at end