# MBot wheel problem: motor 1 not turning

Robot: `mbot-0000` (MBot Omni, three omni wheels). Found 2026-11-24 while testing `bug_navigation.py`.

## Summary

When the robot is told to drive, **motor 1 is powered but its encoder never moves**, so
only motor 3 pushes the robot. The robot spins and drifts instead of driving where it is
told. The navigation code is not the cause: in simulation it reaches the goal and goes
around obstacles correctly, and the wall follower worked earlier on this same robot.

## What we saw

Running `bug_navigation.py` with goal (1.5 m, 0 m, 0 rad) and a box in the way:

- The robot drove sideways/backward instead of toward the goal.
- Odometry heading rotated to about -81 degrees while the code commanded zero rotation
  (`wz = 0`).
- The position moved away from the goal (x went from 0 to -0.42 m, goal distance grew from
  1.50 m to 1.93 m) and then froze.
- The state never left `DRIVE_TO_GOAL`. The nearest obstacle in the path stayed at 0.59 m or
  more, and the code only counts the path as blocked under 0.5 m, so avoidance never started.

## Tests run on the robot

All commands sent with `MBot.drive(vx, vy, wz)` for 1 second, then `stop()`.

| Command | Expected change | Odometry change |
| --- | --- | --- |
| forward, 0.2 m/s | about +0.20 m in x | +0.027 m x, -0.023 m y, **-10.6 deg** |
| backward, -0.2 m/s | about -0.20 m in x | back to about the start, +2.2 deg |
| left, +0.2 m/s | about +0.20 m in y | about 0, no response |
| right, -0.2 m/s | about -0.20 m in y | about 0, no response |
| rotate, +0.5 rad/s | about +29 deg | +0.6 deg |
| rotate, -0.5 rad/s | about -29 deg | -0.5 deg |

Motor channels, read while driving forward at 0.2 m/s:

| | Motor 1 | Motor 2 | Motor 3 |
| --- | --- | --- | --- |
| PWM sent (`MBOT_MOTOR_PWM`) | 0.094 | 0.0 | 0.126 |
| Encoder `delta_ticks` (`MBOT_ENCODERS`) | **0** | 0 | 15 |
| Encoder `ticks`, idle vs. driving | -129493 vs. -129493 | 19244 vs. 19244 | 38314 vs. 38550 |

Driving straight forward on a three-wheel omni robot should turn motors 1 and 3 at equal and
opposite speeds, with motor 2 still. Motor 2 being still is correct. Motor 1 is being
commanded but does not move.

## What this does and does not show

- **Measured:** motor 1 gets a non-zero PWM and its encoder count does not change. Motor 3
  responds. The measured body velocity (`MBOT_VEL`) during the forward command was about
  vx 0.04, vy -0.02, wz -0.22, far from the commanded vx 0.2, vy 0, wz 0.
- **Not known:** whether the cause is the motor, the wheel/gearbox jamming, a loose motor or
  encoder cable, or a weak battery that cannot turn a stiff motor at low power.
  The Pi reports no under-voltage (`vcgencmd get_throttled` = `0x0`), but that is the
  computer's supply, not the motor battery.

## Software that was ruled out

- All `mbot-*` services were running (bridge, serial link, motion controller, lidar, SLAM).
- Odometry publishes (it was empty earlier, until the robot was power-cycled).
- The lidar works and `wall_follower.py` followed a wall earlier the same day.
- `bug_navigation.py` run against a simulated robot reached the goal and went around a box in
  each case tried.

## Things to check

1. Motor 1's power and encoder cables, at the motor and at the board.
2. Wheel 1 by hand with the robot lifted: it should spin freely. Look for jams or debris on
   the axle.
3. Battery voltage on the OLED screen.
4. After any fix, repeat the 4-direction test. Each 1-second move should show about 0.2 m
   (or 29 degrees for the rotations) with almost no twist on the straight moves, and the
   forward move should show encoder ticks on both motor 1 and motor 3.

If motor 1 still does not turn with good cabling and a charged battery, the motor or the
motor board needs repair.
