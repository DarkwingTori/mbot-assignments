"""2D Follow Me controller: drives the robot toward or away from the
nearest obstacle detected by the Lidar, maintaining a fixed distance
regardless of which direction that obstacle is in, using a PD
controller to avoid overshoot.
"""
import time
import numpy as np
from mbot_bridge.api import MBot

SETPOINT = 0.5       # Desired distance to hold from the nearest obstacle, in meters.
DEADBAND = 0.02      # No correction while within this distance of SETPOINT (m).
KP = 1.0             # Proportional gain.
KD = 0.3             # Derivative gain, damps the approach so it doesn't overshoot.
MAX_SPEED = 0.3      # Speed cap, in m/s.
STOP_DIST = 0.15     # Closer than this the robot is probably touching it (m).
BACKOFF_SPEED = 0.1  # Speed backing away from the obstacle when too close (m/s).
DT = 0.05            # Loop period, in seconds.

# Sign between the raw Lidar angle and the robot body frame (+x forward, +y
# left, counter-clockwise positive). On this robot the raw Lidar angle already
# matches the body frame: an obstacle on the right reads about -90 deg. Set to
# -1.0 if the printed angle is positive when the obstacle is on the right.
LIDAR_ANGLE_SIGN = 1.0


def find_min_dist(ranges, thetas):
    """Finds the length and angle of the minimum ray in the scan.
    Args:
        ranges (list): The length of each ray in the Lidar scan.
        thetas (list): The angle of each ray in the Lidar scan.
    Returns:
        tuple: The length and angle of the shortest valid ray in the
            scan, or (None, None) if no valid rays are found.
    """
    ranges = np.array(ranges)
    thetas = np.array(thetas)

    valid_idx = (ranges > 0).nonzero()
    valid_ranges = ranges[valid_idx]
    valid_thetas = thetas[valid_idx]

    if len(valid_ranges) == 0:
        return None, None

    min_idx = np.argmin(valid_ranges)
    return valid_ranges[min_idx], valid_thetas[min_idx]


def read_scan(robot):
    """Reads a Lidar scan, or returns None if the robot has none to give."""
    scan = robot.read_lidar()
    if scan is None or len(scan) != 2 or len(scan[0]) == 0:
        return None
    return scan


def main():
    robot = MBot()

    prev_dist = None     # Distance from the last *new* scan.
    prev_time = None     # When that scan was first seen.
    derivative = 0.0     # Rate of change of the distance, m/s.

    try:
        while True:
            scan = read_scan(robot)
            if scan is None:
                dist_to_wall, angle_to_wall = None, None
            else:
                dist_to_wall, angle_to_wall = find_min_dist(*scan)

            if dist_to_wall is None:
                print("no valid Lidar reading; holding still")
                robot.stop()
                time.sleep(DT)
                continue

            now = time.time()

            # The Lidar publishes new scans slower than this loop runs, so on
            # many passes we read the same scan we already used. Differencing
            # two copies of one scan gives a derivative of 0, then the next
            # new scan gives double. Only update the derivative when the
            # reading has actually changed, and divide by the real time since
            # the last new scan.
            if prev_dist is None:
                prev_dist, prev_time = dist_to_wall, now
            elif dist_to_wall != prev_dist:
                derivative = (dist_to_wall - prev_dist) / (now - prev_time)
                prev_dist, prev_time = dist_to_wall, now

            # Direction of the nearest point in the robot body frame.
            # Wrapped to [-pi, pi] so it prints as -90 for "right", not 270.
            body_angle = np.arctan2(np.sin(LIDAR_ANGLE_SIGN * angle_to_wall),
                                    np.cos(LIDAR_ANGLE_SIGN * angle_to_wall))

            if dist_to_wall < STOP_DIST:
                # Too close, probably touching it. Freezing here would leave
                # the robot jammed, so back straight away from it.
                speed = -BACKOFF_SPEED
            else:
                error = dist_to_wall - SETPOINT   # Positive: too far, drive toward it.
                if abs(error) < DEADBAND:
                    speed = 0.0
                else:
                    # The derivative brakes the approach if the distance is
                    # shrinking quickly.
                    speed = KP * error + KD * derivative
                    speed = float(np.clip(speed, -MAX_SPEED, MAX_SPEED))

            # Decompose the speed along the direction of the closest ray so
            # the robot drives straight at (or away from) the nearest
            # point, not just straight ahead.
            vx = speed * np.cos(body_angle)
            vy = speed * np.sin(body_angle)

            print(f"dist: {dist_to_wall:.2f} m   angle: "
                  f"{np.degrees(body_angle):6.1f} deg   speed: {speed:+.2f} m/s")
            robot.drive(vx, vy, 0)
            time.sleep(DT)
    except KeyboardInterrupt:
        pass
    except Exception as err:
        # Say what went wrong instead of silently stopping.
        print(f"\nStopped: {err}")
    finally:
        # Always stop the robot on the way out, however the loop ended.
        robot.stop()


if __name__ == "__main__":
    main()
