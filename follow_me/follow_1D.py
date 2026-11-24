"""1D Follow Me controller: drives the robot forward/backward to
maintain a fixed distance from a wall directly ahead, using a
PD controller on Lidar range data.
"""
import time
import numpy as np
from mbot_bridge.api import MBot

SETPOINT = 0.5       # Desired distance to hold from the wall, in meters.
DEADBAND = 0.02      # No correction while within this distance of SETPOINT (m).
KP = 1.0             # Proportional gain.
KD = 0.3             # Derivative gain, damps the approach so it doesn't overshoot.
MAX_SPEED = 0.3      # Speed cap, in m/s.
STOP_DIST = 0.15     # Closer than this the robot is probably touching the wall (m).
BACKOFF_SPEED = 0.1  # Speed backing away from the wall when too close (m/s).
DT = 0.05            # Loop period, in seconds.


def find_fwd_dist(ranges, thetas, window=5):
    """Find the distance to the nearest object in front of the robot.
    Args:
        ranges (list): The ranges from the Lidar scan.
        thetas (list): The angles from the Lidar scan.
        window (int, optional): The window to average ranges over. Defaults to 5.
    Returns:
        float: The distance to the nearest obstacle in front of the robot,
            or None if no valid rays are found in the window.
    """
    # Convert first: if these were numpy arrays, + would add them elementwise
    # instead of joining the front and back of the scan.
    ranges = list(ranges)
    thetas = list(thetas)
    fwd_ranges = np.array(ranges[:window] + ranges[-window:])
    fwd_thetas = np.array(thetas[:window] + thetas[-window:])

    valid_idx = (fwd_ranges > 0).nonzero()
    fwd_ranges = fwd_ranges[valid_idx]
    fwd_thetas = fwd_thetas[valid_idx]

    if len(fwd_ranges) == 0:
        return None

    # cos() is the same for +angle and -angle, so this does not depend on
    # which way the Lidar counts angles.
    fwd_dists = fwd_ranges * np.cos(fwd_thetas)
    return np.mean(fwd_dists)


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
            dist_to_wall = None if scan is None else find_fwd_dist(*scan)

            if dist_to_wall is None:
                print("no valid Lidar reading ahead; holding still")
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

            if dist_to_wall < STOP_DIST:
                # Too close, probably touching the wall. Freezing here would
                # leave the robot jammed, so back straight away from it.
                vx = -BACKOFF_SPEED
            else:
                error = dist_to_wall - SETPOINT   # Positive: too far, drive forward.
                if abs(error) < DEADBAND:
                    vx = 0.0
                else:
                    # The derivative brakes the approach if the distance is
                    # shrinking quickly.
                    vx = KP * error + KD * derivative
                    vx = float(np.clip(vx, -MAX_SPEED, MAX_SPEED))

            print(f"dist: {dist_to_wall:.2f} m   vx: {vx:+.2f} m/s")
            robot.drive(vx, 0, 0)
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
