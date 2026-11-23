import time
import numpy as np
from mbot_bridge.api import MBot

# Wall follower for the University of Michigan MBot Omni (hexagonal base,
# three omni wheels).

def find_min_dist(ranges, thetas):
    """Finds the length and angle of the minimum ray in the scan.

    Make sure you ignore any rays with length 0! Those are invalid.

    Args:
        ranges (list): The length of each ray in the Lidar scan.
        thetas (list): The angle of each ray in the Lidar scan.

    Returns:
        tuple: The length and angle of the shortest ray in the Lidar scan.
    """
    min_dist, min_angle = None, None

    for dist, angle in zip(ranges, thetas):
        # Rays with zero range are invalid and must be skipped.
        if dist <= 0:
            continue
        if min_dist is None or dist < min_dist:
            min_dist, min_angle = dist, angle

    return min_dist, min_angle


def cross_product(v1, v2):
    """Compute the Cross Product between two vectors.

    Args:
        v1 (list): First vector of length 3.
        v2 (list): Second vector of length 3.

    Returns:
        list: The result of the cross product operation.
    """
    res = np.zeros(3)
    res[0] = v1[1] * v2[2] - v1[2] * v2[1]
    res[1] = v1[2] * v2[0] - v1[0] * v2[2]
    res[2] = v1[0] * v2[1] - v1[1] * v2[0]
    return res


def wrap_angle(angle):
    """Wrap an angle (or array of angles) to [-pi, pi]."""
    return np.arctan2(np.sin(angle), np.cos(angle))


robot = MBot()

SETPOINT = 0.35         # Desired distance from the wall (meters). Must be less
                        # than half the corridor width.
DEADBAND = 0.05         # No correction while within this distance of SETPOINT (m).
DRIVE_SPEED = 0.25      # Speed along the wall (m/s).
KP = 1.2                # Proportional gain: correction speed per meter of error.
MAX_CORRECTION = 0.15   # Cap on the correction speed toward/away from the wall (m/s).

# If the robot is closer than this it is probably touching the wall and
# jammed, so ignore the along-wall motion and back straight away from it.
STALL_DIST = 0.15       # (m)
STALL_SPEED = 0.15      # Speed away from the wall while freeing itself (m/s).
STALL_WARN_TIME = 3.0   # Warn if the lidar reading is frozen this long (s).

# The Lidar reports angles measured clockwise, which is the opposite of the
# robot body frame's right-hand-rule convention. Set to 1.0 if the raw lidar
# angle is positive when the wall is physically on the robot's left.
LIDAR_ANGLE_SIGN = -1.0

# Calibration for a robot whose rear leads the motion. Only one of these
# should need changing; measure it on the robot.
#   LIDAR_ANGLE_OFFSET: set to np.pi if the lidar's 0 deg is the robot's rear
#                       (an object right in front reads about 180 deg).
#   DRIVE_X_SIGN: set to -1.0 if commanding +x moves the robot toward its rear.
LIDAR_ANGLE_OFFSET = 0.0
DRIVE_X_SIGN = 1.0

# Rays this far past the front/back of the latched side still count as "the
# wall", so the robot can see a wall ahead when rounding an inside corner.
SIDE_MARGIN = np.radians(20)

wall_side = None        # +1 = wall on the left, -1 = wall on the right (latched).
last_reading = None     # (dist, angle) of the previous scan, for stall warnings.
last_change = time.time()

try:
    while True:
        # Read the latest lidar scan and convert every ray to the body frame
        # (+x forward, +y left, angles counter-clockwise, in [-pi, pi]).
        ranges, thetas = robot.read_lidar()
        ranges = np.asarray(ranges)
        angles = wrap_angle(LIDAR_ANGLE_SIGN * np.asarray(thetas) + LIDAR_ANGLE_OFFSET)

        # Latch the side of the nearest wall on the first valid scan, so the
        # robot keeps following that wall instead of flipping between two
        # walls in a corridor.
        if wall_side is None:
            first_dist, first_angle = find_min_dist(ranges, angles)
            if first_dist is not None:
                wall_side = 1.0 if first_angle > 0 else -1.0
        if wall_side is None:
            robot.stop()
            time.sleep(0.1)
            continue

        # Only rays on the latched side (plus a margin past the front and
        # back) count as the wall.
        on_side = (wall_side * angles > -SIDE_MARGIN) | (np.abs(angles) > np.pi - SIDE_MARGIN)
        min_dist, angle = find_min_dist(ranges[on_side], angles[on_side])
        if min_dist is None:
            # No valid rays on that side; stop and try again.
            robot.stop()
            time.sleep(0.1)
            continue

        print(f"dist: {min_dist:.2f} m   angle: {np.degrees(angle):6.1f} deg")

        # Warn if the reading never changes: the robot is likely stuck.
        now = time.time()
        if (last_reading is None
                or abs(min_dist - last_reading[0]) > 0.005
                or abs(angle - last_reading[1]) > np.radians(0.5)):
            last_reading = (min_dist, angle)
            last_change = now
        elif now - last_change > STALL_WARN_TIME:
            print("STALL: lidar reading has not changed; robot may be stuck")
            last_change = now

        # Unit vector pointing from the robot toward the nearest point on the
        # wall. The shortest ray is perpendicular to the wall, so this is the
        # wall normal.
        to_wall = np.array([np.cos(angle), np.sin(angle), 0.0])

        # Cross z-hat with the normal to get a unit vector parallel to the
        # wall. It points forward (+x) when the wall is on the right, so negate
        # it for a wall on the left. The direction is fixed by the latched
        # side, so it can't flip on noise when the wall is dead ahead.
        along_wall = -wall_side * cross_product([0.0, 0.0, 1.0], to_wall)

        if min_dist < STALL_DIST:
            # Too close: back straight away from the wall to get unstuck.
            velocity = -STALL_SPEED * to_wall
        else:
            # Proportional control with a deadband: correction scales with the
            # distance error (positive = too far, steer into the wall) and is
            # capped so a bad reading can't cause a violent move.
            error = min_dist - SETPOINT
            if abs(error) < DEADBAND:
                correction = 0.0
            else:
                correction = float(np.clip(KP * error, -MAX_CORRECTION, MAX_CORRECTION))

            # Combine driving along the wall with the correction toward/away
            # from it.
            velocity = DRIVE_SPEED * along_wall + correction * to_wall

        # DRIVE_X_SIGN corrects for a robot whose +x moves it toward its rear.
        robot.drive(DRIVE_X_SIGN * velocity[0], velocity[1], 0.0)

        # Sleep for a bit before reading a new scan.
        time.sleep(0.05)
except KeyboardInterrupt:
    pass
finally:
    # Always stop the robot on the way out, however the loop ended.
    robot.stop()
