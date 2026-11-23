import time
import pygame
from mbot_bridge.api import MBot


def apply_deadzone(value, deadzone):
    """Zeros out small stick/trigger noise near the center.
    Args:
        value (float): The raw axis value, in [-1, 1].
        deadzone (float): Values with absolute value below this are treated as 0.
    Returns:
        float: The axis value, or 0 if it was inside the deadzone.
    """
    if abs(value) < deadzone:
        return 0.0
    return value


# NOTE: Pair/connect the Xbox controller over Bluetooth at the OS level
# (e.g. bluetoothctl, or System Settings) before running this script --
# pygame only reads an already-connected joystick, it won't pair one.
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    raise RuntimeError("No controller found. Pair the Xbox controller first.")

joystick = pygame.joystick.Joystick(0)
joystick.init()

# NOTE: Axis/button indices below are for a typical Xbox Series controller,
# but pygame's mapping can vary by OS/driver. If driving feels wrong, print
# joystick.get_axis(i) for each i in a quick loop to confirm which index is
# which stick/trigger on your machine.
LEFT_STICK_X = 0
LEFT_STICK_Y = 1
RIGHT_STICK_X = 3
RIGHT_TRIGGER = 5

max_linear_speed = 0.3    # TODO: Tune this (m/s). Cap on vx/vy.
max_angular_speed = 1.0   # TODO: Tune this (rad/s). Cap on wz.
deadzone = 0.15           # TODO: Tune this. Ignores stick noise near center.
trigger_threshold = 0.0   # TODO: Tune this. Right trigger must exceed this to enable driving.

robot = MBot()

try:
    while True:
        pygame.event.pump()

        # Deadman switch: only drive while the right trigger is held past
        # the threshold. Otherwise stop immediately.
        trigger_val = joystick.get_axis(RIGHT_TRIGGER)
        if trigger_val <= trigger_threshold:
            robot.drive(0, 0, 0)
            time.sleep(0.1)
            continue

        # Left stick controls forward/strafe, right stick x controls rotation.
        raw_vx = -joystick.get_axis(LEFT_STICK_Y)  # Pushing up is negative on most controllers.
        raw_vy = -joystick.get_axis(LEFT_STICK_X)
        raw_wz = -joystick.get_axis(RIGHT_STICK_X)

        vx = apply_deadzone(raw_vx, deadzone) * max_linear_speed
        vy = apply_deadzone(raw_vy, deadzone) * max_linear_speed
        wz = apply_deadzone(raw_wz, deadzone) * max_angular_speed

        # Command the robot: forward/lateral speed, plus rotation.
        robot.drive(vx, vy, wz)

        # Optionally, sleep for a bit before reading the controller again.
        time.sleep(0.1)
except:
    # Catch any exception, including the user quitting, and stop the robot.
    robot.stop()
