"""Plan a path on the MBot's map and drive it.

On the robot, with SLAM running and localized:

    python3 robot_plan_path.py --goal 1.5 0.5

reads the pose from SLAM, plans from there to the goal on ~/current.map, and
sends the path to the robot with MBot.drive_path(). Goals are in meters in the
map frame, which is relative to where the robot was when mapping started.

Without a robot, --start and --dry-run plan and print the waypoints only:

    python3 robot_plan_path.py --map my.map --start 0 0 --goal 1.5 0.5 --dry-run
"""

import argparse
import math
import os
import time

from graph import GridGraph
from graph_search import a_star_search, breadth_first_search, path_length

DEFAULT_MAP = os.path.expanduser("~/current.map")
GOAL_TOLERANCE = 0.1    # Close enough to the goal to call it reached (m).
WAIT_TIMEOUT = 120.0    # Give up waiting for the robot after this long (s).


def load_map(path, **kwargs):
    """Reads a map file into a GridGraph.

    ASSUMED FORMAT, not confirmed from the course docs: the first line holds
    "origin_x origin_y width height meters_per_cell", followed by
    width * height occupancy values, row by row, separated by whitespace.
    Check it against the real file (`head -c 300 ~/current.map`) and change
    this function if it differs.

    Args:
        path (str): The map file.
        **kwargs: Passed on to GridGraph (for example threshold).

    Returns:
        GridGraph: The map.

    Raises:
        ValueError: If the file does not look like the assumed format.
    """
    with open(path) as f:
        tokens = f.read().split()
    if len(tokens) < 5:
        raise ValueError(f"{path}: too short to be a map file.")
    try:
        origin_x, origin_y = float(tokens[0]), float(tokens[1])
        width, height = int(tokens[2]), int(tokens[3])
        meters_per_cell = float(tokens[4])
        odds = [float(t) for t in tokens[5:]]
    except ValueError as err:
        raise ValueError(f"{path}: not in the assumed map format ({err}).")
    if len(odds) != width * height:
        raise ValueError(
            f"{path}: header says {width} x {height} = {width * height} "
            f"cells but the file has {len(odds)} values, so the format is "
            "not what this loader assumes.")
    return GridGraph(width, height, origin_x, origin_y, meters_per_cell,
                     cell_odds=odds, **kwargs)


def path_to_waypoints(graph, path, goal_pos=None, goal_theta=None):
    """Turns a cell path into the waypoints MBot.drive_path() takes.

    Cells in a straight run add nothing, so only the points where the path
    changes direction are kept, plus the start and end. The last waypoint is
    the exact goal position if one is given, rather than its cell's center.

    Args:
        graph (GridGraph): The map the path was planned on.
        path (list): The (row, column) cells from start to goal.
        goal_pos (tuple): The exact goal (x, y) in meters, or None.
        goal_theta (float): The heading to finish with in radians, or None
            to leave the heading to drive_path's default.

    Returns:
        list: Waypoints as [x, y], with the last one [x, y, theta] if a
            heading was given. Empty if the path is empty.
    """
    if not path:
        return []

    kept = [path[0]]
    for prev, cell, nxt in zip(path, path[1:], path[2:]):
        if (cell[0] - prev[0], cell[1] - prev[1]) != \
                (nxt[0] - cell[0], nxt[1] - cell[1]):
            kept.append(cell)
    if len(path) > 1:
        kept.append(path[-1])

    waypoints = [list(graph.cell_to_pos(*cell)) for cell in kept]
    if goal_pos is not None:
        waypoints[-1] = [goal_pos[0], goal_pos[1]]
    if goal_theta is not None:
        waypoints[-1].append(goal_theta)
    return waypoints


def plan(graph, start_pos, goal_pos, use_astar=True):
    """Plans from a start position to a goal position, both in meters.

    Args:
        graph (GridGraph): The map.
        start_pos (tuple): The start (x, y) in meters.
        goal_pos (tuple): The goal (x, y) in meters.
        use_astar (bool): A* if True, breadth first search if False.

    Returns:
        list: The cell path, or [] if there is none.
    """
    start = graph.pos_to_cell(*start_pos)
    goal = graph.pos_to_cell(*goal_pos)
    search = a_star_search if use_astar else breadth_first_search

    # If the robot is closer to a wall than the collision radius, the search
    # would refuse to start. Walk it out to the nearest free cell first, then
    # plan from there.
    escape = graph.escape_path(start)
    if not escape:
        return []
    path = search(graph, escape[-1], goal)
    if not path:
        return []
    return escape[:-1] + path


def wait_for_goal(robot, goal_pos):
    """Waits until the robot is near the goal, since drive_path() returns at
    once.

    Args:
        robot (MBot): The robot.
        goal_pos (tuple): The goal (x, y) in meters.

    Returns:
        bool: True if the robot got within GOAL_TOLERANCE of the goal, False
            if it timed out.
    """
    deadline = time.time() + WAIT_TIMEOUT
    while time.time() < deadline:
        pose = robot.read_slam_pose()
        if pose is not None and len(pose) == 3:
            if math.hypot(goal_pos[0] - pose[0],
                          goal_pos[1] - pose[1]) < GOAL_TOLERANCE:
                return True
        time.sleep(0.1)
    return False


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--goal", type=float, nargs=2, required=True,
                        metavar=("X", "Y"), help="goal in meters, map frame")
    parser.add_argument("--theta", type=float, default=None,
                        help="finishing heading in radians (optional)")
    parser.add_argument("--map", default=DEFAULT_MAP, help="map file")
    parser.add_argument("--bfs", action="store_true",
                        help="use breadth first search instead of A*")
    parser.add_argument("--start", type=float, nargs=2, metavar=("X", "Y"),
                        help="start in meters instead of reading SLAM "
                             "(needed with --dry-run and no robot)")
    parser.add_argument("--dry-run", action="store_true",
                        help="plan and print the waypoints but don't drive")
    args = parser.parse_args()

    graph = load_map(args.map)
    goal_pos = tuple(args.goal)

    robot = None
    if args.start is not None:
        start_pos = tuple(args.start)
    else:
        from mbot_bridge.api import MBot    # Only needed on the robot.
        robot = MBot()
        pose = robot.read_slam_pose()
        if pose is None or len(pose) != 3:
            raise SystemExit("No SLAM pose from the robot. Is SLAM running "
                             "and localized?")
        start_pos = (pose[0], pose[1])

    path = plan(graph, start_pos, goal_pos, use_astar=not args.bfs)
    if not path:
        raise SystemExit("No path: the start or goal is blocked, or the goal "
                         "can't be reached.")

    waypoints = path_to_waypoints(graph, path, goal_pos, args.theta)
    print(f"Path: {len(path)} cells, {path_length(path) * graph.meters_per_cell:.2f} m,"
          f" {len(waypoints)} waypoints")
    for waypoint in waypoints:
        print("  " + ", ".join(f"{v:.2f}" for v in waypoint))

    if args.dry_run:
        return
    if robot is None:
        from mbot_bridge.api import MBot
        robot = MBot()

    try:
        robot.drive_path(waypoints)
        print("Driving. Ctrl-C to stop.")
        reached = wait_for_goal(robot, goal_pos)
        print("Goal reached." if reached else "Timed out before the goal.")
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        robot.stop()


if __name__ == "__main__":
    main()
