# MBot Assignments

Python code for the MBot-Omni (University of Michigan robot with three omni wheels), using the `mbot_bridge` API.

| Folder | Assignment | What it does |
| --- | --- | --- |
| [`wall_follower/`](wall_follower) | Wall Follower | Follows the nearest wall on either side at a set distance, using proportional control with a deadband and a stall guard. |
| [`bug_navigation/`](bug_navigation) | Bug Navigation | Drives to a goal pose using odometry and follows the edge of any obstacle in the way. See [ALGORITHM.md](bug_navigation/ALGORITHM.md). |
| [`follow_me/`](follow_me) | Follow 1D and 2D | Follows an object using the Lidar: `follow_1D.py` along the line in front of the robot, `follow_2D.py` toward the nearest object. `teleop.py` drives the robot from a gamepad (needs `pygame`). |
| [`path_planning/`](path_planning) | Path Planning | A* search on an occupancy grid, with tests that run without a robot. |

## Running on the robot

Copy a script to the robot and run it there:

```bash
scp wall_follower/wall_follower.py mbot@<robot-ip>:~/
ssh mbot@<robot-ip> 'python3 ~/wall_follower.py'
```

Press Ctrl-C to stop. The scripts stop the motors on the way out.

## Status

- **Path planning:** tested offline with `python3 path_planning/test_astar.py`.
- **Wall follower and bug navigation:** written and run on the robot in pieces, but not yet checked end to end. The robot did not always move as commanded, and the robot's forward direction versus the Lidar and drive signs (`LIDAR_ANGLE_SIGN`, and the `DRIVE_X_SIGN` and `LIDAR_ANGLE_OFFSET` constants in the wall follower) still need to be calibrated on the hardware.
- **Follow 1D/2D:** copied as-is from the Follow Me checkpoint repo; not changed or re-tested while building this repo.

## License

See [LICENSE.txt](LICENSE.txt).
