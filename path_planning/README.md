# Path Planning

Path planning on the MBot's occupancy map. It follows the structure of the course's [Project 3](https://hellorob.org/projects/p3): breadth first search is the required part, and A* is the optional extension.

| File | What it is |
| --- | --- |
| [`graph.py`](graph.py) | `GridGraph`: `width`, `height`, `origin_x`, `origin_y`, `meters_per_cell`, `collision_radius`, `threshold`, `cell_odds`, `obstacle_distances`, `visited_cells`. Includes `cell_to_idx`, `idx_to_cell`, `cell_to_pos`, `pos_to_cell`, `check_collision`, `find_neighbors`, `init_graph` and `trace_path`. |
| [`graph_search.py`](graph_search.py) | `breadth_first_search(graph, start, goal)` and `a_star_search(graph, start, goal)`. Each returns the cells from start to goal, or `[]` if there is no path. |
| [`robot_plan_path.py`](robot_plan_path.py) | Loads the map, reads the SLAM pose, plans, and sends the path to the robot with `MBot.drive_path()`. |
| [`test_planner.py`](test_planner.py) | 26 tests that run without a robot. |

## Built for the MBot

- **Cells.** `(i, j)` is `(row, column)`, as the course's `Cell` struct says. Column `j` runs along world x and row `i` along world y. `cell_to_pos` and `pos_to_cell` are the only places that know this.
- **Robot size.** A cell is blocked if it is occupied or closer to an obstacle than `collision_radius`. The default is `ROBOT_RADIUS` plus one cell width, as the course does. `ROBOT_RADIUS` is **0.15 m, a placeholder**: the pages give no body size, and in our runs the robot stalled 0.12 to 0.15 m from a wall. Measure the robot and set it in `graph.py`.
- **Moves.** 8-connected, and a diagonal step may not cut the corner of an obstacle.
- **Search.** BFS finds the path with the fewest steps, not the shortest one in meters. A* uses an octile heuristic, so it finds the shortest. With `clearance=` (meters), A* adds cost near walls so the path keeps away from them but still uses a tight gap if it must.
- **Driving.** Only the points where the path turns are sent, as `[x, y]` waypoints in meters, with the last as `[x, y, theta]` if you pass a heading. `drive_path()` returns immediately, so the script then watches `read_slam_pose()` until the robot is within 0.1 m of the goal, or 120 seconds pass.

## Running it

On the robot, with SLAM running and localized (goals are in meters in the map frame, relative to where mapping started):

```bash
python3 robot_plan_path.py --goal 1.5 0.5          # A*, current map ~/current.map
python3 robot_plan_path.py --goal 1.5 0.5 --bfs    # breadth first search
python3 robot_plan_path.py --goal 1.5 0.5 --theta 1.57 --dry-run   # plan only
```

Without a robot, give a start and plan only:

```bash
python3 robot_plan_path.py --map my.map --start 0 0 --goal 1.5 0.5 --dry-run
```

Tests: `python3 test_planner.py`.

## Not confirmed, check before relying on it

- **Map file format.** `load_map()` assumes the first line is `origin_x origin_y width height meters_per_cell`, followed by `width * height` occupancy values row by row. I could not confirm this from the course pages. Look at the real file with `head -c 300 ~/current.map` and change `load_map()` if it differs. It stops with an error if the sizes don't match.
- **Occupied value.** A cell is occupied when its value is at or above `threshold` (default 1). If the map stores log-odds, set `threshold` to what the course uses.
- **Row and column vs x and y.** The course page describes `Cell` in two different ways. This code uses row = y and column = x. If your map is the other way round, change `cell_to_pos` and `pos_to_cell`.
- **Starter code.** The course gives its own `graph.py`, `graph_search.py` and `robot_plan_path.py`. This is written from the page's description, not from those files, so function names or signatures may need adapting to the graded version.
- **Not run on the robot.** Everything here is tested offline only.
