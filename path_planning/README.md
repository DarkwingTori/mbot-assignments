# Path Planning

A* search on a 2D occupancy grid, in [astar.py](astar.py).

## How it works

- The grid is a list of rows. `0` is free and anything else is occupied. Cells are `(row, col)`.
- Moves are 8-connected. Straight steps cost 1 and diagonal steps cost `sqrt(2)`.
- A diagonal move is not allowed if it would cut the corner of an occupied cell.
- The heuristic is the octile distance. It never overestimates, so the path found is the cheapest.
- Optional clearance cost: with `clearance_radius > 0`, cells within that many cells of an obstacle cost extra, in proportion to how close they are. The path then keeps away from walls but still goes through a tight gap if it has to.
- `astar()` returns the list of cells from start to goal, or `None` if the start or goal is blocked or no path exists.

```python
from astar import astar, cell_to_world

path = astar(grid, start=(2, 3), goal=(40, 55), clearance_radius=4)
waypoints = [cell_to_world(cell, resolution=0.05) for cell in path]
```

## Tests

```bash
python3 test_astar.py
```

They run offline on small hand-made grids and cover straight paths, going around a wall through a gap, no-path cases, blocked or out-of-bounds cells, corner cutting, optimality, and the clearance cost.

## What is not done

This is a general A* planner written without the assignment's starter code. It does not read an MBot map file, run on the robot, or drive the waypoints. If the assignment needs a specific function signature or map format, adapt `astar()` to it.
