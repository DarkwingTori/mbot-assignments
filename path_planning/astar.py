"""A* path planning on a 2D occupancy grid.

The grid is a list of rows. A cell holds 0 when it is free and anything else
when it is occupied. Cells are addressed as (row, col), with row 0 at the top.

Moves are 8-connected. A diagonal move may not cut the corner of an occupied
cell, so the robot never squeezes between two diagonally touching obstacles.
"""

import heapq
import math
from collections import deque

SQRT2 = math.sqrt(2.0)

# The 8 neighbours as (d_row, d_col, step cost).
MOVES = [
    (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
    (-1, -1, SQRT2), (-1, 1, SQRT2), (1, -1, SQRT2), (1, 1, SQRT2),
]


def in_bounds(grid, cell):
    """Checks whether a cell lies inside the grid.

    Args:
        grid (list): The occupancy grid.
        cell (tuple): The (row, col) to check.

    Returns:
        bool: True if the cell is inside the grid.
    """
    row, col = cell
    return 0 <= row < len(grid) and 0 <= col < len(grid[0])


def is_free(grid, cell):
    """Checks whether a cell is inside the grid and not occupied.

    Args:
        grid (list): The occupancy grid.
        cell (tuple): The (row, col) to check.

    Returns:
        bool: True if the cell can be driven through.
    """
    return in_bounds(grid, cell) and grid[cell[0]][cell[1]] == 0


def octile_distance(a, b):
    """Shortest distance between two cells on an 8-connected grid.

    This never overestimates the true path cost, so A* using it as the
    heuristic still finds the cheapest path.

    Args:
        a (tuple): First (row, col).
        b (tuple): Second (row, col).

    Returns:
        float: The octile distance in cell units.
    """
    d_row = abs(a[0] - b[0])
    d_col = abs(a[1] - b[1])
    return (d_row + d_col) + (SQRT2 - 2.0) * min(d_row, d_col)


def obstacle_distances(grid):
    """Computes each cell's distance to the nearest obstacle.

    Uses a breadth-first search that starts from every obstacle at once, so
    it costs one pass over the grid however many obstacles there are.
    Distances are counted in 8-connected steps (cells).

    Args:
        grid (list): The occupancy grid.

    Returns:
        list: A grid of the same shape holding the distance in cells.
            Cells in a grid with no obstacles get infinity.
    """
    rows, cols = len(grid), len(grid[0])
    dist = [[math.inf] * cols for _ in range(rows)]
    queue = deque()

    for r in range(rows):
        for c in range(cols):
            if grid[r][c] != 0:
                dist[r][c] = 0
                queue.append((r, c))

    while queue:
        r, c = queue.popleft()
        for d_row, d_col, _ in MOVES:
            nr, nc = r + d_row, c + d_col
            if 0 <= nr < rows and 0 <= nc < cols and dist[nr][nc] == math.inf:
                dist[nr][nc] = dist[r][c] + 1
                queue.append((nr, nc))

    return dist


def astar(grid, start, goal, clearance_radius=0, clearance_weight=1.0):
    """Finds the cheapest path between two cells with A*.

    With clearance_radius above zero, cells within that many cells of an
    obstacle cost extra, in proportion to how close they are. The path then
    prefers to keep away from walls while still going through tight gaps if
    that is the only way.

    Args:
        grid (list): The occupancy grid, 0 for free.
        start (tuple): The (row, col) to start from.
        goal (tuple): The (row, col) to reach.
        clearance_radius (float): Distance from an obstacle, in cells, within
            which extra cost is added. 0 turns the extra cost off.
        clearance_weight (float): Extra cost per cell of intrusion into the
            clearance radius.

    Returns:
        list: The cells from start to goal inclusive, or None if the start or
            goal is blocked or no path exists.
    """
    start, goal = tuple(start), tuple(goal)
    if not is_free(grid, start) or not is_free(grid, goal):
        return None

    near_obstacle = None
    if clearance_radius > 0:
        near_obstacle = obstacle_distances(grid)

    # Heap entries are (f, tie_breaker, cell). The counter keeps heapq from
    # ever comparing two cells when the f values tie.
    counter = 0
    open_heap = [(octile_distance(start, goal), counter, start)]
    cost_so_far = {start: 0.0}
    came_from = {start: None}
    closed = set()

    while open_heap:
        _, _, current = heapq.heappop(open_heap)
        if current in closed:
            continue            # A cheaper entry for this cell was handled.
        closed.add(current)

        if current == goal:
            return reconstruct_path(came_from, goal)

        for d_row, d_col, step in MOVES:
            neighbour = (current[0] + d_row, current[1] + d_col)
            if not is_free(grid, neighbour) or neighbour in closed:
                continue

            # No cutting the corner of an obstacle on a diagonal move.
            if d_row != 0 and d_col != 0:
                if (not is_free(grid, (current[0] + d_row, current[1]))
                        or not is_free(grid, (current[0], current[1] + d_col))):
                    continue

            cost = cost_so_far[current] + step
            if near_obstacle is not None:
                intrusion = clearance_radius - near_obstacle[neighbour[0]][neighbour[1]]
                if intrusion > 0:
                    cost += clearance_weight * intrusion

            if neighbour not in cost_so_far or cost < cost_so_far[neighbour]:
                cost_so_far[neighbour] = cost
                came_from[neighbour] = current
                counter += 1
                priority = cost + octile_distance(neighbour, goal)
                heapq.heappush(open_heap, (priority, counter, neighbour))

    return None


def reconstruct_path(came_from, goal):
    """Walks back from the goal to the start to build the path.

    Args:
        came_from (dict): Maps each reached cell to the cell before it.
        goal (tuple): The cell to walk back from.

    Returns:
        list: The cells from start to goal.
    """
    path = []
    cell = goal
    while cell is not None:
        path.append(cell)
        cell = came_from[cell]
    path.reverse()
    return path


def path_length(path):
    """Length of a path in cell units.

    Args:
        path (list): Cells from start to goal.

    Returns:
        float: Straight steps count 1 and diagonal steps count sqrt(2).
    """
    return sum(math.hypot(b[0] - a[0], b[1] - a[1])
               for a, b in zip(path, path[1:]))


def cell_to_world(cell, resolution, origin=(0.0, 0.0)):
    """Converts a grid cell to the world position of its center.

    x grows with the column and y grows with the row, matching the grid
    layout. If your map stores row 0 at the top and y upward, flip the row
    yourself before calling this.

    Args:
        cell (tuple): The (row, col) of the cell.
        resolution (float): Cell size in meters.
        origin (tuple): World (x, y) of the corner of cell (0, 0).

    Returns:
        tuple: The world (x, y) in meters.
    """
    row, col = cell
    return (origin[0] + (col + 0.5) * resolution,
            origin[1] + (row + 0.5) * resolution)


def world_to_cell(point, resolution, origin=(0.0, 0.0)):
    """Converts a world position to the cell that contains it.

    Args:
        point (tuple): The world (x, y) in meters.
        resolution (float): Cell size in meters.
        origin (tuple): World (x, y) of the corner of cell (0, 0).

    Returns:
        tuple: The (row, col) of the cell.
    """
    return (int(math.floor((point[1] - origin[1]) / resolution)),
            int(math.floor((point[0] - origin[0]) / resolution)))
