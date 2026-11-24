"""Occupancy-grid graph for the MBot path planning project.

The structure mirrors the course's GridGraph (width, height, origin_x,
origin_y, meters_per_cell, collision_radius, threshold, cell_odds,
obstacle_distances, visited_cells), so the planners in graph_search.py work on
the same shape of data the MBot uses.

A cell is a tuple (i, j): i is the row and j is the column, as the course's
Cell struct describes it. Column j runs along world x and row i runs along
world y. cell_to_pos() and pos_to_cell() are the only places that know this,
so if the course's map turns out to be laid out the other way, change them.

Cells are also numbered by a flat index idx = i * width + j, which is how
cell_odds and obstacle_distances are stored.
"""

import heapq
import math
from collections import deque

# Distance from the center of the robot to the edge of its body (m). The
# course pages don't give a value, so this is a conservative placeholder
# chosen from our own runs, where the robot sat jammed 0.12 to 0.15 m from a
# wall. Measure the real robot and set this.
ROBOT_RADIUS = 0.15

SQRT2 = math.sqrt(2.0)

# The 8 neighbours as (d_row, d_col, step cost in cells).
MOVES = [
    (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
    (-1, -1, SQRT2), (-1, 1, SQRT2), (1, -1, SQRT2), (1, 1, SQRT2),
]


class GridGraph:
    """A 2D occupancy grid plus the information needed to plan on it.

    Attributes:
        width (int): Number of columns.
        height (int): Number of rows.
        origin_x (float): World x of the corner of cell (0, 0) (m).
        origin_y (float): World y of the corner of cell (0, 0) (m).
        meters_per_cell (float): Width of one cell (m).
        collision_radius (float): Cells closer than this to an obstacle are
            treated as blocked (m).
        threshold (int): A cell with odds at or above this is occupied.
        cell_odds (list): Occupancy value of every cell, row by row.
        obstacle_distances (list): Distance in meters from every cell to the
            nearest occupied cell, row by row. Filled by init_graph().
        visited_cells (list): Cells a search looked at, for visualising.
    """

    def __init__(self, width, height, origin_x=0.0, origin_y=0.0,
                 meters_per_cell=0.05, collision_radius=None, threshold=1,
                 cell_odds=None):
        self.width = width
        self.height = height
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.meters_per_cell = meters_per_cell
        # The course adds one cell width to the robot radius for margin.
        self.collision_radius = (ROBOT_RADIUS + meters_per_cell
                                 if collision_radius is None
                                 else collision_radius)
        self.threshold = threshold
        self.cell_odds = (list(cell_odds) if cell_odds is not None
                          else [0] * (width * height))
        if len(self.cell_odds) != width * height:
            raise ValueError(
                f"cell_odds has {len(self.cell_odds)} values but a "
                f"{width} x {height} grid needs {width * height}.")
        self.obstacle_distances = []
        self.visited_cells = []
        self.init_graph()

    @classmethod
    def from_rows(cls, rows, **kwargs):
        """Builds a graph from a list of rows, e.g. a hand-made test map.

        Args:
            rows (list): A list of rows, each a list of occupancy values.
            **kwargs: Passed on to the constructor.

        Returns:
            GridGraph: The graph.
        """
        height, width = len(rows), len(rows[0])
        odds = [value for row in rows for value in row]
        return cls(width, height, cell_odds=odds, **kwargs)

    # --- indexing and coordinates ---

    def cell_to_idx(self, i, j):
        """Flat index of the cell in row i, column j."""
        return i * self.width + j

    def idx_to_cell(self, idx):
        """The (row, column) of a flat index."""
        return (idx // self.width, idx % self.width)

    def in_bounds(self, i, j):
        """Whether (row i, column j) is inside the grid."""
        return 0 <= i < self.height and 0 <= j < self.width

    def cell_to_pos(self, i, j):
        """World position (x, y) in meters of the center of a cell."""
        return (self.origin_x + (j + 0.5) * self.meters_per_cell,
                self.origin_y + (i + 0.5) * self.meters_per_cell)

    def pos_to_cell(self, x, y):
        """The (row, column) of the cell containing a world position."""
        return (int(math.floor((y - self.origin_y) / self.meters_per_cell)),
                int(math.floor((x - self.origin_x) / self.meters_per_cell)))

    # --- occupancy ---

    def is_occupied(self, i, j):
        """Whether the cell itself is an obstacle (ignoring robot size)."""
        return self.cell_odds[self.cell_to_idx(i, j)] >= self.threshold

    def check_collision(self, i, j):
        """Whether the robot's body would hit something at this cell.

        A cell collides if it is outside the grid, occupied, or closer to an
        obstacle than the robot's collision radius.

        Args:
            i (int): Row.
            j (int): Column.

        Returns:
            bool: True if the robot cannot be centered on this cell.
        """
        if not self.in_bounds(i, j):
            return True
        idx = self.cell_to_idx(i, j)
        return (self.cell_odds[idx] >= self.threshold
                or self.obstacle_distances[idx] < self.collision_radius)

    def init_graph(self):
        """Computes obstacle_distances for every cell.

        Runs one Dijkstra pass that starts from every occupied cell at once,
        with straight steps costing 1 and diagonal steps sqrt(2), then
        converts cells to meters. Cells in a grid with no obstacles get
        infinity.
        """
        cells = self.width * self.height
        dist = [math.inf] * cells
        heap = []
        for idx, odds in enumerate(self.cell_odds):
            if odds >= self.threshold:
                dist[idx] = 0.0
                heap.append((0.0, idx))
        heapq.heapify(heap)

        while heap:
            d, idx = heapq.heappop(heap)
            if d > dist[idx]:
                continue
            i, j = self.idx_to_cell(idx)
            for d_row, d_col, step in MOVES:
                ni, nj = i + d_row, j + d_col
                if not self.in_bounds(ni, nj):
                    continue
                nidx = self.cell_to_idx(ni, nj)
                if d + step < dist[nidx]:
                    dist[nidx] = d + step
                    heapq.heappush(heap, (d + step, nidx))

        self.obstacle_distances = [d * self.meters_per_cell for d in dist]

    # --- graph structure ---

    def find_neighbors(self, i, j):
        """Cells the robot can move to from (i, j) in one step.

        The graph is 8-connected. A neighbour is left out if the robot would
        collide there, and a diagonal move is left out if it would cut the
        corner of an obstacle.

        Args:
            i (int): Row.
            j (int): Column.

        Returns:
            list: The reachable neighbouring (row, column) cells.
        """
        neighbours = []
        for d_row, d_col, _ in MOVES:
            ni, nj = i + d_row, j + d_col
            if self.check_collision(ni, nj):
                continue
            if d_row != 0 and d_col != 0:
                if self.check_collision(i + d_row, j) \
                        or self.check_collision(i, j + d_col):
                    continue
            neighbours.append((ni, nj))
        return neighbours

    def escape_path(self, cell):
        """Shortest way out of the zone too close to an obstacle.

        The robot often ends up closer to a wall than the collision radius:
        ours sat 0.15 to 0.2 m from walls, while the collision radius is
        larger. Every search refuses to start from such a cell, so the robot
        would be told there is no path. This walks it out first, over cells
        that are not themselves obstacles, to the nearest cell with enough
        room.

        Args:
            cell (tuple): The (row, column) the robot is in.

        Returns:
            list: The cells from the given cell to the nearest cell the robot
                can be centered on. Just [cell] if it is already free. Empty
                if the cell is outside the grid or an obstacle, or no free
                cell can be reached.
        """
        cell = tuple(cell)
        if not self.in_bounds(*cell) or self.is_occupied(*cell):
            return []

        parents = {cell: None}
        queue = deque([cell])
        while queue:
            current = queue.popleft()
            if not self.check_collision(*current):
                return self.trace_path(parents, current)
            for d_row, d_col, _ in MOVES:
                neighbour = (current[0] + d_row, current[1] + d_col)
                if (neighbour in parents
                        or not self.in_bounds(*neighbour)
                        or self.is_occupied(*neighbour)):
                    continue
                parents[neighbour] = current
                queue.append(neighbour)
        return []

    def trace_path(self, parents, goal):
        """Walks back from the goal through the parents to build the path.

        Args:
            parents (dict): Maps each reached cell to the cell before it,
                with the start mapped to None.
            goal (tuple): The cell to walk back from.

        Returns:
            list: The cells from start to goal, or an empty list if the goal
                was never reached.
        """
        if goal not in parents:
            return []
        path = []
        cell = goal
        while cell is not None:
            path.append(cell)
            cell = parents[cell]
        path.reverse()
        return path
