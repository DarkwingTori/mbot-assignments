"""Graph searches for the MBot path planning project.

breadth_first_search() is the part the course project requires. a_star_search()
is the optional extension. Both take a GridGraph (graph.py) and return the
path as a list of (row, column) cells from start to goal, or an empty list if
there is no path, which is how the course's tracePath() reports it.
"""

import heapq
import math
from collections import deque

from graph import SQRT2


def breadth_first_search(graph, start, goal):
    """Finds a path with the fewest steps, using breadth first search.

    Every step counts the same, so a diagonal move is no more expensive than
    a straight one. That makes this the path with the fewest cells, not the
    shortest one in meters. Use a_star_search() for that.

    Args:
        graph (GridGraph): The map to search.
        start (tuple): The (row, column) to start from.
        goal (tuple): The (row, column) to reach.

    Returns:
        list: The cells from start to goal, or [] if the start or goal is
            blocked or no path exists.
    """
    start, goal = tuple(start), tuple(goal)
    graph.visited_cells = []
    if graph.check_collision(*start) or graph.check_collision(*goal):
        return []

    parents = {start: None}
    queue = deque([start])

    while queue:
        cell = queue.popleft()
        graph.visited_cells.append(cell)
        if cell == goal:
            return graph.trace_path(parents, goal)
        for neighbour in graph.find_neighbors(*cell):
            if neighbour not in parents:
                parents[neighbour] = cell
                queue.append(neighbour)

    return []


def octile_distance(a, b):
    """Shortest distance between two cells on an 8-connected grid.

    It never overestimates the true cost, so A* using it as the heuristic
    still finds the cheapest path.

    Args:
        a (tuple): First (row, column).
        b (tuple): Second (row, column).

    Returns:
        float: The octile distance in cell units.
    """
    d_row, d_col = abs(a[0] - b[0]), abs(a[1] - b[1])
    return (d_row + d_col) + (SQRT2 - 2.0) * min(d_row, d_col)


def get_score(cell, goal, cost_so_far):
    """A* score of a cell: cost so far plus the heuristic to the goal.

    This is the value A* sorts its open list by. Any clearance penalty is
    already part of cost_so_far.

    Args:
        cell (tuple): The cell to score.
        goal (tuple): The goal cell.
        cost_so_far (float): Cost of the best known path to the cell, in
            cell units.

    Returns:
        float: The score.
    """
    return cost_so_far + octile_distance(cell, goal)


def a_star_search(graph, start, goal, clearance=0.0, clearance_weight=1.0):
    """Finds the cheapest path with A*.

    Straight steps cost 1 and diagonal steps cost sqrt(2). With a clearance
    above zero, stepping onto a cell closer than that many meters to an
    obstacle costs extra in proportion to how close it is, so the path keeps
    away from walls but still goes through a tight gap if it has to.

    Args:
        graph (GridGraph): The map to search.
        start (tuple): The (row, column) to start from.
        goal (tuple): The (row, column) to reach.
        clearance (float): Distance from an obstacle in meters within which
            extra cost is added. 0 turns the extra cost off.
        clearance_weight (float): Extra cost in cell units for each cell of
            intrusion into the clearance distance.

    Returns:
        list: The cells from start to goal, or [] if the start or goal is
            blocked or no path exists.
    """
    start, goal = tuple(start), tuple(goal)
    graph.visited_cells = []
    if graph.check_collision(*start) or graph.check_collision(*goal):
        return []

    cost_so_far = {start: 0.0}
    parents = {start: None}
    closed = set()
    # Heap entries are (score, tie_breaker, cell). The counter stops heapq
    # from ever comparing two cells when the scores tie.
    counter = 0
    heap = [(get_score(start, goal, 0.0), counter, start)]

    while heap:
        _, _, cell = heapq.heappop(heap)
        if cell in closed:
            continue            # A cheaper entry for this cell was handled.
        closed.add(cell)
        graph.visited_cells.append(cell)

        if cell == goal:
            return graph.trace_path(parents, goal)

        for neighbour in graph.find_neighbors(*cell):
            if neighbour in closed:
                continue
            step = math.hypot(neighbour[0] - cell[0], neighbour[1] - cell[1])
            cost = cost_so_far[cell] + step

            if clearance > 0:
                idx = graph.cell_to_idx(*neighbour)
                intrusion = (clearance - graph.obstacle_distances[idx]) \
                    / graph.meters_per_cell
                if intrusion > 0:
                    cost += clearance_weight * intrusion

            if neighbour not in cost_so_far or cost < cost_so_far[neighbour]:
                cost_so_far[neighbour] = cost
                parents[neighbour] = cell
                counter += 1
                heapq.heappush(
                    heap, (get_score(neighbour, goal, cost),
                           counter, neighbour))

    return []


def path_length(path):
    """Length of a path in cell units.

    Args:
        path (list): Cells from start to goal.

    Returns:
        float: Straight steps count 1 and diagonal steps count sqrt(2).
    """
    return sum(math.hypot(b[0] - a[0], b[1] - a[1])
               for a, b in zip(path, path[1:]))
