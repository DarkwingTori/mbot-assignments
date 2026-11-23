"""Offline tests for astar.py. Run with `python3 test_astar.py` or pytest."""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from astar import (astar, cell_to_world, obstacle_distances, octile_distance,
                   path_length, world_to_cell)


def blank(rows, cols):
    return [[0] * cols for _ in range(rows)]


def test_straight_line():
    path = astar(blank(1, 6), (0, 0), (0, 5))
    assert path == [(0, c) for c in range(6)]


def test_start_equals_goal():
    assert astar(blank(3, 3), (1, 1), (1, 1)) == [(1, 1)]


def test_diagonal_is_shorter_than_two_straights():
    path = astar(blank(5, 5), (0, 0), (4, 4))
    assert abs(path_length(path) - 4 * math.sqrt(2)) < 1e-9


def test_goes_around_wall_through_gap():
    grid = blank(5, 5)
    for r in range(5):
        grid[r][2] = 1
    grid[4][2] = 0          # The only gap is on the bottom row.
    path = astar(grid, (0, 0), (0, 4))
    assert path is not None and (4, 2) in path
    assert all(grid[r][c] == 0 for r, c in path)


def test_no_path_returns_none():
    grid = blank(5, 5)
    for r in range(5):
        grid[r][2] = 1
    assert astar(grid, (0, 0), (0, 4)) is None


def test_blocked_start_or_goal_returns_none():
    grid = blank(3, 3)
    grid[0][0] = 1
    grid[2][2] = 1
    assert astar(grid, (0, 0), (1, 1)) is None
    assert astar(grid, (1, 1), (2, 2)) is None


def test_out_of_bounds_returns_none():
    assert astar(blank(3, 3), (0, 0), (5, 5)) is None


def test_no_corner_cutting():
    # Two obstacles touching at a corner: the robot must not slip between.
    grid = blank(2, 2)
    grid[0][1] = 1
    grid[1][0] = 1
    assert astar(grid, (0, 0), (1, 1)) is None


def test_path_is_optimal_against_octile_distance():
    path = astar(blank(8, 12), (0, 0), (7, 11))
    assert abs(path_length(path) - octile_distance((0, 0), (7, 11))) < 1e-9


def test_clearance_keeps_away_from_wall():
    # A wall along the top row: with clearance the path should drop away
    # from it, without it the path hugs the wall's edge.
    grid = blank(7, 9)
    for c in range(9):
        grid[0][c] = 1
    plain = astar(grid, (1, 0), (1, 8))
    safe = astar(grid, (1, 0), (1, 8), clearance_radius=3, clearance_weight=2.0)
    # Skip the start and goal cells themselves: both sit on row 1 by design.
    assert min(r for r, _ in plain[1:-1]) == 1
    assert min(r for r, _ in safe[1:-1]) > 1


def test_clearance_still_uses_a_tight_gap():
    grid = blank(5, 5)
    for r in range(5):
        grid[r][2] = 1
    grid[2][2] = 0          # One-cell gap, every cell near the wall.
    path = astar(grid, (2, 0), (2, 4), clearance_radius=3)
    assert path is not None and (2, 2) in path


def test_obstacle_distances():
    grid = blank(1, 5)
    grid[0][0] = 1
    assert obstacle_distances(grid)[0] == [0, 1, 2, 3, 4]


def test_world_conversion_round_trip():
    cell = (3, 7)
    point = cell_to_world(cell, 0.05, origin=(-1.0, -2.0))
    assert world_to_cell(point, 0.05, origin=(-1.0, -2.0)) == cell


if __name__ == "__main__":
    tests = [(n, f) for n, f in sorted(globals().items())
             if n.startswith("test_") and callable(f)]
    for name, fn in tests:
        fn()
        print(f"ok   {name}")
    print(f"\n{len(tests)} tests passed")
