"""Offline tests for the planner. Run with `python3 test_planner.py` or pytest."""

import math
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from graph import GridGraph
from graph_search import (a_star_search, breadth_first_search, octile_distance,
                          path_length)
from robot_plan_path import load_map, path_to_waypoints, plan


def make(rows, **kwargs):
    """A graph with 1 m cells whose collision radius only blocks real
    obstacles, so tests reason about cells and not about robot size."""
    kwargs.setdefault("meters_per_cell", 1.0)
    kwargs.setdefault("collision_radius", 0.5)
    return GridGraph.from_rows(rows, **kwargs)


def blank(rows, cols):
    return [[0] * cols for _ in range(rows)]


def wall_with_gap(gap_row=None):
    rows = blank(5, 5)
    for r in range(5):
        rows[r][2] = 1
    if gap_row is not None:
        rows[gap_row][2] = 0
    return rows


# --- graph ---

def test_index_round_trip():
    g = make(blank(4, 6))
    assert g.idx_to_cell(g.cell_to_idx(3, 5)) == (3, 5)


def test_world_conversion_round_trip():
    g = make(blank(10, 10), meters_per_cell=0.05, origin_x=-1.0, origin_y=-2.0)
    x, y = g.cell_to_pos(3, 7)
    assert g.pos_to_cell(x, y) == (3, 7)


def test_wrong_odds_length_is_rejected():
    try:
        GridGraph(3, 3, cell_odds=[0] * 8)
    except ValueError:
        return
    raise AssertionError("expected a ValueError")


def test_obstacle_distances_in_meters():
    rows = blank(1, 5)
    rows[0][0] = 1
    g = make(rows, meters_per_cell=0.1)
    assert [round(d, 6) for d in g.obstacle_distances] == [0, 0.1, 0.2, 0.3, 0.4]


def test_collision_radius_blocks_cells_near_walls():
    rows = blank(1, 6)
    rows[0][0] = 1
    g = make(rows, meters_per_cell=0.1, collision_radius=0.25)
    assert [g.check_collision(0, j) for j in range(6)] == \
        [True, True, True, False, False, False]


def test_out_of_bounds_collides():
    g = make(blank(3, 3))
    assert g.check_collision(-1, 0) and g.check_collision(0, 3)


def test_neighbors_are_eight_connected():
    g = make(blank(3, 3))
    assert len(g.find_neighbors(1, 1)) == 8
    assert len(g.find_neighbors(0, 0)) == 3


def test_no_corner_cutting():
    # Two obstacles touching at a corner: the robot must not slip between.
    g = make([[0, 1], [1, 0]])
    assert g.find_neighbors(0, 0) == []


# --- searches (run for both) ---

SEARCHES = [breadth_first_search, a_star_search]


def test_straight_line():
    for search in SEARCHES:
        assert search(make(blank(1, 6)), (0, 0), (0, 5)) == \
            [(0, c) for c in range(6)]


def test_start_equals_goal():
    for search in SEARCHES:
        assert search(make(blank(3, 3)), (1, 1), (1, 1)) == [(1, 1)]


def test_goes_around_wall_through_gap():
    for search in SEARCHES:
        g = make(wall_with_gap(gap_row=4))
        path = search(g, (0, 0), (0, 4))
        assert path and (4, 2) in path
        assert not any(g.check_collision(*cell) for cell in path)


def test_no_path_returns_empty_list():
    for search in SEARCHES:
        assert search(make(wall_with_gap()), (0, 0), (0, 4)) == []


def test_blocked_start_or_goal_returns_empty_list():
    rows = blank(3, 3)
    rows[0][0] = 1
    rows[2][2] = 1
    for search in SEARCHES:
        g = make(rows)
        assert search(g, (0, 0), (1, 1)) == []
        assert search(g, (1, 1), (2, 2)) == []


def test_out_of_bounds_returns_empty_list():
    for search in SEARCHES:
        assert search(make(blank(3, 3)), (0, 0), (5, 5)) == []


def test_visited_cells_are_recorded():
    for search in SEARCHES:
        g = make(blank(4, 4))
        search(g, (0, 0), (3, 3))
        assert g.visited_cells and g.visited_cells[0] == (0, 0)


def test_a_star_is_shortest_against_octile_distance():
    g = make(blank(8, 12))
    path = a_star_search(g, (0, 0), (7, 11))
    assert abs(path_length(path) - octile_distance((0, 0), (7, 11))) < 1e-9


def test_bfs_uses_fewest_steps():
    g = make(blank(5, 5))
    assert len(breadth_first_search(g, (0, 0), (4, 4))) == 5


def test_a_star_beats_bfs_on_length_when_they_differ():
    g = make(blank(6, 6))
    assert path_length(a_star_search(g, (0, 0), (5, 2))) <= \
        path_length(breadth_first_search(g, (0, 0), (5, 2))) + 1e-9


def test_clearance_keeps_away_from_wall():
    rows = blank(7, 9)
    for c in range(9):
        rows[0][c] = 1
    g = make(rows)
    plain = a_star_search(g, (1, 0), (1, 8))
    safe = a_star_search(g, (1, 0), (1, 8), clearance=3.0, clearance_weight=2.0)
    # Skip the start and goal cells themselves: both sit on row 1 by design.
    assert min(r for r, _ in plain[1:-1]) == 1
    assert min(r for r, _ in safe[1:-1]) > 1


def test_clearance_still_uses_a_tight_gap():
    g = make(wall_with_gap(gap_row=2))
    path = a_star_search(g, (2, 0), (2, 4), clearance=3.0)
    assert path and (2, 2) in path


# --- robot_plan_path ---

def test_waypoints_keep_only_turns_and_use_exact_goal():
    g = make(blank(5, 5))
    path = [(0, 0), (0, 1), (0, 2), (1, 2), (2, 2)]
    waypoints = path_to_waypoints(g, path, goal_pos=(2.4, 2.1), goal_theta=1.5)
    assert waypoints == [[0.5, 0.5], [2.5, 0.5], [2.4, 2.1, 1.5]]


def test_waypoints_without_theta_are_x_y_pairs():
    g = make(blank(3, 3))
    waypoints = path_to_waypoints(g, [(0, 0), (0, 1)])
    assert all(len(w) == 2 for w in waypoints)


def test_empty_path_gives_no_waypoints():
    assert path_to_waypoints(make(blank(3, 3)), []) == []


def test_plan_converts_positions_to_cells():
    g = make(blank(5, 5))
    path = plan(g, (0.5, 0.5), (4.5, 4.5))
    assert path[0] == (0, 0) and path[-1] == (4, 4)


def test_load_map_reads_assumed_format():
    text = "-1.0 -2.0 3 2 0.5\n0 0 1\n0 0 0\n"
    with tempfile.NamedTemporaryFile("w", suffix=".map", delete=False) as f:
        f.write(text)
        name = f.name
    try:
        g = load_map(name)
    finally:
        os.remove(name)
    assert (g.width, g.height, g.origin_x, g.origin_y) == (3, 2, -1.0, -2.0)
    assert g.meters_per_cell == 0.5 and g.is_occupied(0, 2)


def test_load_map_rejects_wrong_size():
    with tempfile.NamedTemporaryFile("w", suffix=".map", delete=False) as f:
        f.write("0 0 3 3 0.05\n0 0 0\n")
        name = f.name
    try:
        load_map(name)
    except ValueError:
        return
    finally:
        os.remove(name)
    raise AssertionError("expected a ValueError")


if __name__ == "__main__":
    tests = [(n, f) for n, f in sorted(globals().items())
             if n.startswith("test_") and callable(f)]
    for name, fn in tests:
        fn()
        print(f"ok   {name}")
    print(f"\n{len(tests)} tests passed")
