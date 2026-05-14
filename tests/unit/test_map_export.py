"""Unit tests for warehouse.generators.map_export."""

from __future__ import annotations


def test_rasterize_single_obstacle_box():
    from warehouse.generators.map_export import rasterize_obstacles

    # 10m × 10m world, 5cm/px → 200×200 grid
    obstacles = [{"x_min": 4.0, "x_max": 5.0, "y_min": 4.0, "y_max": 5.0}]
    grid = rasterize_obstacles(
        world_w_m=10.0,
        world_h_m=10.0,
        resolution_m_per_px=0.05,
        obstacles=obstacles,
    )

    assert grid.shape == (200, 200)
    # 1m × 1m obstacle = 20×20 cells = 400 occupied px
    assert int((grid == 100).sum()) == 400
    # All other cells are free (0)
    assert int((grid == 0).sum()) == 200 * 200 - 400


def test_rasterize_multiple_obstacles():
    from warehouse.generators.map_export import rasterize_obstacles

    obstacles = [
        {"x_min": 0.0, "x_max": 1.0, "y_min": 0.0, "y_max": 1.0},
        {"x_min": 2.0, "x_max": 3.0, "y_min": 2.0, "y_max": 3.0},
    ]
    grid = rasterize_obstacles(
        world_w_m=5.0,
        world_h_m=5.0,
        resolution_m_per_px=0.1,
        obstacles=obstacles,
    )
    assert grid.shape == (50, 50)
    assert int((grid == 100).sum()) == 2 * 10 * 10  # two 1m² obstacles at 10 px/m


def test_rasterize_obstacle_outside_world_clipped():
    from warehouse.generators.map_export import rasterize_obstacles

    obstacles = [{"x_min": 9.0, "x_max": 11.0, "y_min": 4.0, "y_max": 5.0}]
    grid = rasterize_obstacles(
        world_w_m=10.0, world_h_m=10.0, resolution_m_per_px=0.05, obstacles=obstacles
    )
    # Clipped to x∈[9,10] = 1m wide = 20 cells. y is 1m = 20 cells. 400 px.
    assert int((grid == 100).sum()) == 400
