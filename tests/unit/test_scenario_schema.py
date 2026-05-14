from scenarios.schema import load_scenario


def test_load_smoke_scenario(tmp_path):
    p = tmp_path / "s.yaml"
    p.write_text(
        """
name: smoke
duration_s: 60
orders:
  - {id: o1, shelf_xy: [4.0, 8.0], arrival_t: 0.0}
"""
    )
    s = load_scenario(p)
    assert s.fleet_size == 6
    assert len(s.orders) == 1
