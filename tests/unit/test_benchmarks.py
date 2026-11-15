"""
Unit Tests for Cross-Platform Vehicle Benchmarking and Validation Suite.
"""

import json

from Localization.core.dynamics import F1TENTH_PARAMS
from scripts.benchmark_vehicles import (
    VehicleBenchmarkResult,
    execute_fleet_benchmarks,
    generate_benchmark_circuit,
    run_single_vehicle_benchmark,
)


def test_generate_benchmark_circuit():
    """Verify generated benchmark track geometry is closed and well-formed."""
    circuit = generate_benchmark_circuit()
    assert circuit.total_length > 400.0
    assert circuit.is_closed
    assert len(circuit.centerline) == 200
    assert len(circuit.headings) == 200


def test_single_vehicle_benchmark():
    """Verify single vehicle benchmark simulation generates consistent metrics."""
    circuit = generate_benchmark_circuit()
    result = run_single_vehicle_benchmark(
        name="F1TENTH-Test",
        params=F1TENTH_PARAMS,
        hd_map=circuit,
        sim_duration_s=1.5,
        dt=0.05,
    )

    assert isinstance(result, VehicleBenchmarkResult)
    assert result.platform_name == "F1TENTH-Test"
    assert result.mass_kg == F1TENTH_PARAMS.mass
    assert result.top_speed_kph > 10.0
    assert result.max_lateral_g >= 0.0
    assert result.rmse_cross_track_m < 2.0
    # Model validation error must be sub-decimeter on short rollout
    assert result.validation_rmse_pos_m < 0.10


def test_execute_fleet_benchmarks_and_json_output(tmp_path):
    """Verify multi-platform comparison exports valid machine-readable JSON."""
    out_json = tmp_path / "fleet_benchmarks.json"
    results = execute_fleet_benchmarks(output_json_path=out_json)

    assert len(results) == 3
    assert out_json.is_file()

    with open(out_json, encoding="utf-8") as f:
        data = json.load(f)

    assert len(data) == 3
    platform_names = [d["platform_name"] for d in data]
    assert any("F1TENTH" in p for p in platform_names)
    assert any("Go-Kart" in p for p in platform_names)
    assert any("AV-21" in p for p in platform_names)

    # Dallara AV-21 top speed must exceed 1:10 scaled F1TENTH
    f1_spd = next(d["top_speed_kph"] for d in data if "F1TENTH" in d["platform_name"])
    av_spd = next(d["top_speed_kph"] for d in data if "AV-21" in d["platform_name"])
    assert av_spd > f1_spd
