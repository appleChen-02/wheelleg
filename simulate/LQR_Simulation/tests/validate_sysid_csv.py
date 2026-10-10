"""Validate complete simulation captures against USB CSV schema without serial/scipy dependencies."""
import argparse
import ast
import csv
import math
from pathlib import Path


def usb_schema():
    logger = Path(__file__).resolve().parents[3] / "上位机" / "usb_data_logger.py"
    wanted = {
        "SYS_ID_COMMON_COLUMNS", "SYS_ID_STATE_COLUMNS", "SYS_ID_INPUT_COLUMNS",
        "SYS_ID_MOTOR_TORQUE_COLUMNS", "SYS_ID_12_COLUMNS", "SYS_ID_REFERENCE_COLUMNS",
    }
    namespace = {}
    # Only evaluate schema assignments; importing the logger would require pyserial.
    for node in ast.parse(logger.read_text(encoding="utf-8")).body:
        if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id in wanted for t in node.targets):
            exec(compile(ast.Module(body=[node], type_ignores=[]), str(logger), "exec"), namespace)
    return namespace


def validate(directory, schema):
    metadata = dict(line.split("=", 1) for line in (directory / "simulation_experiment.txt").read_text().splitlines())
    assert metadata["phase"] == "complete", "Capture did not finish"
    tables = []
    for filename, key in [("system_identification_12.csv", "SYS_ID_12_COLUMNS"),
                          ("system_identification_reference.csv", "SYS_ID_REFERENCE_COLUMNS")]:
        with (directory / filename).open(newline="", encoding="utf-8") as stream:
            reader = csv.DictReader(stream)
            assert reader.fieldnames == ["host_time_s"] + schema[key], f"USB schema differs: {filename}"
            rows = [{k: float(v) for k, v in row.items()} for row in reader]
        assert rows and all(math.isfinite(v) for row in rows for v in row.values()), "Missing or non-finite data"
        tables.append(rows)
    samples, references = tables
    assert len(samples) == int(metadata["rows"])
    for prev, current in zip(samples, samples[1:]):
        assert current["control_tick_ms"] - prev["control_tick_ms"] == 4, "Nonuniform sample interval"
        assert current["sequence"] - prev["sequence"] == 2, "Sequence gap"
    reference_columns = [k for k in references[0] if k.startswith("reference_")]
    for row in references:
        assert all(row[k] == references[0][k] for k in reference_columns), "Reference changed during capture"
    for previous, current in zip(references, references[1:]):
        assert abs(current["host_time_s"] - previous["host_time_s"] - 1) < 1e-6

    channels = schema["SYS_ID_INPUT_COLUMNS"][:5]
    start = float(metadata["excitation_start_s"])
    end = float(metadata["planned_excitation_end_s"])
    enabled = int(metadata["excitation_enable"]) and int(metadata["excitation_mode"]) != 0
    mask = int(metadata["input_mask"])
    hold = int(metadata["prbs_hold_samples"])
    mode = int(metadata["excitation_mode"])
    assert samples[0]["control_tick_ms"] / 1000 <= start - float(metadata["pre_seconds"]) + 0.0041
    assert samples[-1]["control_tick_ms"] / 1000 >= end + float(metadata["post_seconds"]) - 0.0041
    max_residual = max_actual_difference = max_wave_error = 0.0
    seeds = [int(metadata["seed_" + ch]) for ch in channels]
    prbs = []
    if enabled and mode == 2:
        for _ in range(math.ceil((end - start) / 0.002 / hold) + 1):
            prbs.append([1 if seed & 1 else -1 for seed in seeds])
            seeds = [(seed >> 1) ^ (0xB400 if seed & 1 else 0) for seed in seeds]
    for row in samples:
        time = row["control_tick_ms"] / 1000
        active = enabled and start - 1e-8 <= time < end - 1e-8
        assert bool(int(row["status_flags"]) & 2) == bool(active), "Incorrect excitation-active flag"
        assert int(row["status_flags"]) & 0xC1 == 0xC1, "Invalid state/input/reference flags"
        assert row["motor_saturation_mask"] == 0, "Unexpected saturation in validation capture"
        for i, ch in enumerate(channels):
            residual = row["command_" + ch] - row["controller_" + ch] - row["excitation_" + ch] - references[0]["equilibrium_" + ch]
            max_residual = max(max_residual, abs(residual))
            max_actual_difference = max(max_actual_difference, abs(row["actual_" + ch] - row["command_" + ch]))
            expected = 0.0
            if active and mask & (1 << i):
                amplitude = float(metadata["amplitude_" + ch])
                tau = time - start
                if mode == 1:
                    f0, f1, duration = [float(metadata[k]) for k in ("sweep_f0_hz", "sweep_f1_hz", "sweep_duration_s")]
                    law = metadata.get("sweep_law", "linear")  # Historical captures used linear sweeps.
                    if law == "logarithmic":
                        log_ratio = math.log1p((f1 - f0) / f0)
                        cycles = f0 * duration / log_ratio * math.expm1(log_ratio * tau / duration)
                    else:
                        assert law == "linear", f"Unknown sweep law: {law}"
                        cycles = f0 * tau + 0.5 * (f1 - f0) / duration * tau * tau
                    expected = amplitude * math.sin(2 * math.pi * cycles)
                else:
                    expected = amplitude * prbs[round(tau / 0.002) // hold][i]
            max_wave_error = max(max_wave_error, abs(row["excitation_" + ch] - expected))
        assert abs(row["W0_feedback_torque"] - row["actual_left_wheel_t"]) < 1e-10
        assert abs(row["W1_feedback_torque"] + row["actual_right_wheel_t"]) < 1e-10
        assert abs(row["Tail_feedback_torque"] + row["actual_tail_t"]) < 1e-10
    assert max_residual < 1e-10, f"Input decomposition residual: {max_residual}"
    assert max_actual_difference < 1e-8, f"Applied generalized torque differs: {max_actual_difference}"
    assert max_wave_error < 1e-7, f"Excitation waveform differs: {max_wave_error}"
    print(f"PASS {directory}: {len(samples)} rows, 250 Hz, schema matches USB, "
          f"residual={max_residual:.2g}, actual error={max_actual_difference:.2g}, wave error={max_wave_error:.2g}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directories", type=Path, nargs="+")
    args = parser.parse_args()
    schema = usb_schema()
    for directory in args.directories:
        validate(directory, schema)
