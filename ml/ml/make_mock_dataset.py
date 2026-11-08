import argparse
from pathlib import Path

import numpy as np
import pandas as pd


# Contract-defined values
ROAD_CLASSES = ["A", "B", "C"]
SPEEDS = [10.0, 20.0, 30.0]
LOADS = [300.0, 440.0, 580.0]
PRESETS = {
    "soft": 1800.0,
    "medium": 3000.0,
    "firm": 4500.0,
}
DROPOUTS = [0.0, 5.0, 20.0]

TRAIN_SEEDS = range(10)
TEST_SEEDS = range(100, 105)


def make_row(
    scenario_id,
    split,
    road_class,
    speed_mps,
    ms_kg,
    preset,
    c_nsm,
    dropout_pct,
    seed,
):
    rng = np.random.default_rng(seed)

    # Physics-like baseline.
    road_factor = {
        "A": 0.85,
        "B": 1.00,
        "C": 1.20,
        "D": 1.35,
    }[road_class]

    speed_factor = 1.0 + 0.015 * speed_mps
    load_factor = ms_kg / 440.0

    p_phys = (
        0.8
        + road_factor
        * speed_factor
        * load_factor
        * (1.0 + 0.00008 * c_nsm)
    )

    # Reference comfort value with deterministic residual structure.
    residual = (
        0.08 * np.sin(speed_mps / 10.0)
        + 0.05 * (load_factor - 1.0)
        - 0.00001 * (c_nsm - 3000.0)
        + rng.normal(0.0, 0.025)
    )

    y_ref = p_phys + residual

    # Sensor / uncertainty-related features.
    cov_trace = (
        0.02
        + 0.0005 * speed_mps
        + 0.00001 * ms_kg
        + 0.001 * (dropout_pct / 100.0)
    )

    dropout_frac = dropout_pct / 100.0

    band_lo = max(0.0, p_phys - 0.08 - 0.02 * dropout_frac)
    band_mid = p_phys
    band_hi = p_phys + 0.08 + 0.02 * dropout_frac

    # Constraint-related quantities.
    rms_tyre_force_n = (
        850.0
        + 18.0 * speed_mps
        + 0.35 * ms_kg
        + 0.02 * c_nsm
        + {"A": 0.0, "B": 120.0, "C": 250.0, "D": 350.0}[road_class]
        + rng.normal(0.0, 20.0)
    )

    max_travel_mm = (
        45.0
        + 0.8 * speed_mps
        + 0.015 * (ms_kg - 300.0)
        + {"soft": 10.0, "medium": 0.0, "firm": -6.0}[preset]
        + rng.normal(0.0, 2.0)
    )

    bottom_out = bool(max_travel_mm > 120.0)

    return {
        "scenario_id": scenario_id,
        "split": split,
        "road_class": road_class,
        "speed_mps": speed_mps,
        "ms_kg": ms_kg,
        "preset": preset,
        "c_nsm": c_nsm,
        "dropout_pct": dropout_pct,
        "seed": seed,
        "p_phys": p_phys,
        "y_ref": y_ref,
        "cov_trace": cov_trace,
        "dropout_frac": dropout_frac,
        "band_lo": band_lo,
        "band_mid": band_mid,
        "band_hi": band_hi,
        "rms_tyre_force_n": rms_tyre_force_n,
        "max_travel_mm": max_travel_mm,
        "bottom_out": bottom_out,
    }


def generate_dataset():
    rows = []

    # Training scenarios
    for road_class in ROAD_CLASSES:
        for speed_mps in SPEEDS:
            for ms_kg in LOADS:
                for preset, c_nsm in PRESETS.items():
                    for dropout_pct in DROPOUTS:
                        scenario_id = (
                            f"{road_class}_"
                            f"{int(speed_mps)}_"
                            f"{int(ms_kg)}_"
                            f"{preset}_"
                            f"{int(dropout_pct)}"
                        )

                        for seed in TRAIN_SEEDS:
                            rows.append(
                                make_row(
                                    scenario_id,
                                    "train",
                                    road_class,
                                    speed_mps,
                                    ms_kg,
                                    preset,
                                    c_nsm,
                                    dropout_pct,
                                    seed,
                                )
                            )

                        for seed in TEST_SEEDS:
                            rows.append(
                                make_row(
                                    scenario_id,
                                    "test",
                                    road_class,
                                    speed_mps,
                                    ms_kg,
                                    preset,
                                    c_nsm,
                                    dropout_pct,
                                    seed,
                                )
                            )

    # OOD scenarios are only needed for envelope testing.
    ood_cases = [
        ("D", 20.0, 440.0, "medium", 3000.0, 0.0, 200),
        ("A", 40.0, 440.0, "medium", 3000.0, 0.0, 201),
        ("A", 20.0, 700.0, "medium", 3000.0, 0.0, 202),
        ("A", 20.0, 440.0, "medium", 3000.0, 30.0, 203),
    ]

    for (
        road_class,
        speed_mps,
        ms_kg,
        preset,
        c_nsm,
        dropout_pct,
        seed,
    ) in ood_cases:
        rows.append(
            make_row(
                f"OOD_{road_class}_{int(speed_mps)}_{int(ms_kg)}",
                "ood",
                road_class,
                speed_mps,
                ms_kg,
                preset,
                c_nsm,
                dropout_pct,
                seed,
            )
        )

    return pd.DataFrame(rows)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--out",
        default="out/comfort_dataset.csv",
        help="Output CSV path",
    )
    args = parser.parse_args()

    output_path = Path(args.out)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    df = generate_dataset()
    df.to_csv(output_path, index=False)

    print(f"Created: {output_path}")
    print(f"Rows: {len(df)}")
    print(f"Columns: {len(df.columns)}")
    print("\nSplit counts:")
    print(df["split"].value_counts())
    print("\nColumns:")
    print(list(df.columns))


if __name__ == "__main__":
    main()