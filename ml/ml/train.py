import argparse
import json
from pathlib import Path

import pandas as pd

from sklearn.linear_model import LinearRegression
from sklearn.metrics import mean_absolute_error

from plots import (
    plot_predictions,
    plot_residuals,
    plot_prediction_intervals,
    plot_envelope,
    plot_semiactive,
)

from features import get_features

from model import train_pipeline

from envelope import (
    fit_envelope,
    evaluate_envelope,
    calculate_ood_detection_rate,
)

from semiactive import (
    rank_groups,
    DEFAULT_DLC_LIMIT,
    DEFAULT_TRAVEL_LIMIT_MM,
)


def calculate_linear_baseline(
    train_df: pd.DataFrame,
    test_df: pd.DataFrame,
):
    """Train the required linear regression baseline."""

    X_train = get_features(train_df)
    X_test = get_features(test_df)

    # Convert categorical road class to numeric dummy variables.
    X_train = pd.get_dummies(
        X_train,
        columns=["road_class"],
        dtype=float,
    )

    X_test = pd.get_dummies(
        X_test,
        columns=["road_class"],
        dtype=float,
    )

    X_test = X_test.reindex(
        columns=X_train.columns,
        fill_value=0.0,
    )

    model = LinearRegression()

    model.fit(
        X_train,
        train_df["y_ref"],
    )

    prediction = model.predict(X_test)

    mae = mean_absolute_error(
        test_df["y_ref"],
        prediction,
    )

    return float(mae)


def build_comfort_predictions(
    result,
    linear_mae,
    fixture=False,
    mock=False,
    test_flagged=None,
):
    """Build the contract-defined comfort prediction rows.

    test_flagged: optional boolean array (one entry per test row)
    from the envelope. True means the row is outside the supported
    envelope, so supported=False. If omitted, all rows are supported.
    """

    test_df = result["test_df"].copy()
    predictions = result["predictions"]

    rows = []

    for i, (_, row) in enumerate(
        test_df.iterrows()
    ):
        comfort_pred = (
            row["p_phys"]
            + predictions["q50"][i]
        )

        lo = (
            row["p_phys"]
            + predictions["q05"][i]
        )

        hi = (
            row["p_phys"]
            + predictions["q95"][i]
        )

        if test_flagged is None:
            supported = True
        else:
            supported = not bool(test_flagged[i])

        rows.append({
            "scenario_id": row["scenario_id"],
            "split": row["split"],
            "p_phys": float(row["p_phys"]),
            "comfort_pred": float(
                comfort_pred
            ),
            "lo": float(lo),
            "hi": float(hi),
            "y_ref": float(row["y_ref"]),
            "residual": float(
                row["y_ref"] - comfort_pred
            ),
            "supported": supported,
        })

    metrics = result["metrics"]

    return {
        "meta": {
            "fixture": fixture,
            "mock": mock,
            "model": "quantile_gradient_boosting_residual",
            "alpha": 0.1,
            "metrics": {
                "mae_physics_only": (
                    metrics["mae_physics_only"]
                ),
                "mae_linear": linear_mae,
                "mae_ml": metrics["mae_ml"],
                "interval_coverage": (
                    metrics["interval_coverage"]
                ),
                "n_test": len(test_df),
            },
        },
        "rows": rows,
    }


def _flag_rate(flagged, mask):
    """Fraction of flagged rows within a mask (0.0 if empty)."""

    if not mask.any():
        return 0.0

    return float(flagged[mask].mean())


def build_envelope_json(
    envelope,
    envelope_results,
    ood_detection_rate,
    source_df,
    fixture=False,
    mock=False,
):
    """Build the contract-defined envelope JSON."""

    features_min = {
        key: float(value)
        for key, value in envelope[
            "feature_min"
        ].items()
    }

    features_max = {
        key: float(value)
        for key, value in envelope[
            "feature_max"
        ].items()
    }

    split = source_df["split"].to_numpy()

    # envelope_results does not contain the original
    # split column. Use source_df to identify OOD rows.
    ood_mask = split == "ood"

    ood_rows = envelope_results.loc[ood_mask]

    ood_results = []

    for _, row in ood_rows.iterrows():
        ood_results.append({
            "scenario_id": row["scenario_id"],
            "flagged": bool(row["flagged"]),
            "reason": row["reason"],
            "all_reasons": str(
                row.get("all_reasons", row["reason"])
            ),
        })

    # False-alarm rates on in-distribution rows. A detector that
    # flags everything would score 1.0 on OOD, so report these.
    flagged = envelope_results["flagged"].to_numpy()

    false_positive_rate = {
        "train": _flag_rate(flagged, split == "train"),
        "test": _flag_rate(flagged, split == "test"),
    }

    return {
        "meta": {
            "fixture": fixture,
            "mock": mock,
        },
        "features": {
            "min": features_min,
            "max": features_max,
        },
        "mahalanobis_threshold": float(
            envelope["mahalanobis_threshold"]
        ),
        "ood_detection_rate": float(
            ood_detection_rate
        ),
        "false_positive_rate": false_positive_rate,
        "ood_results": ood_results,
    }


def build_semiactive_json(
    groups,
    fixture=False,
    mock=False,
    dlc_limit=DEFAULT_DLC_LIMIT,
    travel_limit_mm=DEFAULT_TRAVEL_LIMIT_MM,
):
    """Build the contract-defined semi-active JSON."""

    output_groups = []

    for group in groups:
        output_groups.append({
            "group_id": group["group_id"],
            "options": group["options"],
            "recommended": group[
                "recommended"
            ],
            "note": group["note"],
        })

    return {
        "meta": {
            "fixture": fixture,
            "mock": mock,
            "dlc_limit": dlc_limit,
            "travel_limit_mm": travel_limit_mm,
        },
        "groups": output_groups,
    }


def main():

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--in",
        dest="input_csv",
        required=True,
        help="Input comfort dataset CSV",
    )

    parser.add_argument(
        "--out",
        dest="output_dir",
        required=True,
        help="Output directory",
    )

    parser.add_argument(
        "--mock",
        action="store_true",
        help="Mark outputs as generated from mock/fixture data",
    )

    args = parser.parse_args()

    output_dir = Path(
        args.output_dir
    )

    output_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    # Determine whether this run is using mock data.
    fixture = args.mock
    mock = args.mock

    df = pd.read_csv(
        args.input_csv
    )

    # -------------------------
    # B2: Comfort model
    # -------------------------

    result = train_pipeline(
        df
    )

    train_df = result["train_df"]
    test_df = result["test_df"]

    linear_mae = calculate_linear_baseline(
        train_df,
        test_df,
    )

    # -------------------------
    # B3: Envelope (runs before the predictions are built so that
    # the per-row `supported` flag can be filled in)
    # -------------------------

    envelope = fit_envelope(
        train_df
    )

    envelope_results = evaluate_envelope(
        envelope,
        df,
    )

    ood_detection_rate = (
        calculate_ood_detection_rate(
            df,
            envelope_results,
        )
    )

    envelope_json = build_envelope_json(
        envelope,
        envelope_results,
        ood_detection_rate,
        df,
        fixture=fixture,
        mock=mock,
    )

    # envelope_results has the same row order/index as df,
    # so test_df.index selects the matching rows.
    test_flagged = (
        envelope_results
        .loc[test_df.index, "flagged"]
        .to_numpy()
    )

    comfort_json = build_comfort_predictions(
        result,
        linear_mae,
        fixture=fixture,
        mock=mock,
        test_flagged=test_flagged,
    )

    # -------------------------
    # B4: Semi-active ranking
    # -------------------------

    ranking_input = test_df.copy()

    predictions = result[
        "predictions"
    ]

    ranking_input[
        "comfort_pred"
    ] = (
        ranking_input["p_phys"].to_numpy()
        + predictions["q50"]
    )

    ranking_input["lo"] = (
        ranking_input["p_phys"].to_numpy()
        + predictions["q05"]
    )

    ranking_input["hi"] = (
        ranking_input["p_phys"].to_numpy()
        + predictions["q95"]
    )

    groups = rank_groups(
        ranking_input,
        dlc_limit=DEFAULT_DLC_LIMIT,
        travel_limit_mm=DEFAULT_TRAVEL_LIMIT_MM,
    )

    semiactive_json = (
        build_semiactive_json(
            groups,
            fixture=fixture,
            mock=mock,
            dlc_limit=DEFAULT_DLC_LIMIT,
            travel_limit_mm=DEFAULT_TRAVEL_LIMIT_MM,
        )
    )

    # -------------------------
    # B5: Plots
    # -------------------------

    predictions_df = pd.DataFrame(
        comfort_json["rows"]
    )

    plot_predictions(
        predictions_df,
        output_dir,
    )

    plot_residuals(
        predictions_df,
        output_dir,
    )

    plot_prediction_intervals(
        predictions_df,
        output_dir,
    )

    # The envelope plot needs the split of each row.
    envelope_plot_df = envelope_results.assign(
        split=df["split"].to_numpy()
    )

    plot_envelope(
        envelope_plot_df,
        output_dir,
    )

    plot_semiactive(
        semiactive_json,
        output_dir,
    )

    # -------------------------
    # Write outputs
    # -------------------------

    with open(
        output_dir / "comfort_predictions.json",
        "w",
    ) as f:
        json.dump(comfort_json, f, indent=2)

    with open(
        output_dir / "envelope.json",
        "w",
    ) as f:
        json.dump(envelope_json, f, indent=2)

    with open(
        output_dir / "semiactive_ranking.json",
        "w",
    ) as f:
        json.dump(semiactive_json, f, indent=2)

    # -------------------------
    # Console summary
    # -------------------------

    m = comfort_json["meta"]["metrics"]
    fpr = envelope_json["false_positive_rate"]

    print("Training pipeline complete.")
    print()
    print(f"Input rows: {len(df)}")
    print(f"Test rows: {len(test_df)}")
    print()
    print("Comfort MAE:")
    print(f"  Physics only: {m['mae_physics_only']:.6f}")
    print(f"  Linear:       {m['mae_linear']:.6f}")
    print(f"  ML:           {m['mae_ml']:.6f}")
    print(f"  Coverage:     {m['interval_coverage']:.3f}")
    print()
    print(f"OOD detection rate: {ood_detection_rate:.3f}")
    print(
        f"False-positive rate: train {fpr['train']:.3f}, "
        f"test {fpr['test']:.3f}"
    )
    print(
        f"Test rows marked unsupported: "
        f"{int(test_flagged.sum())} of {len(test_flagged)}"
    )
    print()
    print("Created:")
    print(f"  {output_dir / 'comfort_predictions.json'}")
    print(f"  {output_dir / 'envelope.json'}")
    print(f"  {output_dir / 'semiactive_ranking.json'}")


if __name__ == "__main__":
    main()