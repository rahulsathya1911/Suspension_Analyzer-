import json
from pathlib import Path

import pandas as pd

from ml.features import (
    FEATURE_COLUMNS,
    FORBIDDEN_COLUMNS,
    get_features,
    validate_dataset,
)
from ml.model import train_pipeline
from ml.envelope import (
    fit_envelope,
    evaluate_envelope,
    calculate_ood_detection_rate,
)
from ml.semiactive import rank_groups


ROOT = Path(__file__).resolve().parents[2]
DATASET = ROOT / "out" / "comfort_dataset.csv"


def load_dataset():
    return pd.read_csv(DATASET)


def test_dataset_schema():
    df = load_dataset()

    validate_dataset(df)

    assert list(get_features(df).columns) == FEATURE_COLUMNS

    for column in FORBIDDEN_COLUMNS:
        assert column not in FEATURE_COLUMNS


def test_model_training():
    df = load_dataset()

    result = train_pipeline(df)

    assert len(result["train_df"]) > 0
    assert len(result["test_df"]) > 0

    predictions = result["predictions"]

    assert len(predictions["q05"]) == len(result["test_df"])
    assert len(predictions["q50"]) == len(result["test_df"])
    assert len(predictions["q95"]) == len(result["test_df"])


def test_prediction_intervals():
    df = load_dataset()

    result = train_pipeline(df)

    predictions = result["predictions"]

    assert (predictions["q05"] <= predictions["q50"]).all()
    assert (predictions["q50"] <= predictions["q95"]).all()


def test_envelope_detects_ood():
    df = load_dataset()

    train_df = df[
        df["split"] == "train"
    ].copy()

    envelope = fit_envelope(train_df)

    results = evaluate_envelope(
        envelope,
        df,
    )

    ood_rate = calculate_ood_detection_rate(
        df,
        results,
    )

    assert ood_rate > 0.0

    ood_rows = results[
        df["split"].to_numpy() == "ood"
    ]

    assert ood_rows["flagged"].all()


def test_semiactive_ranking():
    df = load_dataset()

    result = train_pipeline(df)

    test_df = result["test_df"].copy()
    predictions = result["predictions"]

    test_df["comfort_pred"] = (
        test_df["p_phys"].to_numpy()
        + predictions["q50"]
    )

    test_df["lo"] = (
        test_df["p_phys"].to_numpy()
        + predictions["q05"]
    )

    test_df["hi"] = (
        test_df["p_phys"].to_numpy()
        + predictions["q95"]
    )

    groups = rank_groups(test_df)

    assert len(groups) > 0

    for group in groups:
        assert "group_id" in group
        assert "options" in group
        assert "recommended" in group
        assert "note" in group

        assert len(group["options"]) == 3


def test_output_json_files():
    output_dir = ROOT / "out"

    required_files = [
        "comfort_predictions.json",
        "envelope.json",
        "semiactive_ranking.json",
    ]

    for filename in required_files:
        path = output_dir / filename

        assert path.exists()

        with open(path, "r") as f:
            data = json.load(f)

        assert isinstance(data, dict)
        assert "meta" in data