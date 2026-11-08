import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def plot_predictions(predictions_df, out_dir):
    """
    Plot predicted comfort against reference comfort.
    """
    os.makedirs(out_dir, exist_ok=True)

    plt.figure(figsize=(7, 6))

    plt.scatter(
        predictions_df["y_ref"],
        predictions_df["comfort_pred"],
        alpha=0.5
    )

    min_val = min(
        predictions_df["y_ref"].min(),
        predictions_df["comfort_pred"].min()
    )
    max_val = max(
        predictions_df["y_ref"].max(),
        predictions_df["comfort_pred"].max()
    )

    plt.plot(
        [min_val, max_val],
        [min_val, max_val],
        linestyle="--"
    )

    plt.xlabel("Reference Comfort (y_ref)")
    plt.ylabel("Predicted Comfort")
    plt.title("Predicted vs Reference Comfort")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()

    plt.savefig(
        os.path.join(out_dir, "predicted_vs_actual.png"),
        dpi=200
    )
    plt.close()


def plot_residuals(predictions_df, out_dir):
    """
    Plot residual distribution.
    """
    os.makedirs(out_dir, exist_ok=True)

    residuals = predictions_df["residual"]

    plt.figure(figsize=(7, 5))

    plt.hist(
        residuals,
        bins=30,
        alpha=0.8
    )

    plt.axvline(
        0,
        linestyle="--"
    )

    plt.xlabel("Residual (y_ref - comfort_pred)")
    plt.ylabel("Count")
    plt.title("Residual Distribution")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()

    plt.savefig(
        os.path.join(out_dir, "residual_distribution.png"),
        dpi=200
    )
    plt.close()


def plot_prediction_intervals(predictions_df, out_dir):
    """
    Plot prediction intervals for a subset of test samples.
    """
    os.makedirs(out_dir, exist_ok=True)

    df = predictions_df.reset_index(drop=True)

    # Limit the plot size so it remains readable.
    n = min(len(df), 150)

    df = df.iloc[:n]

    x = np.arange(n)

    plt.figure(figsize=(12, 6))

    plt.fill_between(
        x,
        df["lo"],
        df["hi"],
        alpha=0.25,
        label="90% prediction interval"
    )

    plt.plot(
        x,
        df["comfort_pred"],
        label="Prediction"
    )

    plt.scatter(
        x,
        df["y_ref"],
        s=12,
        label="Reference"
    )

    plt.xlabel("Test Sample")
    plt.ylabel("Comfort")
    plt.title("Comfort Prediction Intervals")
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()

    plt.savefig(
        os.path.join(out_dir, "prediction_intervals.png"),
        dpi=200
    )
    plt.close()


def plot_envelope(ood_results_df, out_dir):
    """
    Plot envelope/OOD detection results.
    """
    os.makedirs(out_dir, exist_ok=True)

    if len(ood_results_df) == 0:
        return

    counts = ood_results_df["flagged"].value_counts()

    labels = ["Supported", "Flagged"]
    values = [
        int(counts.get(False, 0)),
        int(counts.get(True, 0))
    ]

    plt.figure(figsize=(6, 5))

    plt.bar(labels, values)

    plt.ylabel("Number of Samples")
    plt.title("Envelope / OOD Detection")
    plt.grid(axis="y", alpha=0.3)

    plt.tight_layout()

    plt.savefig(
        os.path.join(out_dir, "envelope_ood.png"),
        dpi=200
    )
    plt.close()


def plot_semiactive(ranking, out_dir):
    """
    Plot semi-active comfort predictions for the first few groups.
    """
    os.makedirs(out_dir, exist_ok=True)

    groups = ranking.get("groups", [])

    if not groups:
        return

    # Plot first 10 groups for readability.
    groups = groups[:10]

    labels = []
    values = []

    for group in groups:
        group_id = group["group_id"]

        for option in group["options"]:
            labels.append(
                f'{group_id}\n{option["preset"]}'
            )
            values.append(option["comfort_pred"])

    plt.figure(figsize=(12, 6))

    plt.bar(
        np.arange(len(values)),
        values
    )

    plt.xticks(
        np.arange(len(labels)),
        labels,
        rotation=60,
        ha="right"
    )

    plt.ylabel("Predicted Comfort")
    plt.title("Semi-Active Damping Comfort Ranking")
    plt.grid(axis="y", alpha=0.3)
    plt.tight_layout()

    plt.savefig(
        os.path.join(out_dir, "semiactive_ranking.png"),
        dpi=200
    )
    plt.close()