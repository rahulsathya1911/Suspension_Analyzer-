import numpy as np
import pandas as pd

from scipy.spatial.distance import mahalanobis

try:
    from .features import get_features
except ImportError:
    from features import get_features


def fit_envelope(train_df: pd.DataFrame):
    """
    Build the supported operating envelope from training data.

    The envelope consists of:
    - feature-wise min/max limits
    - Mahalanobis distance threshold
    - training mean/covariance information
    - the set of road classes seen in training
    """

    X = get_features(train_df).copy()

    # One-hot encode road class so that the Mahalanobis
    # calculation can operate on numeric features.
    X_encoded = pd.get_dummies(
        X,
        columns=["road_class"],
        dtype=float,
    )

    # Keep the exact column order for future data.
    feature_columns = list(X_encoded.columns)

    mean_vector = X_encoded.mean().to_numpy()

    covariance = np.cov(
        X_encoded.to_numpy(),
        rowvar=False,
    )

    # Small regularization prevents singular covariance matrices.
    covariance += np.eye(covariance.shape[0]) * 1e-8

    inverse_covariance = np.linalg.pinv(covariance)

    training_values = X_encoded.to_numpy()

    distances = np.array([
        mahalanobis(
            row,
            mean_vector,
            inverse_covariance,
        )
        for row in training_values
    ])

    # 99th percentile defines the supported Mahalanobis envelope.
    mahalanobis_threshold = float(
        np.percentile(distances, 99)
    )

    # Feature-wise range limits.
    feature_min = X_encoded.min().to_dict()
    feature_max = X_encoded.max().to_dict()

    return {
        "feature_columns": feature_columns,
        "feature_min": feature_min,
        "feature_max": feature_max,
        "mean_vector": mean_vector,
        "inverse_covariance": inverse_covariance,
        "mahalanobis_threshold": mahalanobis_threshold,
        "road_classes": sorted(
            train_df["road_class"].unique().tolist()
        ),
    }


def encode_features(
    df: pd.DataFrame,
    feature_columns,
):
    """Encode features using the same representation as training."""

    X = get_features(df).copy()

    X_encoded = pd.get_dummies(
        X,
        columns=["road_class"],
        dtype=float,
    )

    # Add missing columns as zero.
    for column in feature_columns:
        if column not in X_encoded.columns:
            X_encoded[column] = 0.0

    # Remove unexpected columns and preserve training order.
    X_encoded = X_encoded.reindex(
        columns=feature_columns,
        fill_value=0.0,
    )

    return X_encoded


def collect_reasons(
    row,
    envelope,
):
    """Return every envelope violation for one encoded row."""

    feature_columns = envelope["feature_columns"]

    values = row.to_numpy(dtype=float)

    reasons = []

    # Range checks (all features, not just the first violation).
    for index, feature in enumerate(feature_columns):
        value = values[index]
        minimum = envelope["feature_min"][feature]
        maximum = envelope["feature_max"][feature]

        if value < minimum or value > maximum:
            reasons.append(f"range:{feature}")

    # Mahalanobis check.
    distance = mahalanobis(
        values,
        envelope["mean_vector"],
        envelope["inverse_covariance"],
    )

    if distance > envelope["mahalanobis_threshold"]:
        reasons.append("mahalanobis")

    return reasons


def check_row(
    row,
    envelope,
):
    """Check one row against the supported envelope.

    Returns (supported, first_reason). Signature unchanged.
    """

    reasons = collect_reasons(row, envelope)

    if reasons:
        return False, reasons[0]

    return True, "none"


def evaluate_envelope(
    envelope,
    df: pd.DataFrame,
):
    """
    Evaluate all rows against the envelope.

    Returns a DataFrame containing:
    - scenario_id
    - flagged
    - reason       (first violation, or "none")
    - all_reasons  (every violation, ";"-separated)
    """

    X_encoded = encode_features(
        df,
        envelope["feature_columns"],
    )

    known_road_classes = set(
        envelope.get("road_classes", [])
    )

    results = []

    for index, row in X_encoded.iterrows():
        reasons = collect_reasons(
            row,
            envelope,
        )

        # A road class never seen in training is its own reason.
        # (One-hot encoding silently drops it, so it needs an
        # explicit check.)
        road_class = df.loc[index, "road_class"]

        if known_road_classes and road_class not in known_road_classes:
            reasons.insert(
                0,
                f"unseen_road_class:{road_class}",
            )

        results.append({
            "scenario_id": df.loc[index, "scenario_id"],
            "flagged": bool(reasons),
            "reason": reasons[0] if reasons else "none",
            "all_reasons": ";".join(reasons) if reasons else "none",
        })

    return pd.DataFrame(results)


def calculate_ood_detection_rate(
    df: pd.DataFrame,
    results: pd.DataFrame,
):
    """Calculate the fraction of OOD rows correctly flagged."""

    ood_mask = df["split"].to_numpy() == "ood"

    if not np.any(ood_mask):
        return 0.0

    flagged = results["flagged"].to_numpy()

    return float(
        np.mean(flagged[ood_mask])
    )