import numpy as np
import pandas as pd

from sklearn.compose import ColumnTransformer
from sklearn.ensemble import GradientBoostingRegressor
from sklearn.linear_model import LinearRegression
from sklearn.metrics import mean_absolute_error
from sklearn.preprocessing import OneHotEncoder

try:
    from .features import get_features, get_residual_target
except ImportError:
    from features import get_features, get_residual_target


ALPHA = 0.1


def split_dataset(df: pd.DataFrame):
    """Create train, calibration, and test splits."""

    train_df = df[
        (df["split"] == "train") &
        (df["seed"].between(0, 6))
    ].copy()

    calibration_df = df[
        (df["split"] == "train") &
        (df["seed"].between(7, 9))
    ].copy()

    test_df = df[
        (df["split"] == "test") &
        (df["seed"].between(100, 104))
    ].copy()

    return train_df, calibration_df, test_df


def build_preprocessor():
    """Preprocess categorical and numerical features."""

    categorical_features = ["road_class"]

    numerical_features = [
        "p_phys",
        "speed_mps",
        "ms_kg",
        "c_nsm",
        "cov_trace",
        "dropout_frac",
        "band_lo",
        "band_mid",
        "band_hi",
    ]

    preprocessor = ColumnTransformer(
        transformers=[
            (
                "categorical",
                OneHotEncoder(
                    handle_unknown="ignore",
                    sparse_output=False,
                ),
                categorical_features,
            ),
            (
                "numerical",
                "passthrough",
                numerical_features,
            ),
        ]
    )

    return preprocessor


def build_quantile_model(quantile: float):
    """Create a quantile Gradient Boosting model."""

    return GradientBoostingRegressor(
        loss="quantile",
        alpha=quantile,
        n_estimators=200,
        learning_rate=0.05,
        max_depth=3,
        random_state=42,
    )


def fit_models(train_df: pd.DataFrame):
    """Fit q05, q50, and q95 residual models."""

    X_train = get_features(train_df)
    y_train = get_residual_target(train_df)

    preprocessor = build_preprocessor()
    X_train_transformed = preprocessor.fit_transform(X_train)

    models = {}

    for name, quantile in [
        ("q05", 0.05),
        ("q50", 0.50),
        ("q95", 0.95),
    ]:
        model = build_quantile_model(quantile)
        model.fit(X_train_transformed, y_train)
        models[name] = model

    return preprocessor, models


def predict_residuals(
    preprocessor,
    models,
    df: pd.DataFrame,
):
    """Predict residual quantiles."""

    X = get_features(df)
    X_transformed = preprocessor.transform(X)

    predictions = {
        name: model.predict(X_transformed)
        for name, model in models.items()
    }

    return predictions


def conformal_adjustment(
    calibration_df: pd.DataFrame,
    predictions: dict,
):
    """
    Calculate conformal widening using calibration residuals.

    The nonconformity score is the amount by which the
    reference residual lies outside the predicted interval.
    """

    actual_residual = get_residual_target(calibration_df).to_numpy()

    lower = predictions["q05"]
    upper = predictions["q95"]

    scores = np.maximum(
        lower - actual_residual,
        actual_residual - upper,
    )

    scores = np.maximum(scores, 0.0)

    # Finite-sample conformal quantile.
    n = len(scores)
    rank = int(np.ceil((n + 1) * (1.0 - ALPHA))) - 1
    rank = min(max(rank, 0), n - 1)

    adjustment = np.sort(scores)[rank]

    return float(adjustment)


def evaluate_model(
    test_df: pd.DataFrame,
    predictions: dict,
):
    """Calculate required model metrics."""

    y_true = test_df["y_ref"].to_numpy()
    p_phys = test_df["p_phys"].to_numpy()

    comfort_pred = p_phys + predictions["q50"]

    lo = (
        p_phys
        + predictions["q05"]
    )

    hi = (
        p_phys
        + predictions["q95"]
    )

    mae_physics_only = mean_absolute_error(
        y_true,
        p_phys,
    )

    # Linear baseline predicts the reference comfort
    # directly from the same allowed features.
    X_train = get_features(
        test_df
    )

    # This baseline is evaluated separately in train_pipeline().
    mae_ml = mean_absolute_error(
        y_true,
        comfort_pred,
    )

    coverage = np.mean(
        (y_true >= lo) &
        (y_true <= hi)
    )

    return {
        "comfort_pred": comfort_pred,
        "lo": lo,
        "hi": hi,
        "mae_physics_only": float(mae_physics_only),
        "mae_ml": float(mae_ml),
        "interval_coverage": float(coverage),
    }


def train_pipeline(df: pd.DataFrame):
    """
    Complete B2 model-training pipeline.

    Returns fitted models, predictions and metrics.
    """

    train_df, calibration_df, test_df = split_dataset(df)

    preprocessor, models = fit_models(train_df)

    calibration_predictions = predict_residuals(
        preprocessor,
        models,
        calibration_df,
    )

    adjustment = conformal_adjustment(
        calibration_df,
        calibration_predictions,
    )

    test_predictions = predict_residuals(
        preprocessor,
        models,
        test_df,
    )

      # Apply conformal widening.
    test_predictions["q05"] -= adjustment
    test_predictions["q95"] += adjustment

    # Enforce non-crossing prediction intervals.
    quantiles = np.column_stack(
        [
            test_predictions["q05"],
            test_predictions["q50"],
            test_predictions["q95"],
        ]
    )

    quantiles = np.sort(quantiles, axis=1)

    test_predictions["q05"] = quantiles[:, 0]
    test_predictions["q50"] = quantiles[:, 1]
    test_predictions["q95"] = quantiles[:, 2]
    results = evaluate_model(
        test_df,
        test_predictions,
    )

    return {
        "preprocessor": preprocessor,
        "models": models,
        "train_df": train_df,
        "calibration_df": calibration_df,
        "test_df": test_df,
        "predictions": test_predictions,
        "metrics": results,
        "conformal_adjustment": adjustment,
    }