import pandas as pd


FEATURE_COLUMNS = [
    "p_phys",
    "road_class",
    "speed_mps",
    "ms_kg",
    "c_nsm",
    "cov_trace",
    "dropout_frac",
    "band_lo",
    "band_mid",
    "band_hi",
]

TARGET_COLUMN = "y_ref"

FORBIDDEN_COLUMNS = [
    "y_ref",
    "rms_tyre_force_n",
    "max_travel_mm",
    "bottom_out",
]


def validate_dataset(df: pd.DataFrame) -> None:
    """Validate that the dataset contains the required contract columns."""

    required_columns = [
        "scenario_id",
        "split",
        "road_class",
        "speed_mps",
        "ms_kg",
        "preset",
        "c_nsm",
        "dropout_pct",
        "seed",
        "p_phys",
        "y_ref",
        "cov_trace",
        "dropout_frac",
        "band_lo",
        "band_mid",
        "band_hi",
        "rms_tyre_force_n",
        "max_travel_mm",
        "bottom_out",
    ]

    missing = [
        column for column in required_columns
        if column not in df.columns
    ]

    if missing:
        raise ValueError(
            f"Dataset is missing required columns: {missing}"
        )


def get_features(df: pd.DataFrame) -> pd.DataFrame:
    """Return only the features allowed by the ML contract."""

    validate_dataset(df)

    return df[FEATURE_COLUMNS].copy()


def get_target(df: pd.DataFrame) -> pd.Series:
    """Return the reference comfort target."""

    validate_dataset(df)

    return df[TARGET_COLUMN].copy()


def get_residual_target(df: pd.DataFrame) -> pd.Series:
    """Return the residual target: y_ref - p_phys."""

    validate_dataset(df)

    return df["y_ref"] - df["p_phys"]