import numpy as np
import pandas as pd


DEFAULT_DLC_LIMIT = 0.33
DEFAULT_TRAVEL_LIMIT_MM = 120.0
DEFAULT_MU_KG = 78.0


def calculate_dlc(
    rms_tyre_force_n: float,
    ms_kg: float,
    mu_kg: float = DEFAULT_MU_KG,
) -> float:
    """Calculate Dynamic Load Coefficient."""

    return rms_tyre_force_n / (
        (ms_kg + mu_kg) * 9.81
    )


def make_group_id(row: pd.Series) -> str:
    """
    Create a group identifier that removes the damping option
    and seed.

    Each group therefore represents the same road/speed/load/
    dropout scenario with different damping choices.
    """

    return (
        f"{row['road_class']}_"
        f"{int(row['speed_mps'])}_"
        f"{int(row['ms_kg'])}_"
        f"{int(row['dropout_pct'])}"
    )


def prepare_ranking_data(
    predictions_df: pd.DataFrame,
    dlc_limit: float = DEFAULT_DLC_LIMIT,
    travel_limit_mm: float = DEFAULT_TRAVEL_LIMIT_MM,
    mu_kg: float = DEFAULT_MU_KG,
) -> pd.DataFrame:
    """
    Combine comfort predictions with physical constraints
    and determine feasibility.
    """

    df = predictions_df.copy()

    df["group_id"] = df.apply(
        make_group_id,
        axis=1,
    )

    df["dlc"] = df.apply(
        lambda row: calculate_dlc(
            row["rms_tyre_force_n"],
            row["ms_kg"],
            mu_kg,
        ),
        axis=1,
    )

    # Average across test seeds.
    grouped = (
        df.groupby(
            [
                "group_id",
                "road_class",
                "speed_mps",
                "ms_kg",
                "dropout_pct",
                "preset",
                "c_nsm",
            ],
            as_index=False,
        )
        .agg(
            comfort_pred=("comfort_pred", "mean"),
            lo=("lo", "mean"),
            hi=("hi", "mean"),
            dlc=("dlc", "mean"),
            max_travel_mm=("max_travel_mm", "mean"),
            bottom_out=("bottom_out", "any"),
        )
    )

    grouped["violations"] = ""

    violations = []

    for _, row in grouped.iterrows():
        row_violations = []

        if row["dlc"] > dlc_limit:
            row_violations.append("dlc")

        if row["max_travel_mm"] > travel_limit_mm:
            row_violations.append("travel")

        if bool(row["bottom_out"]):
            row_violations.append("bottom_out")

        violations.append(",".join(row_violations))

    grouped["violations"] = violations
    grouped["feasible"] = grouped["violations"] == ""

    return grouped


def select_recommendation(
    options: pd.DataFrame,
) -> tuple[str | None, str]:
    """
    Select the best feasible damping option.

    Returns:
        (recommended_preset, note)
    """

    feasible = options[
        options["feasible"]
    ].sort_values(
        "comfort_pred"
    )

    if feasible.empty:
        return None, "no feasible option"

    # If there is only one feasible option,
    # it is automatically the recommendation.
    if len(feasible) == 1:
        return (
            feasible.iloc[0]["preset"],
            "recommended",
        )

    best = feasible.iloc[0]
    second = feasible.iloc[1]

    # Check whether the prediction intervals overlap.
    intervals_overlap = (
        best["lo"] <= second["hi"]
        and second["lo"] <= best["hi"]
    )

    if intervals_overlap:
        return None, "no clear winner"

    return (
        best["preset"],
        "recommended",
    )


def rank_groups(
    predictions_df: pd.DataFrame,
    dlc_limit: float = DEFAULT_DLC_LIMIT,
    travel_limit_mm: float = DEFAULT_TRAVEL_LIMIT_MM,
    mu_kg: float = DEFAULT_MU_KG,
):
    """
    Generate the semi-active damping ranking.

    Returns one dictionary per scenario group.
    """

    prepared = prepare_ranking_data(
        predictions_df,
        dlc_limit=dlc_limit,
        travel_limit_mm=travel_limit_mm,
        mu_kg=mu_kg,
    )

    groups = []

    for group_id, group in prepared.groupby(
        "group_id"
    ):
        recommended, note = select_recommendation(
            group
        )

        options = []

        for _, row in group.iterrows():
            options.append({
                "preset": row["preset"],
                "c_nsm": float(row["c_nsm"]),
                "comfort_pred": float(
                    row["comfort_pred"]
                ),
                "lo": float(row["lo"]),
                "hi": float(row["hi"]),
                "feasible": bool(
                    row["feasible"]
                ),
                "violations": (
                    row["violations"]
                ),
            })

        groups.append({
            "group_id": group_id,
            "options": options,
            "recommended": recommended,
            "note": note,
        })

    return groups
