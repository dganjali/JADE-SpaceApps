"""Model utilities for training and evaluating Remaining Useful Life regressors."""
from __future__ import annotations

from pathlib import Path
from typing import Dict, List

import joblib
import numpy as np
import pandas as pd
from sklearn.compose import ColumnTransformer
from sklearn.impute import SimpleImputer
from sklearn.metrics import mean_absolute_error, mean_squared_error
from sklearn.model_selection import GroupKFold, cross_validate
from sklearn.pipeline import Pipeline
from sklearn.preprocessing import StandardScaler
from sklearn.ensemble import HistGradientBoostingRegressor

FEATURE_COLUMNS: List[str] = [
    "cycle_index",
    "sequence_index",
    "capacity_ah",
    "time_elapsed_s",
    "voltage_measured_mean",
    "voltage_measured_std",
    "voltage_measured_min",
    "voltage_measured_max",
    "voltage_load_mean",
    "current_measured_mean",
    "current_measured_std",
    "current_load_mean",
    "temperature_mean",
    "temperature_max",
    "internal_resistance_mohm",
    "initial_capacity_ah",
    "capacity_ratio",
    "capacity_fade_ah",
    "capacity_delta_ah",
    "capacity_delta_pct",
    "time_per_ah",
    "temp_delta",
    "voltage_drop",
    "voltage_drop_delta",
    "cycle_progress",
]

TARGET_COLUMN = "rul_cycles"
GROUP_COLUMN = "battery_id"


def build_regression_pipeline(random_state: int | None = 42) -> Pipeline:
    numerical_transformer = Pipeline(
        steps=[
            ("imputer", SimpleImputer(strategy="median")),
            ("scaler", StandardScaler()),
        ]
    )

    preprocessor = ColumnTransformer(
        transformers=[("numeric", numerical_transformer, FEATURE_COLUMNS)],
        remainder="drop",
    )

    regressor = HistGradientBoostingRegressor(
        loss="squared_error",
        learning_rate=0.05,
        max_iter=600,
        max_depth=6,
        min_samples_leaf=15,
        l2_regularization=0.01,
        random_state=random_state,
    )

    return Pipeline(steps=[("preprocess", preprocessor), ("regressor", regressor)])


def cross_validate_pipeline(
    frame: pd.DataFrame,
    n_splits: int = 4,
    random_state: int | None = 42,
) -> Dict[str, float]:
    pipeline = build_regression_pipeline(random_state=random_state)
    groups = frame[GROUP_COLUMN]
    gkf = GroupKFold(n_splits=n_splits)

    scoring = {
        "mae": "neg_mean_absolute_error",
        "rmse": "neg_root_mean_squared_error",
    }

    scores = cross_validate(
        pipeline,
        frame,
        frame[TARGET_COLUMN],
        groups=groups,
        cv=gkf,
        scoring=scoring,
        n_jobs=-1,
        return_train_score=False,
    )

    aggregated: Dict[str, float] = {}
    for metric_name in scoring.keys():
        test_key = f"test_{metric_name}"
        values = -np.asarray(scores[test_key])
        aggregated[f"cv_{metric_name}_mean"] = float(values.mean())
        aggregated[f"cv_{metric_name}_std"] = float(values.std())
    return aggregated


def fit_pipeline(frame: pd.DataFrame, random_state: int | None = 42) -> Pipeline:
    model = build_regression_pipeline(random_state=random_state)
    model.fit(frame, frame[TARGET_COLUMN])
    return model


def evaluate_holdout(model: Pipeline, holdout: pd.DataFrame) -> Dict[str, float]:
    predictions = model.predict(holdout)
    mae = mean_absolute_error(holdout[TARGET_COLUMN], predictions)
    rmse = mean_squared_error(holdout[TARGET_COLUMN], predictions, squared=False)
    return {"holdout_mae": mae, "holdout_rmse": rmse}


def save_model(model: Pipeline, path: Path) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    joblib.dump(model, path)


def save_metadata(metadata: Dict[str, float], path: Path) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    pd.Series(metadata).to_json(path, indent=2)

