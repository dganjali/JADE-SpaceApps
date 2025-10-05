"""Entry point for training Li-ion battery RUL models."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Dict

import pandas as pd

from . import data_loader, feature_engineering, modeling


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Train a Li-ion battery RUL regression model.")
    parser.add_argument(
        "--data-root",
        type=Path,
        required=True,
        help="Directory containing NASA PCoE MATLAB files (B0005.mat, ...).",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("artifacts"),
        help="Directory where trained model and metrics will be saved.",
    )
    parser.add_argument(
        "--validation-fraction",
        type=float,
        default=0.25,
        help="Fraction of batteries to reserve for final holdout evaluation.",
    )
    parser.add_argument(
        "--random-state",
        type=int,
        default=42,
        help="Random seed for reproducibility.",
    )
    parser.add_argument(
        "--threshold-fraction",
        type=float,
        default=0.7,
        help="Relative capacity threshold (fraction of initial capacity) that defines EOL.",
    )
    parser.add_argument(
        "--no-final-refit",
        action="store_true",
        help="Skip refitting the model on the full dataset after evaluation.",
    )
    return parser.parse_args()


def _save_dataframe(df: pd.DataFrame, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(path, index=False)


def main() -> None:
    args = _parse_args()

    raw_cycles = data_loader.load_discharge_cycles(args.data_root)
    full_dataset = feature_engineering.build_training_table(
        raw_cycles,
        config=feature_engineering.TargetConfig(capacity_threshold_fraction=args.threshold_fraction),
    )

    train_df, holdout_df = data_loader.train_validation_split(
        full_dataset, validation_fraction=args.validation_fraction, random_state=args.random_state
    )

    metrics: Dict[str, float] = {}

    cv_metrics = modeling.cross_validate_pipeline(train_df, random_state=args.random_state)
    metrics.update(cv_metrics)

    model = modeling.fit_pipeline(train_df, random_state=args.random_state)

    if not holdout_df.empty:
        holdout_scores = modeling.evaluate_holdout(model, holdout_df)
        metrics.update(holdout_scores)

    # Optionally refit on the full dataset for deployment.
    if not args.no_final_refit:
        model = modeling.fit_pipeline(full_dataset, random_state=args.random_state)

    output_dir = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)

    modeling.save_model(model, output_dir / "battery_rul_model.joblib")
    modeling.save_metadata(metrics, output_dir / "metrics.json")

    with (output_dir / "feature_columns.json").open("w", encoding="utf-8") as fp:
        json.dump(modeling.FEATURE_COLUMNS, fp, indent=2)

    _save_dataframe(full_dataset, output_dir / "training_dataset.csv")


if __name__ == "__main__":
    main()

