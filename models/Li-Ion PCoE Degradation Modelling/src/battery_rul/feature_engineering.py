"""Feature engineering utilities for remaining useful life modelling."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Sequence

import pandas as pd


@dataclass(frozen=True)
class TargetConfig:
    capacity_threshold_fraction: float = 0.7
    minimum_rul: int = 0


def _validate_threshold(value: float) -> None:
    if not 0 < value <= 1:
        raise ValueError("capacity_threshold_fraction must be within (0, 1]")


def add_degradation_features(frame: pd.DataFrame) -> pd.DataFrame:
    """Add capacity-based degradation features and simple temporal derivatives."""

    df = frame.copy()
    df["initial_capacity_ah"] = df.groupby("battery_id")["capacity_ah"].transform("first")
    df["capacity_ratio"] = df["capacity_ah"] / df["initial_capacity_ah"]
    df["capacity_fade_ah"] = df["initial_capacity_ah"] - df["capacity_ah"]
    df["capacity_delta_ah"] = (
        df.groupby("battery_id")["capacity_ah"].diff().fillna(0.0)
    )
    df["capacity_delta_pct"] = df["capacity_delta_ah"] / df["initial_capacity_ah"]
    df["time_per_ah"] = df["time_elapsed_s"] / df["capacity_ah"].replace(0, pd.NA)
    df["temp_delta"] = df.groupby("battery_id")["temperature_mean"].diff().fillna(0.0)
    df["voltage_drop"] = df["voltage_measured_max"] - df["voltage_measured_min"]
    df["voltage_drop_delta"] = df.groupby("battery_id")["voltage_drop"].diff().fillna(0.0)
    return df


def add_rul_targets(frame: pd.DataFrame, config: TargetConfig = TargetConfig()) -> pd.DataFrame:
    """Derive Remaining Useful Life labels and metadata per battery."""

    _validate_threshold(config.capacity_threshold_fraction)
    if config.minimum_rul < 0:
        raise ValueError("minimum_rul must be non-negative")

    def _annotate(group: pd.DataFrame) -> pd.DataFrame:
        initial_capacity = group["capacity_ah"].iloc[0]
        threshold_value = initial_capacity * config.capacity_threshold_fraction
        under_threshold = group[group["capacity_ah"] <= threshold_value]
        if not under_threshold.empty:
            eol_cycle = int(under_threshold["cycle_index"].iloc[0])
        else:
            eol_cycle = int(group["cycle_index"].iloc[-1])

        group = group.copy()
        group["eol_cycle_index"] = eol_cycle
        group["cycles_to_eol"] = group["eol_cycle_index"] - group["cycle_index"]
        group.loc[group["cycles_to_eol"] < config.minimum_rul, "cycles_to_eol"] = config.minimum_rul
        group["cycle_progress"] = group["cycle_index"] / group["eol_cycle_index"].where(
            group["eol_cycle_index"] > 0, other=group["cycle_index"].max()
        )
        group["cycle_progress"] = group["cycle_progress"].clip(upper=1.0)
        return group

    annotated = frame.groupby("battery_id", group_keys=False).apply(_annotate)
    annotated.rename(columns={"cycles_to_eol": "rul_cycles"}, inplace=True)
    return annotated


def build_training_table(frame: pd.DataFrame, config: TargetConfig = TargetConfig()) -> pd.DataFrame:
    """Convenience helper that chains feature and target creation."""

    enriched = add_degradation_features(frame)
    labelled = add_rul_targets(enriched, config=config)
    labelled.dropna(axis="columns", how="all", inplace=True)
    return labelled.reset_index(drop=True)

