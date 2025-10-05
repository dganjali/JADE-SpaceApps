"""Data ingestion utilities for the NASA PCoE lithium-ion battery dataset."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List

import numpy as np
import pandas as pd
from scipy.io import loadmat

DISCHARGE_LABEL = "discharge"
MAT_FILE_PATTERN = "B*.mat"


@dataclass
class CycleRecord:
    """Container for the engineered features extracted from a single discharge cycle."""

    battery_id: str
    cycle_index: int
    sequence_index: int
    capacity_ah: float
    time_elapsed_s: float
    voltage_measured_mean: float
    voltage_measured_std: float
    voltage_measured_min: float
    voltage_measured_max: float
    voltage_load_mean: float
    current_measured_mean: float
    current_measured_std: float
    current_load_mean: float
    temperature_mean: float
    temperature_max: float
    internal_resistance_mohm: float

    @classmethod
    def fields(cls) -> List[str]:
        return [
            "battery_id",
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
        ]


def _ensure_iterable(value) -> np.ndarray:
    """Convert MATLAB cell/struct fields to a flat numpy array of floats."""
    arr = np.array(value, dtype=float)
    if arr.ndim == 0:
        return arr.reshape(1)
    return arr


def _to_float(value) -> float:
    try:
        arr = np.array(value, dtype=float)
    except Exception as exc:  # pragma: no cover - defensive guard
        raise ValueError(f"Expected numeric value, received {type(value)}") from exc
    return float(arr.reshape(1)[0])


def _extract_cycle_features(battery_id: str, cycle, sequence_index: int) -> CycleRecord:
    data = cycle.data
    voltage_measured = _ensure_iterable(data.Voltage_measured)
    voltage_load = _ensure_iterable(getattr(data, "Voltage_load", np.nan))
    current_measured = _ensure_iterable(data.Current_measured)
    current_load = _ensure_iterable(getattr(data, "Current_load", np.nan))
    temperature = _ensure_iterable(data.Temperature_measured)
    time_array = _ensure_iterable(data.Time)

    capacity = _to_float(data.Capacity)
    cycle_index = int(np.atleast_1d(cycle.cycle)[0])
    time_elapsed = float(time_array[-1] - time_array[0]) if time_array.size > 1 else float(time_array[0])

    ir_value = getattr(data, "IR", np.nan)

    return CycleRecord(
        battery_id=battery_id,
        cycle_index=cycle_index,
        sequence_index=sequence_index,
        capacity_ah=capacity,
        time_elapsed_s=time_elapsed,
        voltage_measured_mean=float(np.nanmean(voltage_measured)),
        voltage_measured_std=float(np.nanstd(voltage_measured)),
        voltage_measured_min=float(np.nanmin(voltage_measured)),
        voltage_measured_max=float(np.nanmax(voltage_measured)),
        voltage_load_mean=float(np.nanmean(voltage_load)),
        current_measured_mean=float(np.nanmean(current_measured)),
        current_measured_std=float(np.nanstd(current_measured)),
        current_load_mean=float(np.nanmean(current_load)),
        temperature_mean=float(np.nanmean(temperature)),
        temperature_max=float(np.nanmax(temperature)),
        internal_resistance_mohm=float(np.nanmean(_ensure_iterable(ir_value))) * 1e3,
    )


def load_discharge_cycles(data_root: Path) -> pd.DataFrame:
    """Load all discharge cycles from every .mat file under ``data_root``.

    Parameters
    ----------
    data_root:
        Directory that contains the MATLAB files published by the NASA PCoE team
        (e.g. B0005.mat, B0006.mat, ...).

    Returns
    -------
    pd.DataFrame
        Tabular representation where each row corresponds to a discharge cycle
        enriched with summary statistics that are well-suited for classical ML models.
    """

    data_root = Path(data_root)
    mat_files = sorted(data_root.glob(MAT_FILE_PATTERN))
    if not mat_files:
        raise FileNotFoundError(
            f"No MATLAB files matching pattern '{MAT_FILE_PATTERN}' were found in {data_root}."
        )

    records: List[CycleRecord] = []

    for mat_path in mat_files:
        battery_key = mat_path.stem
        mat = loadmat(mat_path, struct_as_record=False, squeeze_me=True)
        battery_struct = mat[battery_key]
        cycles = np.atleast_1d(battery_struct.cycle)
        discharge_cycles = [c for c in cycles if getattr(c, "type", None) == DISCHARGE_LABEL]

        for sequence_index, cycle in enumerate(discharge_cycles):
            record = _extract_cycle_features(battery_key, cycle, sequence_index)
            records.append(record)

    frame = pd.DataFrame.from_records([r.__dict__ for r in records], columns=CycleRecord.fields())
    frame.sort_values(["battery_id", "cycle_index"], inplace=True)
    frame.reset_index(drop=True, inplace=True)
    return frame


def train_validation_split(
    frame: pd.DataFrame, validation_fraction: float = 0.2, random_state: int | None = 42
) -> tuple[pd.DataFrame, pd.DataFrame]:
    """Simple helper that performs a stratified split across battery IDs.

    The function is intended for rapid experimentation; model training will
    typically rely on ``GroupKFold`` during cross-validation.
    """

    if not 0 < validation_fraction < 1:
        raise ValueError("validation_fraction must be in (0, 1)")

    rng = np.random.default_rng(random_state)
    batteries = frame["battery_id"].unique()
    rng.shuffle(batteries)
    cutoff = int(len(batteries) * (1 - validation_fraction))
    train_ids = set(batteries[:cutoff])

    train_df = frame[frame["battery_id"].isin(train_ids)].reset_index(drop=True)
    val_df = frame[~frame["battery_id"].isin(train_ids)].reset_index(drop=True)
    return train_df, val_df

