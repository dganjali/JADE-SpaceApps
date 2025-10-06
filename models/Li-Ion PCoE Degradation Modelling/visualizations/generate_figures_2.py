#!/usr/bin/env python
"""Generate enhanced RUL visualisations using training artifacts."""

from pathlib import Path
from typing import Dict, Iterable

import joblib
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

BASE_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = BASE_DIR.parent
ARTIFACTS_DIR = PROJECT_ROOT / "degradation_modelling" / "artifacts"
SUMMARY_PATH = ARTIFACTS_DIR / "training_summary.joblib"

FIGURES_DIR = BASE_DIR / "figures_2"
FIGURES_DIR.mkdir(parents=True, exist_ok=True)

COSMIC_COLORS: Dict[str, str] = {
    "aqua": "#4BF9FF",
    "mauve": "#C77DFF",
    "chartreuse": "#C4F247",
    "amber": "#FFB547",
    "magenta": "#FF5DA2",
    "violet": "#8C54FF",
    "teal": "#41E2BA",
    "gold": "#FFDC5E",
    "crimson": "#FF3366",
}

plt.style.use("dark_background")
plt.rcParams.update(
    {
        "axes.facecolor": "#090A10",
        "figure.facecolor": "#090A10",
        "axes.edgecolor": "#1F2333",
        "axes.labelcolor": "#E6F1FF",
        "xtick.color": "#C7D8FF",
        "ytick.color": "#C7D8FF",
        "font.family": "DejaVu Sans",
        "axes.titleweight": "bold",
    }
)


def _load_summary() -> dict:
    if not SUMMARY_PATH.exists():
        raise FileNotFoundError(f"Expected artifact not found: {SUMMARY_PATH}")
    return joblib.load(SUMMARY_PATH)


def _prepare_series_dataframe(summary: dict) -> pd.DataFrame:
    df: pd.DataFrame = summary["data"]["engineered_dataframe"].copy()
    num_sequences = summary["metadata"].get("num_sequences", len(df))
    df = df.iloc[-num_sequences:]
    df = df[["battery_id", "cycle_index", "rul_cycles"]].rename(
        columns={"cycle_index": "cycle", "rul_cycles": "rul"}
    )
    df = df.sort_values(["battery_id", "cycle"]).reset_index(drop=True)
    return df


def _plot_overall_trend(df: pd.DataFrame) -> None:
    grouped = df.groupby("cycle")
    summary = grouped["rul"].agg(["mean", "median", "count"]).reset_index()
    x = summary["cycle"].to_numpy()
    mean_rul = summary["mean"].to_numpy()
    rolling = pd.Series(mean_rul).rolling(window=7, center=True).mean().to_numpy()
    coeffs = np.polyfit(x, mean_rul, deg=3)
    trend = np.polyval(coeffs, x)

    fig, ax = plt.subplots(figsize=(12, 6), dpi=160)
    ax.scatter(
        x,
        mean_rul,
        color=COSMIC_COLORS["teal"],
        alpha=0.5,
        s=35,
        label="Mean RUL per Cycle",
    )
    ax.plot(
        x,
        rolling,
        color=COSMIC_COLORS["mauve"],
        linewidth=2.2,
        label="7-cycle Rolling Mean",
    )
    ax.plot(
        x,
        trend,
        color=COSMIC_COLORS["magenta"],
        linewidth=2.4,
        linestyle="--",
        label="Cubic Trendline",
    )
    ax.set_title("SMAP Battery RUL Decay Across Cycles")
    ax.set_xlabel("Cycle Index")
    ax.set_ylabel("Remaining Useful Life (cycles)")
    ax.grid(color="#1F2333", linestyle="--", linewidth=0.5, alpha=0.5)
    ax.legend(loc="upper right", frameon=False)
    ax.set_xlim(x.min(), x.max())
    ax.set_ylim(bottom=0)
    fig.savefig(FIGURES_DIR / "overall_rul_trend.png", bbox_inches="tight")
    plt.close(fig)


def _plot_sample_batteries(df: pd.DataFrame, num_samples: int = 4) -> None:
    battery_order = (
        df.groupby("battery_id")["cycle"]
        .max()
        .sort_values(ascending=False)
        .index.tolist()
    )
    selected = battery_order[:num_samples]

    fig, ax = plt.subplots(figsize=(12, 6), dpi=160)
    palette: Iterable[str] = [
        COSMIC_COLORS["aqua"],
        COSMIC_COLORS["amber"],
        COSMIC_COLORS["violet"],
        COSMIC_COLORS["crimson"],
        COSMIC_COLORS["gold"],
    ]

    for idx, (battery, color) in enumerate(zip(selected, palette)):
        subset = df[df["battery_id"] == battery]
        ax.plot(
            subset["cycle"],
            subset["rul"],
            color=color,
            linewidth=2.2,
            marker="o",
            markersize=4,
            label=f"{battery} RUL",
        )

    ax.set_title("Representative Battery RUL Trajectories")
    ax.set_xlabel("Cycle Index")
    ax.set_ylabel("Remaining Useful Life (cycles)")
    ax.grid(color="#1F2333", linestyle="--", linewidth=0.5, alpha=0.4)
    ax.legend(loc="upper right", frameon=False, ncol=2)
    ax.set_ylim(bottom=0)
    fig.savefig(FIGURES_DIR / "battery_rul_trajectories.png", bbox_inches="tight")
    plt.close(fig)


def _plot_prediction_alignment(summary: dict) -> None:
    predictions = summary["results"]["predictions"]
    test_true = predictions["test"]["true"]
    test_pred = predictions["test"]["pred"]

    x = np.arange(len(test_true))
    smooth_pred = pd.Series(test_pred).rolling(window=9, center=True).mean()

    fig, ax = plt.subplots(figsize=(12, 6), dpi=160)
    ax.plot(
        x,
        test_true,
        color=COSMIC_COLORS["gold"],
        linewidth=1.8,
        label="True RUL (test)",
        alpha=0.85,
    )
    ax.plot(
        x,
        test_pred,
        color=COSMIC_COLORS["chartreuse"],
        linewidth=1.2,
        alpha=0.6,
        label="Predicted RUL",
    )
    ax.plot(
        x,
        smooth_pred,
        color=COSMIC_COLORS["magenta"],
        linewidth=2.3,
        linestyle="--",
        label="Predicted RUL (smoothed)",
    )

    ax.set_title("Model Alignment on Held-out Cycles")
    ax.set_xlabel("Test Sample Index (chronological order)")
    ax.set_ylabel("Remaining Useful Life (cycles)")
    ax.grid(color="#1F2333", linestyle="--", linewidth=0.5, alpha=0.4)
    ax.legend(loc="upper right", frameon=False)
    ax.set_ylim(bottom=0)
    fig.savefig(FIGURES_DIR / "test_set_rul_alignment.png", bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    summary = _load_summary()
    df = _prepare_series_dataframe(summary)
    _plot_overall_trend(df)
    _plot_sample_batteries(df)
    _plot_prediction_alignment(summary)
    print(f"Saved figures to {FIGURES_DIR}")


if __name__ == "__main__":
    main()
