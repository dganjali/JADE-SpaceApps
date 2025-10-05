"""Generate high-quality visualizations for battery RUL model performance."""

from __future__ import annotations

import json
from collections import defaultdict
from pathlib import Path
from typing import Dict, Iterable, Tuple

import joblib
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib import ticker
from scipy.stats import gaussian_kde

# ---------------------------------------------------------------------------
# Paths and constants
# ---------------------------------------------------------------------------
VIS_DIR = Path(__file__).resolve().parent
FIGURES_DIR = VIS_DIR / "figures"
PROJECT_ROOT = VIS_DIR.parent
ARTIFACT_DIR = PROJECT_ROOT / "Li-Ion PCoE Degradation Modelling" / "degradation_modelling" / "artifacts"
PERFORMANCE_FILE = ARTIFACT_DIR / "performance_summary.json"
TRAINING_SUMMARY_FILE = ARTIFACT_DIR / "training_summary.joblib"
BASELINE_FILE = VIS_DIR / "baseline_metrics.json"
FEATURE_SCHEMA_FILE = ARTIFACT_DIR / "feature_schema.json"

FIGURES_DIR.mkdir(parents=True, exist_ok=True)

# Matplotlib global styling
plt.style.use("seaborn-v0_8-darkgrid")
plt.rcParams.update(
    {
        "font.family": "DejaVu Sans",
        "axes.titlesize": 14,
        "axes.labelsize": 12,
        "xtick.labelsize": 11,
        "ytick.labelsize": 11,
        "legend.fontsize": 10,
        "figure.dpi": 120,
        "savefig.dpi": 300,
        "axes.facecolor": "#fbfbfb",
        "figure.facecolor": "#f2f4f5",
        "axes.edgecolor": "#d1d1d1",
        "grid.color": "#e2e6ea",
        "grid.linestyle": "--",
        "grid.linewidth": 0.6,
    }
)

# Benchmark thresholds used for normalization
BENCHMARKS = {
    "mae": {"good": 20.0, "acceptable": 50.0, "poor": 100.0},
    "rmse": {"good": 40.0, "acceptable": 100.0, "poor": 150.0},
    # sMAPE is treated in percentage terms – values above 50% are typically concerning
    "smape": {"good": 25.0, "acceptable": 50.0, "poor": 200.0},
}


# ---------------------------------------------------------------------------
# Utility helpers
# ---------------------------------------------------------------------------

def load_json(path: Path) -> Dict:
    with path.open("r", encoding="utf-8") as fp:
        return json.load(fp)


def normalize_metric(metric: str, value: float) -> float:
    ref = BENCHMARKS[metric]
    if value <= ref["good"]:
        return 1.0
    if value >= ref["poor"]:
        return 0.0
    return (ref["poor"] - value) / (ref["poor"] - ref["good"])


def resolve_subset_indices(
    subset_sequences: np.ndarray,
    subset_features: np.ndarray,
    subset_targets: np.ndarray,
    all_sequences: np.ndarray,
    all_features: np.ndarray,
    all_targets: np.ndarray,
) -> np.ndarray:
    """Find indices of subset rows within the full dataset.

    Uses byte representations of sequences + features + target value for robust matching.
    """

    key_map: Dict[Tuple[bytes, bytes, float], list[int]] = defaultdict(list)
    for idx, (seq, feat, target) in enumerate(
        zip(all_sequences, all_features, all_targets, strict=True)
    ):
        key = (seq.tobytes(), feat.tobytes(), float(target))
        key_map[key].append(idx)

    indices: list[int] = []
    for seq, feat, target in zip(subset_sequences, subset_features, subset_targets, strict=True):
        key = (seq.tobytes(), feat.tobytes(), float(target))
        bucket = key_map.get(key)
        if not bucket:
            raise KeyError("Unable to reconcile subset sample with original dataset.")
        indices.append(bucket.pop())

    return np.array(indices, dtype=int)


def configure_axes(ax: plt.Axes, title: str, xlabel: str | None = None, ylabel: str | None = None) -> None:
    ax.set_title(title, loc="left", fontweight="bold")
    if xlabel:
        ax.set_xlabel(xlabel)
    if ylabel:
        ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.4)


def annotate_bars(ax: plt.Axes, bars: Iterable) -> None:
    for bar in bars:
        height = bar.get_height()
        ax.annotate(
            f"{height:.2f}",
            xy=(bar.get_x() + bar.get_width() / 2, height),
            xytext=(0, 6),
            textcoords="offset points",
            ha="center",
            va="bottom",
            fontsize=10,
            fontweight="bold",
        )


# ---------------------------------------------------------------------------
# Plot builders
# ---------------------------------------------------------------------------

def plot_metric_improvement(baseline: Dict[str, float], current: Dict[str, float]) -> Path:
    metrics = ["mae", "rmse", "smape"]
    labels = [metric.upper() for metric in metrics]
    baseline_values = [baseline[m] for m in metrics]
    current_values = [current[m] for m in metrics]

    x = np.arange(len(metrics))
    width = 0.35

    fig, ax = plt.subplots(figsize=(8, 5))
    bars1 = ax.bar(x - width / 2, baseline_values, width, label=baseline.get("label", "Baseline"), color="#9dbad5")
    bars2 = ax.bar(x + width / 2, current_values, width, label="Enhanced Hybrid (MHSA)", color="#ff7f6c")

    configure_axes(ax, "Error Metrics: Baseline vs Enhanced", ylabel="Cycles")
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.legend(frameon=True, facecolor="white", edgecolor="#d1d1d1")

    annotate_bars(ax, bars1)
    annotate_bars(ax, bars2)

    output_path = FIGURES_DIR / "metric_improvement_comparison.png"
    fig.tight_layout()
    fig.savefig(output_path)
    plt.close(fig)
    return output_path


def plot_metric_radar(baseline: Dict[str, float], current: Dict[str, float]) -> Path:
    metrics = ["mae", "rmse", "smape"]
    angles = np.linspace(0, 2 * np.pi, len(metrics), endpoint=False).tolist()
    angles += angles[:1]

    def _scores(values: Dict[str, float]) -> list[float]:
        scores = [normalize_metric(metric, values[metric]) for metric in metrics]
        scores += scores[:1]
        return scores

    baseline_scores = _scores(baseline)
    current_scores = _scores(current)

    fig, ax = plt.subplots(figsize=(7, 7), subplot_kw=dict(polar=True))
    ax.set_theta_offset(np.pi / 2)
    ax.set_theta_direction(-1)
    ax.set_thetagrids(np.degrees(angles[:-1]), [metric.upper() for metric in metrics])
    ax.set_ylim(0, 1)

    ax.plot(angles, baseline_scores, color="#5a82a6", linewidth=2, label=baseline.get("label", "Baseline"))
    ax.fill(angles, baseline_scores, color="#5a82a6", alpha=0.25)

    ax.plot(angles, current_scores, color="#ff6d50", linewidth=2.5, label="Enhanced Hybrid (MHSA)")
    ax.fill(angles, current_scores, color="#ff6d50", alpha=0.22)

    configure_axes(ax, "Normalized Performance vs NASA Benchmarks")
    ax.legend(loc="upper right", bbox_to_anchor=(1.2, 1.1), frameon=True, facecolor="white", edgecolor="#cccccc")

    output_path = FIGURES_DIR / "benchmark_radar_comparison.png"
    fig.tight_layout()
    fig.savefig(output_path)
    plt.close(fig)
    return output_path


def plot_predictions_vs_truth(y_true: np.ndarray, y_pred: np.ndarray) -> Path:
    order = np.argsort(y_true)
    y_true_sorted = y_true[order]
    y_pred_sorted = y_pred[order]

    fig, ax = plt.subplots(figsize=(10, 5.5))
    ax.plot(y_true_sorted, label="True RUL", color="#2d708e", linewidth=2.0)
    ax.plot(y_pred_sorted, label="Predicted RUL", color="#d1495b", linewidth=2.0, linestyle="--")
    configure_axes(ax, "Test Set RUL Predictions", xlabel="Sorted Test Sample", ylabel="Remaining Useful Life (cycles)")
    ax.fill_between(
        np.arange(len(y_true_sorted)),
        y_true_sorted,
        y_pred_sorted,
        color="#d1495b",
        alpha=0.08,
        label="Prediction gap",
    )
    ax.legend(frameon=True, facecolor="white", edgecolor="#d1d1d1")

    output_path = FIGURES_DIR / "test_predictions_vs_truth.png"
    fig.tight_layout()
    fig.savefig(output_path)
    plt.close(fig)
    return output_path


def plot_residual_distribution(residuals: np.ndarray) -> Path:
    fig, ax = plt.subplots(figsize=(8, 5))
    n, bins, _ = ax.hist(residuals, bins=20, color="#7fc8a9", alpha=0.7, edgecolor="#ffffff", density=True, label="Residual density")

    try:
        kde = gaussian_kde(residuals)
        x_vals = np.linspace(residuals.min() - 5, residuals.max() + 5, 200)
        ax.plot(x_vals, kde(x_vals), color="#1b4332", linewidth=2.2, label="KDE")
    except Exception:  # pragma: no cover - fallback for singular cases
        pass

    configure_axes(ax, "Residual Distribution", xlabel="Prediction Error (cycles)", ylabel="Density")
    ax.axvline(0, color="#4f4f4f", linewidth=1.2, linestyle=":", label="Zero error")
    ax.legend(frameon=True, facecolor="white", edgecolor="#d1d1d1")

    output_path = FIGURES_DIR / "residual_distribution.png"
    fig.tight_layout()
    fig.savefig(output_path)
    plt.close(fig)
    return output_path


def plot_residuals_vs_true_rul(y_true: np.ndarray, residuals: np.ndarray) -> Path:
    fig, ax = plt.subplots(figsize=(8.5, 5.5))
    scatter = ax.scatter(y_true, residuals, c=residuals, cmap="coolwarm", s=45, edgecolor="white", linewidth=0.6, alpha=0.9)
    configure_axes(ax, "Residuals vs True RUL", xlabel="True RUL (cycles)", ylabel="Error (Pred - True)")
    ax.axhline(0, color="#4f4f4f", linestyle=":", linewidth=1.2)
    cbar = plt.colorbar(scatter, ax=ax)
    cbar.set_label("Prediction error (cycles)")

    output_path = FIGURES_DIR / "residuals_vs_true_rul.png"
    fig.tight_layout()
    fig.savefig(output_path)
    plt.close(fig)
    return output_path


def plot_per_battery_mae(battery_ids: np.ndarray, residuals: np.ndarray) -> Path:
    unique_ids = np.unique(battery_ids)
    mae_values = []
    for bid in unique_ids:
        mask = battery_ids == bid
        mae_values.append(np.mean(np.abs(residuals[mask])))

    order = np.argsort(mae_values)
    ordered_ids = unique_ids[order]
    ordered_mae = np.array(mae_values)[order]

    cmap = cm.get_cmap("viridis")
    colors = cmap(np.linspace(0.2, 0.85, len(ordered_ids)))

    fig, ax = plt.subplots(figsize=(9, 5.5))
    bars = ax.barh(range(len(ordered_ids)), ordered_mae, color=colors)
    ax.invert_yaxis()
    configure_axes(ax, "Per-Battery MAE on Test Set", xlabel="MAE (cycles)", ylabel="Battery ID")
    ax.set_yticks(range(len(ordered_ids)))
    ax.set_yticklabels(ordered_ids)
    ax.xaxis.set_major_locator(ticker.MaxNLocator(integer=True))

    for i, bar in enumerate(bars):
        ax.annotate(
            f"{ordered_mae[i]:.2f}",
            xy=(bar.get_width(), bar.get_y() + bar.get_height() / 2),
            xytext=(6, 0),
            textcoords="offset points",
            va="center",
            fontsize=10,
            fontweight="bold",
        )

    output_path = FIGURES_DIR / "per_battery_mae.png"
    fig.tight_layout()
    fig.savefig(output_path)
    plt.close(fig)
    return output_path


def plot_cv_metric_bars(cv_metrics: Dict[str, float]) -> Path:
    metric_names = ["mae", "rmse", "smape"]
    means = [cv_metrics.get(f"cv_{m}_mean") for m in metric_names]
    stds = [cv_metrics.get(f"cv_{m}_std") for m in metric_names]

    fig, ax = plt.subplots(figsize=(8, 5))
    colors = ["#276fbf", "#ff6f59", "#ffa600"]
    bars = ax.bar(metric_names, means, yerr=stds, capsize=10, color=colors, alpha=0.85)
    configure_axes(ax, "Cross-Validation Metrics (Mean +/- Std)", ylabel="Cycles")
    ax.set_xticklabels([name.upper() for name in metric_names])

    annotate_bars(ax, bars)

    output_path = FIGURES_DIR / "cv_metrics_errorbars.png"
    fig.tight_layout()
    fig.savefig(output_path)
    plt.close(fig)
    return output_path


# ---------------------------------------------------------------------------
# Main orchestration
# ---------------------------------------------------------------------------

def main() -> None:
    if not PERFORMANCE_FILE.exists():
        raise FileNotFoundError(f"Performance summary not found: {PERFORMANCE_FILE}")
    if not TRAINING_SUMMARY_FILE.exists():
        raise FileNotFoundError(f"Training summary not found: {TRAINING_SUMMARY_FILE}")

    performance_summary = load_json(PERFORMANCE_FILE)
    baseline_metrics = load_json(BASELINE_FILE)["baseline"] if BASELINE_FILE.exists() else {}
    if not baseline_metrics:
        baseline_metrics = {"label": "Baseline", **performance_summary["test_metrics"]}

    training_summary = joblib.load(TRAINING_SUMMARY_FILE)

    # Current test metrics
    current_metrics = performance_summary["test_metrics"]

    # Predictions
    y_true = np.asarray(training_summary["results"]["predictions"]["test"]["true"], dtype=float)
    y_pred = np.asarray(training_summary["results"]["predictions"]["test"]["pred"], dtype=float)
    residuals = y_pred - y_true

    # All sequences/features/targets for indexing
    sequences_all = np.asarray(training_summary["data"]["sequences"], dtype=float)
    features_all = np.asarray(training_summary["data"]["features"], dtype=float)
    targets_all = np.asarray(training_summary["data"]["targets"], dtype=float)
    battery_ids_all = np.asarray(training_summary["data"]["battery_ids"])

    X_seq_test = np.asarray(training_summary["splits"]["X_seq_test"], dtype=float)
    X_feat_test = np.asarray(training_summary["splits"]["X_feat_test"], dtype=float)
    y_test = np.asarray(training_summary["splits"]["y_test"], dtype=float)

    test_indices = resolve_subset_indices(
        X_seq_test,
        X_feat_test,
        y_test,
        sequences_all,
        features_all,
        targets_all,
    )
    test_battery_ids = battery_ids_all[test_indices]

    # Build visualizations
    outputs = [
        plot_metric_improvement(baseline_metrics, current_metrics),
        plot_metric_radar(baseline_metrics, current_metrics),
        plot_predictions_vs_truth(y_true, y_pred),
        plot_residual_distribution(residuals),
        plot_residuals_vs_true_rul(y_true, residuals),
        plot_per_battery_mae(test_battery_ids, residuals),
        plot_cv_metric_bars(performance_summary.get("cv_metrics", {})),
    ]

    print("Generated figures:")
    for path in outputs:
        print(f" - {path.relative_to(PROJECT_ROOT)}")


if __name__ == "__main__":
    main()

