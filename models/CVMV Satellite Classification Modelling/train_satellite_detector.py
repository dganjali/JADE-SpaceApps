"""YOLOv8 satellite detector training pipeline."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Dict, List

import matplotlib.pyplot as plt
import pandas as pd
from ultralytics import YOLO

# Minimal helper conversions

def _to_float(value):
    if value is None:
        return None
    if isinstance(value, (list, tuple)):
        return [_to_float(v) for v in value]
    try:
        return float(value)
    except (TypeError, ValueError):
        return None

def _extract_box_metrics(metrics_obj: object) -> Dict[str, float]:
    summary: Dict[str, float] = {}
    if not metrics_obj:
        return summary
    box = getattr(metrics_obj, "box", None)
    if not box:
        return summary
    for key in ("map50", "map", "mp", "mr", "precision", "recall"):
        val = getattr(box, key, None)
        if val is not None:
            summary[key] = _to_float(val)
    if "precision" not in summary and hasattr(box, "mp"):
        summary["precision"] = _to_float(box.mp)
    if "recall" not in summary and hasattr(box, "mr"):
        summary["recall"] = _to_float(box.mr)
    if "map50_95" not in summary and hasattr(box, "map"):
        summary["map50_95"] = _to_float(box.map)
    return summary

def _plot_loss_curves(history: pd.DataFrame, viz_dir: Path) -> Path:
    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(history["epoch"], history.get("train/box_loss"), label="Train box")
    ax.plot(history["epoch"], history.get("train/cls_loss"), label="Train cls")
    ax.plot(history["epoch"], history.get("val/box_loss"), label="Val box")
    ax.plot(history["epoch"], history.get("val/cls_loss"), label="Val cls")
    ax.set_xlabel("Epoch")
    ax.set_ylabel("Loss")
    ax.set_title("Training vs Validation Loss")
    ax.grid(True, linestyle="--", alpha=0.5)
    ax.legend()
    out = viz_dir / "loss_curves.png"
    fig.tight_layout()
    fig.savefig(out, dpi=300)
    plt.close(fig)
    return out

def _plot_detection_metrics(history: pd.DataFrame, viz_dir: Path) -> Path:
    fig, ax = plt.subplots(figsize=(9, 5))
    if "metrics/mAP50(B)" in history:
        ax.plot(history["epoch"], history["metrics/mAP50(B)"], label="mAP@50")
    if "metrics/mAP50-95(B)" in history:
        ax.plot(history["epoch"], history["metrics/mAP50-95(B)"], label="mAP@50-95")
    if "metrics/precision(B)" in history:
        ax.plot(history["epoch"], history["metrics/precision(B)"], label="Precision")
    if "metrics/recall(B)" in history:
        ax.plot(history["epoch"], history["metrics/recall(B)"], label="Recall")
    ax.set_xlabel("Epoch")
    ax.set_ylabel("Score")
    ax.set_title("Detection Metrics")
    ax.set_ylim(0, 1.05)
    ax.grid(True, linestyle="--", alpha=0.5)
    ax.legend()
    out = viz_dir / "detection_metrics.png"
    fig.tight_layout()
    fig.savefig(out, dpi=300)
    plt.close(fig)
    return out

def _save_results(payload: Dict, json_path: Path) -> None:
    json_path.write_text(json.dumps(payload, indent=2))

def main() -> None:
    root = Path.cwd()
    work_dir = Path(__file__).resolve().parent
    viz_dir = work_dir / "visualizations"
    viz_dir.mkdir(parents=True, exist_ok=True)

    data_path = root / "YOLOv8 Satellite Data" / "data.yaml"
    project_dir = root / "runs" / "detect"
    run_name = "satellite_detector"

    model = YOLO("yolov8n.pt")
    train_results = model.train(
        data=str(data_path),
        epochs=50,
        imgsz=640,
        batch=16,
        project=str(project_dir),
        name=run_name,
        plots=True,
    )

    trainer = model.trainer
    results_dir = Path(trainer.save_dir)
    best_weights = Path(trainer.best)
    print(f"Best weights saved to: {best_weights}")

    history_csv = results_dir / "results.csv"
    history = pd.read_csv(history_csv)
    final_row = history.iloc[-1].to_dict()

    loss_plot = _plot_loss_curves(history, viz_dir)
    metric_plot = _plot_detection_metrics(history, viz_dir)

    training_summary = {
        "run_dir": str(results_dir.relative_to(root) if results_dir.is_relative_to(root) else results_dir),
        "best_weights": str(best_weights.relative_to(root) if best_weights.is_relative_to(root) else best_weights),
        "final_epoch": int(final_row.get("epoch", len(history) - 1)),
        "train_loss": {
            "box": _to_float(final_row.get("train/box_loss")),
            "cls": _to_float(final_row.get("train/cls_loss")),
            "dfl": _to_float(final_row.get("train/dfl_loss")),
        },
        "val_loss": {
            "box": _to_float(final_row.get("val/box_loss")),
            "cls": _to_float(final_row.get("val/cls_loss")),
            "dfl": _to_float(final_row.get("val/dfl_loss")),
        },
        "metrics": {
            "precision": _to_float(final_row.get("metrics/precision(B)")),
            "recall": _to_float(final_row.get("metrics/recall(B)")),
            "map50": _to_float(final_row.get("metrics/mAP50(B)")),
            "map50_95": _to_float(final_row.get("metrics/mAP50-95(B)")),
        },
        "plots": {
            "loss": str(loss_plot.relative_to(root) if loss_plot.is_relative_to(root) else loss_plot),
            "metrics": str(metric_plot.relative_to(root) if metric_plot.is_relative_to(root) else metric_plot),
        },
    }

    predictions_dir = viz_dir / "val_predictions"
    model.predict(
        source=str(root / "YOLOv8 Satellite Data" / "valid" / "images"),
        imgsz=640,
        conf=0.25,
        project=str(viz_dir),
        name="val_predictions",
        save=True,
        exist_ok=True,
    )
    training_summary["plots"]["predictions_dir"] = str(predictions_dir.relative_to(root) if predictions_dir.is_relative_to(root) else predictions_dir)

    results_json = root / "training_results.json"
    payload = {"training": training_summary}
    _save_results(payload, results_json)

    test_results = model.val(data=str(data_path), split="test", imgsz=640, batch=16)
    test_metrics = _extract_box_metrics(getattr(test_results, "metrics", None))
    speed_info = getattr(test_results, "speed", {})
    if isinstance(speed_info, dict):
        inference_time = _to_float(speed_info.get("inference"))
    else:
        inference_time = _to_float(getattr(speed_info, "inference", None))

    payload["test"] = {
        "metrics": test_metrics,
        "speed_ms": inference_time,
    }
    _save_results(payload, results_json)

    print(f"Results saved to {results_json}")

if __name__ == "__main__":
    main()
