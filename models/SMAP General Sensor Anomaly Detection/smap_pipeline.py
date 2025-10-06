#!/usr/bin/env python
"""
LSTM autoencoder pipeline for NASA SMAP telemetry anomaly detection.

This script downloads the SMAP subset of the NASA Anomaly Detection dataset,
preprocesses the data, trains an LSTM autoencoder on normal telemetry windows,
and visualises detection results with a dark, space-inspired aesthetic.
"""

import json
import ast
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import kagglehub
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import tensorflow as tf
from sklearn.metrics import (
    classification_report,
    precision_recall_fscore_support,
    roc_auc_score,
)
from sklearn.preprocessing import StandardScaler
from tensorflow.keras import Input, Model
from tensorflow.keras.callbacks import EarlyStopping
from tensorflow.keras.layers import Dense, LSTM, RepeatVector, TimeDistributed

np.random.seed(42)
tf.random.set_seed(42)

BASE_DIR = Path(__file__).resolve().parent
RESULTS_DIR = BASE_DIR / "results"
MODEL_PATH = BASE_DIR / "smap_anomaly_detector.h5"

WINDOW_SIZE = 72
STEP_SIZE = 4
EPOCHS = 100
BATCH_SIZE = 128
VALIDATION_SPLIT = 0.1

RESULTS_DIR.mkdir(parents=True, exist_ok=True)

plt.style.use("dark_background")
plt.rcParams.update(
    {
        "axes.facecolor": "#050608",
        "figure.facecolor": "#050608",
        "font.size": 12,
        "axes.labelsize": 14,
        "axes.titlesize": 16,
        "legend.fontsize": 12,
    }
)

COLORS = {
    "train_loss": "#00f5ff",
    "val_loss": "#ff47ff",
    "hist": "#00eaff",
    "signal": "#ffd700",
    "reconstruction": "#ff47ff",
    "threshold": "#ffd700",
    "alert": "#ff4d6d",
    "score": "#00ffc6",
}


def _parse_anomaly_sequences(sequence_str: Any, series_length: int) -> List[Tuple[int, int]]:
    """Parse anomaly sequence strings like [[start, end], ...] from the CSV.

    Returns a list of inclusive index intervals clipped to the available series length.
    """
    if isinstance(sequence_str, float):
        if np.isnan(sequence_str):
            return []
    if not isinstance(sequence_str, str):
        sequence_str = str(sequence_str)

    sequence_str = sequence_str.strip()
    if not sequence_str:
        return []

    try:
        raw_sequences = ast.literal_eval(sequence_str)
    except (ValueError, SyntaxError):
        return []

    parsed: List[Tuple[int, int]] = []
    for entry in raw_sequences:
        if not isinstance(entry, (list, tuple)) or len(entry) != 2:
            continue
        start, end = entry
        try:
            start_idx = int(start)
            end_idx = int(end)
        except (TypeError, ValueError):
            continue
        start_idx = max(0, start_idx)
        end_idx = min(series_length - 1, end_idx)
        if end_idx >= start_idx:
            parsed.append((start_idx, end_idx))

    return parsed

def _interpolate_array(values: np.ndarray) -> np.ndarray:
    """Replace missing/extreme values and interpolate with linear fill."""
    df = pd.DataFrame(values)
    df = df.replace([np.inf, -np.inf], np.nan)
    df = df.interpolate(method="linear", limit_direction="both")
    df = df.fillna(method="ffill").fillna(method="bfill").fillna(0)
    return df.to_numpy(dtype=np.float32)


def _create_sequences(
    data: np.ndarray,
    labels: np.ndarray,
    window_size: int,
    step_size: int,
) -> Tuple[np.ndarray, np.ndarray, List[Tuple[int, int]]]:
    """Generate sliding windows and associated labels/indices for one channel."""
    sequences: List[np.ndarray] = []
    sequence_labels: List[int] = []
    indices: List[Tuple[int, int]] = []
    total_length = len(data)

    if total_length < window_size:
        empty_seq = np.empty((0, window_size, data.shape[1]), dtype=np.float32)
        empty_labels = np.empty((0,), dtype=int)
        return empty_seq, empty_labels, []

    for start in range(0, total_length - window_size + 1, step_size):
        end = start + window_size
        window = data[start:end]
        label_slice = labels[start:end]
        sequences.append(window)
        window_label = int(label_slice.max()) if label_slice.size else 0
        sequence_labels.append(window_label)
        indices.append((start, end))

    stacked = np.stack(sequences).astype(np.float32) if sequences else np.empty((0, window_size, data.shape[1]), dtype=np.float32)
    label_array = np.asarray(sequence_labels, dtype=int) if sequence_labels else np.empty((0,), dtype=int)
    return stacked, label_array, indices


def load_smap_data() -> Dict[str, Any]:
    print("[INFO] Downloading NASA SMAP dataset via KaggleHub...")
    dataset_path = Path(
        kagglehub.dataset_download("patrickfleith/nasa-anomaly-detection-dataset-smap-msl")
    )
    print(f"[INFO] Dataset available at: {dataset_path}")

    data_root = dataset_path / "data" / "data"
    train_dir = data_root / "train"
    test_dir = data_root / "test"
    labels_path = dataset_path / "labeled_anomalies.csv"

    if not data_root.exists() or not train_dir.exists() or not test_dir.exists():
        raise FileNotFoundError(
            "Expected NASA dataset layout with data/data/{train,test} directories not found."
        )
    if not labels_path.exists():
        raise FileNotFoundError("Missing labeled_anomalies.csv in downloaded dataset.")

    labels_df = pd.read_csv(labels_path)
    labels_df['spacecraft'] = labels_df['spacecraft'].astype(str).str.upper()
    smap_df = labels_df[labels_df['spacecraft'] == 'SMAP'].copy()
    if smap_df.empty:
        raise ValueError("Labeled anomalies file does not contain SMAP entries.")

    requested_channels = sorted(smap_df['chan_id'].astype(str).unique().tolist())

    train_data: Dict[str, np.ndarray] = {}
    test_data: Dict[str, np.ndarray] = {}
    anomaly_sequences: Dict[str, List[Tuple[int, int]]] = {}
    skipped_channels: List[str] = []

    grouped_sequences = (
        smap_df.groupby('chan_id')['anomaly_sequences']
        .apply(list)
        .to_dict()
    )

    for channel in requested_channels:
        channel_name = str(channel)
        train_file = train_dir / f"{channel_name}.npy"
        test_file = test_dir / f"{channel_name}.npy"

        if not train_file.exists() or not test_file.exists():
            skipped_channels.append(channel_name)
            continue

        train_arr = np.load(train_file, allow_pickle=True)
        test_arr = np.load(test_file, allow_pickle=True)

        train_data[channel_name] = np.asarray(train_arr, dtype=np.float32)
        test_data[channel_name] = np.asarray(test_arr, dtype=np.float32)

        sequences: List[Tuple[int, int]] = []
        for seq_str in grouped_sequences.get(channel, []):
            sequences.extend(
                _parse_anomaly_sequences(seq_str, series_length=test_data[channel_name].shape[0])
            )
        sequences.sort()
        anomaly_sequences[channel_name] = sequences

    loaded_channels = sorted(train_data.keys())
    if not loaded_channels:
        raise FileNotFoundError("No SMAP channels were loaded. Check dataset contents.")

    print(
        f"[INFO] Loaded SMAP channels ({len(loaded_channels)}): "
        + ", ".join(loaded_channels)
    )
    if skipped_channels:
        print(
            f"[WARN] Skipped {len(skipped_channels)} channel(s) missing files: "
            + ", ".join(skipped_channels)
        )

    return {
        'train': train_data,
        'test': test_data,
        'anomaly_sequences': anomaly_sequences,
        'channel_names': loaded_channels,
    }

def preprocess_data(
    train_data: Dict[str, np.ndarray],
    test_data: Dict[str, np.ndarray],
    anomaly_sequences: Dict[str, List[Tuple[int, int]]],
    window_size: int = WINDOW_SIZE,
    step_size: int = STEP_SIZE,
) -> Dict[str, Any]:
    print("[INFO] Preprocessing data: interpolation, scaling, and windowing.")

    if not train_data:
        raise ValueError("No training data provided for preprocessing.")

    train_clean = {channel: _interpolate_array(values) for channel, values in train_data.items()}
    test_clean = {channel: _interpolate_array(values) for channel, values in test_data.items()}

    scaler = StandardScaler()
    stacked_train = np.vstack(list(train_clean.values()))
    scaler.fit(stacked_train)

    train_scaled = {channel: scaler.transform(values) for channel, values in train_clean.items()}
    test_scaled = {channel: scaler.transform(values) for channel, values in test_clean.items()}

    num_features = next(iter(train_scaled.values())).shape[1]

    def aggregate_sequences(
        series_dict: Dict[str, np.ndarray],
        label_lookup: Dict[str, np.ndarray],
    ) -> Tuple[np.ndarray, np.ndarray, List[Dict[str, Any]], Dict[str, Dict[str, int]]]:
        sequences_list: List[np.ndarray] = []
        label_list: List[np.ndarray] = []
        metadata: List[Dict[str, Any]] = []
        offsets: Dict[str, Dict[str, int]] = {}
        offset_cursor = 0

        for channel in sorted(series_dict.keys()):
            series = series_dict[channel]
            labels = label_lookup[channel]
            offsets[channel] = {"offset": offset_cursor, "length": series.shape[0]}

            seq_array, seq_labels, idxs = _create_sequences(series, labels, window_size, step_size)
            if seq_array.size == 0:
                offset_cursor += series.shape[0]
                continue

            sequences_list.append(seq_array)
            label_list.append(seq_labels)
            for seq_label, (start, end) in zip(seq_labels, idxs):
                metadata.append(
                    {
                        "channel": channel,
                        "start": start,
                        "end": end,
                        "global_start": offset_cursor + start,
                        "global_end": offset_cursor + end,
                        "label": int(seq_label),
                    }
                )
            offset_cursor += series.shape[0]

        if sequences_list:
            sequences_all = np.concatenate(sequences_list, axis=0)
            labels_all = np.concatenate(label_list, axis=0) if label_list else np.zeros((sequences_all.shape[0],), dtype=int)
        else:
            sequences_all = np.empty((0, window_size, num_features), dtype=np.float32)
            labels_all = np.empty((0,), dtype=int)

        return sequences_all.astype(np.float32), labels_all.astype(int), metadata, offsets

    train_label_lookup = {
        channel: np.zeros(series.shape[0], dtype=int) for channel, series in train_scaled.items()
    }

    test_label_lookup: Dict[str, np.ndarray] = {}
    for channel, series in test_scaled.items():
        labels = np.zeros(series.shape[0], dtype=int)
        length = labels.shape[0]
        for start, end in anomaly_sequences.get(channel, []):
            start_idx = max(0, min(length - 1, int(start)))
            end_idx = max(start_idx, min(length - 1, int(end)))
            labels[start_idx : end_idx + 1] = 1
        test_label_lookup[channel] = labels

    train_sequences_all, train_seq_labels, train_metadata, train_offsets = aggregate_sequences(
        train_scaled, train_label_lookup
    )
    test_sequences, test_seq_labels, test_metadata, test_offsets = aggregate_sequences(
        test_scaled, test_label_lookup
    )

    normal_mask = train_seq_labels == 0
    train_sequences_normal = train_sequences_all[normal_mask] if normal_mask.any() else train_sequences_all

    print(f"[INFO] Training windows (normal): {train_sequences_normal.shape}")
    print(f"[INFO] Test windows: {test_sequences.shape}")

    return {
        "scaler": scaler,
        "window_size": window_size,
        "step_size": step_size,
        "train_sequences_normal": train_sequences_normal,
        "train_sequence_labels": train_seq_labels,
        "train_sequence_metadata": train_metadata,
        "train_sequence_offsets": train_offsets,
        "test_sequences": test_sequences,
        "test_sequence_labels": test_seq_labels,
        "test_sequence_metadata": test_metadata,
        "test_sequence_offsets": test_offsets,
        "train_scaled": train_scaled,
        "test_scaled": test_scaled,
    }

def build_lstm_autoencoder(input_shape: Tuple[int, int]) -> Model:
    print(f"[INFO] Building LSTM autoencoder with input shape {input_shape}.")
    inputs = Input(shape=input_shape, name="sequence_input")
    x = LSTM(128, activation="tanh", return_sequences=True, name="encoder_lstm_1")(inputs)
    x = LSTM(64, activation="tanh", return_sequences=False, name="encoder_lstm_2")(x)
    latent = RepeatVector(input_shape[0], name="latent_repeat")(x)
    x = LSTM(64, activation="tanh", return_sequences=True, name="decoder_lstm_1")(latent)
    x = LSTM(128, activation="tanh", return_sequences=True, name="decoder_lstm_2")(x)
    outputs = TimeDistributed(Dense(input_shape[1], activation="linear"), name="reconstruction")(x)

    model = Model(inputs, outputs, name="smap_lstm_autoencoder")
    model.compile(optimizer=tf.keras.optimizers.Adam(learning_rate=1e-3), loss="mse")
    model.summary(print_fn=lambda line: print(f"[MODEL] {line}"))
    return model


def train_model(
    model: Model,
    train_sequences: np.ndarray,
    epochs: int = EPOCHS,
    batch_size: int = BATCH_SIZE,
    validation_split: float = VALIDATION_SPLIT,
) -> Dict[str, List[float]]:
    print("[INFO] Training LSTM autoencoder on normal sequences.")
    num_samples = len(train_sequences)
    val_size = max(1, int(num_samples * validation_split))
    if val_size >= num_samples:
        val_size = max(1, num_samples // 5)
    train_size = num_samples - val_size
    if train_size <= 0:
        train_size = num_samples
        val_size = 0

    x_train = train_sequences[:train_size]
    x_val = train_sequences[train_size:] if val_size > 0 else None

    callbacks = []
    fit_kwargs: Dict[str, Any] = {
        "epochs": epochs,
        "batch_size": batch_size,
        "shuffle": False,
        "verbose": 1,
    }

    if x_val is not None and len(x_val) > 0:
        callbacks.append(
            EarlyStopping(
                monitor="val_loss",
                patience=10,
                restore_best_weights=True,
                mode="min",
                verbose=1,
            )
        )
        fit_kwargs["validation_data"] = (x_val, x_val)

    if callbacks:
        fit_kwargs["callbacks"] = callbacks

    history = model.fit(x_train, x_train, **fit_kwargs)
    model.save(MODEL_PATH, include_optimizer=False)
    print(f"[INFO] Saved model weights to {MODEL_PATH}")
    return history.history


def detect_anomalies(
    model: Model,
    train_sequences: np.ndarray,
    test_sequences: np.ndarray,
    test_labels: Optional[np.ndarray],
    test_metadata: List[Dict[str, Any]],
    results_dir: Path,
    threshold: Optional[float] = None,
) -> Dict[str, Any]:
    print("[INFO] Computing reconstruction errors and determining anomaly threshold.")
    train_recon = model.predict(train_sequences, verbose=0)
    train_errors = np.mean(np.square(train_sequences - train_recon), axis=(1, 2))

    mean_error = float(train_errors.mean()) if len(train_errors) else 0.0
    std_error = float(train_errors.std()) if len(train_errors) else 0.0
    if threshold is None:
        threshold = mean_error + 3 * std_error
    threshold = float(threshold)

    metrics_payload = {
        "mean_reconstruction_error": mean_error,
        "std_reconstruction_error": std_error,
        "threshold": threshold,
    }
    with (results_dir / "threshold_metrics.json").open("w", encoding="utf-8") as fp:
        json.dump(metrics_payload, fp, indent=2)

    print(f"[INFO] Threshold (mean + 3*std): {threshold:.6f}")

    test_recon = model.predict(test_sequences, verbose=0)
    test_errors = np.mean(np.square(test_sequences - test_recon), axis=(1, 2))
    pred_labels = (test_errors > threshold).astype(int)

    metrics: Dict[str, Optional[float]] = {
        "precision": None,
        "recall": None,
        "f1": None,
        "roc_auc": None,
    }
    report = ""

    if test_labels is not None and len(test_labels) == len(pred_labels) and len(test_labels) > 0:
        precision, recall, f1, _ = precision_recall_fscore_support(
            test_labels, pred_labels, average="binary", zero_division=0
        )
        metrics["precision"] = float(precision)
        metrics["recall"] = float(recall)
        metrics["f1"] = float(f1)
        try:
            roc_auc = roc_auc_score(test_labels, test_errors)
            metrics["roc_auc"] = float(roc_auc)
        except ValueError:
            metrics["roc_auc"] = None
        report = classification_report(test_labels, pred_labels, zero_division=0)
    else:
        print("[WARN] Test labels unavailable or mismatched; skipping metric computation.")

    return {
        "threshold": threshold,
        "train_errors": train_errors,
        "test_errors": test_errors,
        "pred_labels": pred_labels,
        "metrics": metrics,
        "classification_report": report,
        "test_reconstructions": test_recon,
        "test_metadata": test_metadata,
        "test_labels": test_labels,
    }

def visualize_results(
    history: Dict[str, List[float]],
    detection_results: Dict[str, Any],
    processed_data: Dict[str, Any],
    channel_names: List[str],
) -> None:
    print("[INFO] Generating visualisations.")
    if channel_names:
        print(f"[INFO] Visualising {len(channel_names)} SMAP channel(s).")

    fig, ax = plt.subplots(figsize=(12, 6), dpi=180)
    ax.plot(history.get("loss", []), label="Training Loss", color=COLORS["train_loss"], linewidth=2.3)
    if "val_loss" in history and history["val_loss"]:
        ax.plot(
            history["val_loss"],
            label="Validation Loss",
            color=COLORS["val_loss"],
            linewidth=2.3,
            linestyle="--",
        )
    ax.set_title("Training Loss Across Epochs")
    ax.set_xlabel("Epoch")
    ax.set_ylabel("MSE Loss")
    ax.grid(alpha=0.2, linestyle="--", linewidth=0.5, color="#1f1f1f")
    ax.legend()
    fig.savefig(RESULTS_DIR / "training_loss_plot.png", bbox_inches="tight")
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(12, 6), dpi=180)
    ax.hist(
        detection_results["train_errors"],
        bins=40,
        color=COLORS["hist"],
        alpha=0.75,
        label="Training Error",
    )
    ax.axvline(
        detection_results["threshold"],
        color=COLORS["threshold"],
        linestyle="--",
        linewidth=2.0,
        label="Threshold (mean + 3*std)",
    )
    ax.set_title("Training Reconstruction Error Distribution")
    ax.set_xlabel("Reconstruction Error")
    ax.set_ylabel("Frequency")
    ax.grid(alpha=0.2, linestyle="--", linewidth=0.5, color="#1f1f1f")
    ax.legend()
    fig.savefig(RESULTS_DIR / "reconstruction_error_hist.png", bbox_inches="tight")
    plt.close(fig)

    test_sequences = processed_data["test_sequences"]
    test_recon = detection_results["test_reconstructions"]
    scaler: StandardScaler = processed_data["scaler"]
    pred_labels = detection_results["pred_labels"]
    metadata = processed_data["test_sequence_metadata"]

    if len(test_sequences) > 0 and metadata:
        anomaly_indices = np.where(pred_labels == 1)[0]
        seq_idx = int(anomaly_indices[0]) if len(anomaly_indices) > 0 else 0
        seq_idx = min(seq_idx, len(test_sequences) - 1)

        meta = metadata[seq_idx]
        channel_id = meta.get("channel", "SMAP")

        actual_seq_scaled = test_sequences[seq_idx]
        recon_seq_scaled = test_recon[seq_idx]
        actual_seq = scaler.inverse_transform(actual_seq_scaled)
        recon_seq = scaler.inverse_transform(recon_seq_scaled)

        per_feature_error = np.mean(np.square(actual_seq - recon_seq), axis=0)
        feature_idx = int(np.argmax(per_feature_error)) if per_feature_error.size else 0
        feature_idx = max(0, min(feature_idx, actual_seq.shape[1] - 1))
        feature_name = f"feature_{feature_idx}"
        timesteps = np.arange(actual_seq.shape[0])

        fig, ax = plt.subplots(figsize=(12, 6), dpi=180)
        ax.plot(
            timesteps,
            actual_seq[:, feature_idx],
            color=COLORS["signal"],
            linewidth=2.4,
            label=f"Actual ({channel_id} | {feature_name})",
        )
        ax.plot(
            timesteps,
            recon_seq[:, feature_idx],
            color=COLORS["reconstruction"],
            linewidth=2.0,
            linestyle="--",
            label="Reconstruction",
        )

        if pred_labels[seq_idx] == 1:
            y_min = min(actual_seq[:, feature_idx].min(), recon_seq[:, feature_idx].min())
            y_max = max(actual_seq[:, feature_idx].max(), recon_seq[:, feature_idx].max())
            ax.fill_between(
                timesteps,
                y_min,
                y_max,
                color=COLORS["alert"],
                alpha=0.15,
                label="Detected Anomaly",
            )

        ax.set_title(f"Reconstruction Overlay | Channel {channel_id}")
        ax.set_xlabel("Timestep")
        ax.set_ylabel("Sensor Reading")
        ax.grid(alpha=0.2, linestyle="--", linewidth=0.5, color="#1f1f1f")
        ax.legend()
        fig.savefig(RESULTS_DIR / "anomaly_overlay_plot.png", bbox_inches="tight")
        plt.close(fig)

    test_errors = detection_results["test_errors"]
    threshold = detection_results["threshold"]

    if metadata:
        window_size = processed_data["window_size"]
        midpoints = [entry["global_start"] + window_size // 2 for entry in metadata]

        fig, ax = plt.subplots(figsize=(12, 6), dpi=180)
        ax.plot(midpoints, test_errors, color=COLORS["score"], linewidth=2.0, label="Reconstruction Error")
        ax.axhline(threshold, color=COLORS["threshold"], linestyle="--", linewidth=2.0, label="Threshold")

        for entry in metadata:
            if entry.get("label", 0) == 1:
                ax.axvspan(entry["global_start"], entry["global_end"], color=COLORS["alert"], alpha=0.08)

        anomaly_positions = [midpoints[i] for i, label in enumerate(pred_labels) if label == 1]
        anomaly_scores = [test_errors[i] for i, label in enumerate(pred_labels) if label == 1]
        if anomaly_positions:
            ax.scatter(
                anomaly_positions,
                anomaly_scores,
                color=COLORS["alert"],
                s=30,
                label="Detected Anomaly",
                zorder=5,
            )

        offsets = processed_data.get("test_sequence_offsets", {})
        if offsets:
            y_min, y_max = ax.get_ylim()
            for channel_id, info in offsets.items():
                boundary = info["offset"] + info["length"]
                ax.axvline(boundary, color="#1c1c28", linewidth=0.8, alpha=0.4)
                text_x = info["offset"] + info["length"] / 2
                ax.text(
                    text_x,
                    y_max * 0.98,
                    channel_id,
                    ha="center",
                    va="top",
                    fontsize=10,
                    color="#cccccc",
                    alpha=0.8,
                )

        ax.set_title("Anomaly Timeline Map")
        ax.set_xlabel("Timestep (concatenated across SMAP channels)")
        ax.set_ylabel("Reconstruction Error")
        ax.grid(alpha=0.2, linestyle="--", linewidth=0.5, color="#1f1f1f")
        ax.legend()
        fig.savefig(RESULTS_DIR / "anomaly_timeline_plot.png", bbox_inches="tight")
        plt.close(fig)

def main() -> None:
    data = load_smap_data()
    processed = preprocess_data(
        train_data=data["train"],
        test_data=data["test"],
        anomaly_sequences=data["anomaly_sequences"],
    )

    train_sequences = processed["train_sequences_normal"]
    if train_sequences.size == 0:
        raise ValueError("No training sequences available for model fitting.")

    input_shape = (train_sequences.shape[1], train_sequences.shape[2])
    model = build_lstm_autoencoder(input_shape)

    history = train_model(model, train_sequences)
    detection = detect_anomalies(
        model=model,
        train_sequences=train_sequences,
        test_sequences=processed["test_sequences"],
        test_labels=processed["test_sequence_labels"],
        test_metadata=processed["test_sequence_metadata"],
        results_dir=RESULTS_DIR,
    )
    visualize_results(history, detection, processed, data["channel_names"])

    detected_count = int(detection["pred_labels"].sum())
    total_sequences = len(detection["pred_labels"])

    print(f"[RESULT] Threshold used: {detection['threshold']:.6f}")
    print(f"[RESULT] Detected anomalies (sequences): {detected_count} / {total_sequences}")

    roc_auc = detection["metrics"].get("roc_auc")
    if roc_auc is not None:
        print(f"[RESULT] ROC AUC: {roc_auc:.4f}")

    precision = detection["metrics"].get("precision")
    recall = detection["metrics"].get("recall")
    f1 = detection["metrics"].get("f1")
    if precision is not None and recall is not None and f1 is not None:
        print(f"[RESULT] Precision: {precision:.4f}, Recall: {recall:.4f}, F1-score: {f1:.4f}")

    if detection["classification_report"]:
        print("[RESULT] Classification report:")
        print(detection["classification_report"])

    print(f"[RESULT] Saved model weights: {MODEL_PATH}")
    print(f"[RESULT] Visualisation assets written to: {RESULTS_DIR}")
    print("[INFO] Pipeline complete.")


if __name__ == "__main__":
    main()
