# Li-Ion PCoE Degradation Modelling

This module builds a remaining useful life (RUL) regressor on top of the NASA PCoE lithium-ion battery aging dataset. It transforms the MATLAB telemetry files (e.g. `B0005.mat`) into cycle-level features, engineers degradation indicators, and trains a gradient-boosted tree model with battery-aware cross-validation.

## Dataset setup

1. Download the dataset archive from Kaggle: [Lithium-Ion Battery Degradation Dataset](https://www.kaggle.com/datasets/programmer3/lithium-ion-battery-degradation-dataset).
2. Extract the contents so the individual MATLAB files (`B0005.mat`, `B0006.mat`, `B0007.mat`, `B0018.mat`, `B0025.mat`, etc.) live in a directory on disk.
3. Point the training script to that directory via `--data-root`. No network access is required at runtime once the files are present.

> Tip: if you prefer automated downloads, place a Kaggle API token in `~/.kaggle/kaggle.json` and call the [Kaggle CLI](https://github.com/Kaggle/kaggle-api). Keeping the dataset local avoids API credentials inside the repository.

## Environment

```bash
python -m venv .venv
source .venv/bin/activate  # Windows: .venv\Scripts\activate
pip install -r "models/Li-Ion PCoE Degradation Modelling/requirements.txt"
export PYTHONPATH="models/Li-Ion PCoE Degradation Modelling/src:${PYTHONPATH}"
```

On Windows PowerShell:

```powershell
python -m venv .venv
.venv\Scripts\Activate.ps1
pip install -r "models/Li-Ion PCoE Degradation Modelling/requirements.txt"
$env:PYTHONPATH = "models/Li-Ion PCoE Degradation Modelling/src;${env:PYTHONPATH}"
```

## Training

```bash
python -m battery_rul.train \
    --data-root /path/to/nasa_pcoe_mat_files \
    --output-dir models/Li-Ion\ PCoE\ Degradation\ Modelling/artifacts \
    --threshold-fraction 0.7 \
    --validation-fraction 0.25
```

Key features of the pipeline:
- MATLAB ingestion -> tabular summary statistics per discharge cycle (`data_loader.py`).
- Degradation features & RUL targets with configurable end-of-life thresholds (`feature_engineering.py`).
- Gradient boosted regressor with battery-aware cross-validation and metrics export (`modeling.py`, `train.py`).

Artifacts saved under `--output-dir`:
- `battery_rul_model.joblib`: trained scikit-learn pipeline (preprocessing + model).
- `metrics.json`: cross-validation and holdout scores.
- `feature_columns.json`: list of feature names baked into the pipeline.
- `training_dataset.csv`: engineered dataset for further analysis or visualization.

## Extending the pipeline

- Adjust `TargetConfig` in `feature_engineering.py` to experiment with different end-of-life definitions.
- Swap the regressor or tune hyper-parameters inside `modeling.build_regression_pipeline`.
- Add more diagnostics (e.g. SHAP values, calibration plots) by building on the saved dataset artifact.

