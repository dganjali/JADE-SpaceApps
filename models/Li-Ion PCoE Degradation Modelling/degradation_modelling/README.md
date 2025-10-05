# NASA PCoE Lithium-Ion Battery RUL Prediction Pipeline

A complete machine learning pipeline for predicting Remaining Useful Life (RUL) of lithium-ion batteries using NASA PCoE datasets.

## Overview

This pipeline implements a hybrid LSTM-GBM model for battery RUL prediction with the following components:

1. **Data Extraction**: Loads and flattens .mat files from NASA PCoE datasets
2. **SOH/RUL Computation**: Computes State of Health and Remaining Useful Life labels
3. **Feature Engineering**: Creates temporal and statistical features using sliding windows
4. **Model Training**: Trains a hybrid LSTM-GBM regression model
5. **Evaluation**: Provides comprehensive metrics and cross-validation

## Dataset Structure

The pipeline processes NASA PCoE Lithium-Ion Battery datasets located in:
```
NASA PCoE Li-Ion Battery Data/
├── 1. BatteryAgingARC-FY08Q4/
├── 2. BatteryAgingARC_25_26_27_28_P1/
├── 3. BatteryAgingARC_25-44/
├── 4. BatteryAgingARC_45_46_47_48/
├── 5. BatteryAgingARC_49_50_51_52/
└── 6. BatteryAgingARC_53_54_55_56/
```

Each folder contains .mat files (e.g., B0005.mat, B0006.mat) with battery cycle data.

## Installation

1. Install dependencies:
```bash
pip install -r requirements.txt
```

2. Activate virtual environment (if using):
```bash
# Windows
ionenv\Scripts\activate

# Linux/Mac
source ionenv/bin/activate
```

## Usage

### Run Complete Pipeline

```bash
python src/run_pipeline.py
```

### Run Individual Steps

```bash
# Data extraction only
python src/run_pipeline.py --step extract

# SOH/RUL computation only
python src/run_pipeline.py --step soh

# Feature engineering only
python src/run_pipeline.py --step features

# Model training only
python src/run_pipeline.py --step train
```

### Custom Directories

```bash
python src/run_pipeline.py --data-dir "path/to/data" --output-dir "path/to/output" --model-dir "path/to/models"
```

## Pipeline Components

### 1. Data Extraction (`parse_mat_files.py`)

- Recursively searches for .mat files
- Extracts discharge cycle data
- Computes statistical features (mean, std, min, max)
- Saves individual battery files and combined dataset

**Output**: `data/processed/all_batteries.csv`

### 2. SOH/RUL Computation (`compute_soh_rul.py`)

- Computes nominal capacity from first 5 cycles
- Calculates State of Health (SOH) = capacity / nominal_capacity
- Identifies End-of-Life (EoL) at SOH ≤ 0.8
- Computes RUL = EoL_cycle - current_cycle

**Output**: `data/processed/all_batteries_with_rul.csv`

### 3. Feature Engineering (`feature_engineering.py`)

- Rolling window statistics (mean, std, min, max, slope)
- Exponential moving averages (EMA)
- Cycle-based features (fade rates, efficiency)
- Interaction features (voltage-capacity ratios)
- Sequence features for LSTM input
- Battery-wise normalization

**Output**: `data/processed/engineered_features.csv`

### 4. Model Architecture (`modeling.py`)

**Hybrid LSTM-GBM Model**:
- **LSTM Encoder**: Processes temporal sequences with attention mechanism
- **GBM Regressor**: Uses LSTM embeddings + static features for final prediction
- **Cross-validation**: GroupKFold by battery to prevent data leakage

### 5. Training Pipeline (`train_model.py`)

- Battery-wise train/test splits
- LSTM encoder training with early stopping
- GBM training on LSTM embeddings
- Comprehensive evaluation metrics (MAE, RMSE, sMAPE)
- Model persistence and artifact saving

## Model Architecture

```
Input Sequences (capacity, voltage, temperature, SOH)
    ↓
LSTM Encoder (Bidirectional + Attention)
    ↓
LSTM Embeddings (32-dim)
    ↓
[LSTM Embeddings + Static Features]
    ↓
XGBoost Regressor
    ↓
RUL Prediction (cycles)
```

## Output Files

### Processed Data
- `data/processed/battery_<id>.csv`: Individual battery data
- `data/processed/all_batteries.csv`: Combined raw data
- `data/processed/all_batteries_with_rul.csv`: Data with SOH/RUL labels
- `data/processed/engineered_features.csv`: Final feature matrix

### Model Artifacts
- `artifacts/lstm_encoder.pth`: Trained LSTM model
- `artifacts/gbm_model.joblib`: Trained XGBoost model
- `artifacts/model_metadata.json`: Model configuration
- `artifacts/feature_schema.json`: Feature column definitions
- `artifacts/metrics.json`: Training and validation metrics

## Key Features

- **Temporal Modeling**: LSTM captures degradation trajectories
- **Statistical Features**: Rolling windows and EMA for trend analysis
- **Battery-wise Validation**: Prevents data leakage across batteries
- **Comprehensive Metrics**: MAE, RMSE, sMAPE for evaluation
- **Modular Design**: Each component can be run independently
- **Reproducible**: Fixed random seeds and version control

## Performance Metrics

The model is evaluated using:
- **MAE**: Mean Absolute Error (cycles)
- **RMSE**: Root Mean Square Error (cycles)
- **sMAPE**: Symmetric Mean Absolute Percentage Error (%)

## Dependencies

- numpy, pandas, scipy: Data processing
- scikit-learn: Machine learning utilities
- xgboost: Gradient boosting
- torch: Deep learning (LSTM)
- tqdm: Progress bars
- matplotlib, seaborn: Visualization

## Notes

- The pipeline is designed for NASA PCoE datasets but can be adapted for other battery datasets
- Sequence length and window sizes are configurable
- Model hyperparameters can be tuned in the training scripts
- The pipeline supports both CPU and GPU training (PyTorch backend)