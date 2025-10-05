"""
Quick training script with robust error handling for battery RUL prediction.
"""

import pandas as pd
import numpy as np
import os
import json
import logging
from pathlib import Path
import sys

# Add current directory to path
sys.path.append(str(Path(__file__).parent))

from modeling import HybridLSTMGBMModel, compute_metrics
from feature_engineering import engineer_all_features

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def quick_train():
    """Quick training with robust error handling."""
    
    # Check if data exists
    data_file = "data/processed/all_batteries_with_rul.csv"
    if not os.path.exists(data_file):
        logger.error(f"Data file not found: {data_file}")
        return
    
    logger.info("Loading data...")
    df = pd.read_csv(data_file)
    logger.info(f"Loaded {len(df)} cycles from {df['battery_id'].nunique()} batteries")
    
    # Simple data quality filters
    logger.info("Applying data quality filters...")
    initial_count = len(df)
    
    # Remove batteries with too few cycles
    battery_counts = df.groupby('battery_id').size()
    valid_batteries = battery_counts[battery_counts >= 20].index
    df = df[df['battery_id'].isin(valid_batteries)]
    
    # Remove extreme outliers
    df = df[(df['capacity'] > 0) & (df['capacity'] < 5.0)]
    df = df[(df['voltage_mean'] > 2.0) & (df['voltage_mean'] < 4.5)]
    df = df[(df['temperature_mean'] > 15) & (df['temperature_mean'] < 50)]
    
    filtered_count = len(df)
    logger.info(f"Data filtering: {initial_count} -> {filtered_count} cycles")
    
    if filtered_count < 100:
        logger.error("Not enough data after filtering")
        return
    
    # Simple feature engineering
    logger.info("Engineering features...")
    try:
        df_engineered = engineer_all_features(df, window_size=10, sequence_length=5)
    except Exception as e:
        logger.error(f"Feature engineering failed: {e}")
        return
    
    # Prepare sequences
    logger.info("Preparing sequences...")
    sequences = []
    features = []
    targets = []
    battery_ids = []
    
    sequence_cols = ['capacity', 'voltage_mean', 'temperature_mean', 'soh']
    feature_cols = [col for col in df_engineered.columns 
                   if col not in ['battery_id', 'cycle_index', 'rul_cycles'] + sequence_cols]
    
    for battery_id in df_engineered['battery_id'].unique():
        battery_df = df_engineered[df_engineered['battery_id'] == battery_id].sort_values('cycle_index')
        
        if len(battery_df) < 5:
            continue
        
        sequence_data = battery_df[sequence_cols].values
        
        for i in range(5, len(battery_df)):
            sequence = sequence_data[i-5:i]
            target = battery_df.iloc[i]['rul_cycles']
            
            # Simple feature extraction
            current_features = battery_df.iloc[i][feature_cols].values
            current_features = pd.Series(current_features).apply(pd.to_numeric, errors='coerce').fillna(0).values
            
            sequences.append(sequence)
            features.append(current_features)
            targets.append(target)
            battery_ids.append(battery_id)
    
    sequences = np.array(sequences)
    features = np.array(features, dtype=np.float64)
    targets = np.array(targets)
    
    logger.info(f"Prepared {len(sequences)} sequences")
    
    if len(sequences) < 50:
        logger.error("Not enough sequences for training")
        return
    
    # Simple train/test split
    n_train = int(0.8 * len(sequences))
    indices = np.random.permutation(len(sequences))
    
    train_idx = indices[:n_train]
    test_idx = indices[n_train:]
    
    X_seq_train, X_feat_train, y_train = sequences[train_idx], features[train_idx], targets[train_idx]
    X_seq_test, X_feat_test, y_test = sequences[test_idx], features[test_idx], targets[test_idx]
    
    logger.info(f"Train: {len(X_seq_train)}, Test: {len(X_seq_test)}")
    
    # Train model
    logger.info("Training model...")
    try:
        model = HybridLSTMGBMModel(sequence_length=5, lstm_hidden_size=32, lstm_layers=2, lstm_output_size=16)
        
        # Train LSTM with fewer epochs
        model.train_lstm_encoder(X_seq_train, X_feat_train, y_train, epochs=20, learning_rate=0.001)
        
        # Extract features and train GBM
        lstm_features = model.extract_lstm_features(X_seq_train)
        model.train_gbm(lstm_features, X_feat_train, y_train)
        
        # Make predictions
        y_pred = model.predict(X_seq_test, X_feat_test)
        
        # Calculate metrics
        metrics = compute_metrics(y_test, y_pred)
        logger.info(f"Test metrics: {metrics}")
        
        # Save model
        os.makedirs("artifacts", exist_ok=True)
        model.save_model("artifacts")
        
        # Save results
        with open("artifacts/metrics.json", "w") as f:
            json.dump(metrics, f, indent=2)
        
        logger.info("Training completed successfully!")
        logger.info(f"Final metrics: MAE={metrics['mae']:.2f}, RMSE={metrics['rmse']:.2f}")
        
    except Exception as e:
        logger.error(f"Training failed: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    quick_train()
