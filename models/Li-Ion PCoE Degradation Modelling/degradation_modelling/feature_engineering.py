"""
Feature engineering for battery RUL prediction.
Implements sliding window features and normalization.
"""

import pandas as pd
import numpy as np
from typing import List, Dict
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def create_rolling_features(df: pd.DataFrame, window_size: int = 15) -> pd.DataFrame:
    """Create rolling window features for temporal sequences."""
    logger.info(f"Creating rolling features with window size {window_size}")
    
    df = df.copy()
    df = df.sort_values(['battery_id', 'cycle_index']).reset_index(drop=True)
    
    # Features to compute rolling statistics for
    feature_cols = ['capacity', 'voltage_mean', 'temperature_mean']
    
    for col in feature_cols:
        if col in df.columns:
            # Rolling statistics
            df[f'{col}_rolling_mean'] = df.groupby('battery_id')[col].rolling(
                window=window_size, min_periods=1
            ).mean().values
            
            df[f'{col}_rolling_std'] = df.groupby('battery_id')[col].rolling(
                window=window_size, min_periods=1
            ).std().values
            
            df[f'{col}_rolling_min'] = df.groupby('battery_id')[col].rolling(
                window=window_size, min_periods=1
            ).min().values
            
            df[f'{col}_rolling_max'] = df.groupby('battery_id')[col].rolling(
                window=window_size, min_periods=1
            ).max().values
            
            # Rolling slope (linear trend)
            df[f'{col}_rolling_slope'] = df.groupby('battery_id')[col].rolling(
                window=window_size, min_periods=2
            ).apply(lambda x: np.polyfit(range(len(x)), x, 1)[0] if len(x) > 1 else 0).values
    
    return df


def create_ema_features(df: pd.DataFrame, alpha: float = 0.3) -> pd.DataFrame:
    """Create exponential moving average features."""
    logger.info(f"Creating EMA features with alpha {alpha}")
    
    df = df.copy()
    feature_cols = ['capacity', 'voltage_mean', 'temperature_mean']
    
    for col in feature_cols:
        if col in df.columns:
            df[f'{col}_ema'] = df.groupby('battery_id')[col].ewm(
                alpha=alpha, adjust=False
            ).mean().values
    
    return df


def create_cycle_features(df: pd.DataFrame) -> pd.DataFrame:
    """Create cycle-based features."""
    logger.info("Creating cycle-based features")
    
    df = df.copy()
    df = df.sort_values(['battery_id', 'cycle_index']).reset_index(drop=True)
    
    # Cycle number (normalized)
    df['cycle_normalized'] = df.groupby('battery_id')['cycle_index'].transform(
        lambda x: x / x.max()
    )
    
    # Capacity fade rate
    df['capacity_fade_rate'] = df.groupby('battery_id')['capacity'].diff()
    
    # Voltage drop rate
    df['voltage_drop_rate'] = df.groupby('battery_id')['voltage_mean'].diff()
    
    # Temperature increase rate
    df['temperature_increase_rate'] = df.groupby('battery_id')['temperature_mean'].diff()
    
    # Energy efficiency (capacity / duration)
    df['energy_efficiency'] = df['capacity'] / (df['duration'] + 1e-6)
    
    return df


def create_interaction_features(df: pd.DataFrame) -> pd.DataFrame:
    """Create interaction features between different measurements."""
    logger.info("Creating interaction features")
    
    df = df.copy()
    
    # Voltage-capacity interaction
    df['voltage_capacity_ratio'] = df['voltage_mean'] / (df['capacity'] + 1e-6)
    
    # Temperature-voltage interaction
    df['temp_voltage_ratio'] = df['temperature_mean'] / (df['voltage_mean'] + 1e-6)
    
    # Current-voltage interaction
    df['current_voltage_ratio'] = df['current_mean'] / (df['voltage_mean'] + 1e-6)
    
    # Power-like features
    df['power_estimate'] = df['voltage_mean'] * df['current_mean']
    
    return df


def normalize_features_by_battery(df: pd.DataFrame, feature_cols: List[str]) -> pd.DataFrame:
    """Normalize features by battery to reduce scale bias."""
    logger.info("Normalizing features by battery")
    
    df = df.copy()
    
    for col in feature_cols:
        if col in df.columns:
            # Z-score normalization per battery
            df[f'{col}_normalized'] = df.groupby('battery_id')[col].transform(
                lambda x: (x - x.mean()) / (x.std() + 1e-6)
            )
    
    return df


def create_sequence_features(df: pd.DataFrame, sequence_length: int = 10) -> pd.DataFrame:
    """Create sequence features for LSTM input."""
    logger.info(f"Creating sequence features with length {sequence_length}")
    
    df = df.copy()
    df = df.sort_values(['battery_id', 'cycle_index']).reset_index(drop=True)
    
    # Features to include in sequences
    sequence_cols = ['capacity', 'voltage_mean', 'temperature_mean', 'soh']
    
    # Create lagged features
    for col in sequence_cols:
        if col in df.columns:
            for lag in range(1, sequence_length + 1):
                df[f'{col}_lag_{lag}'] = df.groupby('battery_id')[col].shift(lag)
    
    return df


def engineer_all_features(df: pd.DataFrame, 
                         window_size: int = 15, 
                         sequence_length: int = 10,
                         ema_alpha: float = 0.3) -> pd.DataFrame:
    """Apply all feature engineering steps."""
    logger.info("Starting comprehensive feature engineering")
    
    # Original feature columns
    original_cols = ['battery_id', 'cycle_index', 'soh', 'rul_cycles']
    
    # Create rolling features
    df = create_rolling_features(df, window_size)
    
    # Create EMA features
    df = create_ema_features(df, ema_alpha)
    
    # Create cycle features
    df = create_cycle_features(df)
    
    # Create interaction features
    df = create_interaction_features(df)
    
    # Create sequence features
    df = create_sequence_features(df, sequence_length)
    
    # Identify all feature columns (excluding original and target)
    feature_cols = [col for col in df.columns 
                    if col not in original_cols and not col.endswith('_normalized')]
    
    # Normalize features by battery
    df = normalize_features_by_battery(df, feature_cols)
    
    # Remove rows with NaN values (from lagged features)
    initial_rows = len(df)
    df = df.dropna()
    logger.info(f"Removed {initial_rows - len(df)} rows with NaN values")
    
    logger.info(f"Feature engineering complete. Final dataset: {len(df)} rows, {len(df.columns)} columns")
    
    return df


def get_feature_importance_info(df: pd.DataFrame) -> Dict:
    """Get information about feature importance and selection."""
    feature_cols = [col for col in df.columns 
                    if col not in ['battery_id', 'cycle_index', 'soh', 'rul_cycles']]
    
    return {
        'total_features': len(feature_cols),
        'feature_columns': feature_cols,
        'battery_count': df['battery_id'].nunique(),
        'total_cycles': len(df)
    }


if __name__ == "__main__":
    # Load data with RUL
    input_file = "data/processed/all_batteries_with_rul.csv"
    output_file = "data/processed/engineered_features.csv"
    
    logger.info(f"Loading data from {input_file}")
    df = pd.read_csv(input_file)
    
    # Engineer features
    df_engineered = engineer_all_features(df)
    
    # Get feature info
    feature_info = get_feature_importance_info(df_engineered)
    logger.info(f"Feature engineering summary: {feature_info}")
    
    # Save engineered features
    df_engineered.to_csv(output_file, index=False)
    logger.info(f"Saved engineered features to {output_file}")

