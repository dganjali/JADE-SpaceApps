"""
Compute State of Health (SOH) and Remaining Useful Life (RUL) labels.
"""

import pandas as pd
import numpy as np
import logging
from typing import Tuple

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def compute_nominal_capacity(df: pd.DataFrame, initial_cycles: int = 5) -> float:
    """Compute nominal capacity from first few cycles."""
    initial_data = df.head(initial_cycles)
    return initial_data['capacity'].max()


def compute_soh_rul_for_battery(df: pd.DataFrame, eol_threshold: float = 0.7) -> pd.DataFrame:
    """Compute SOH and RUL for a single battery."""
    battery_id = df['battery_id'].iloc[0]
    logger.info(f"Computing SOH/RUL for battery {battery_id}")
    
    # Compute nominal capacity from first 5 cycles
    nominal_capacity = compute_nominal_capacity(df)
    
    if nominal_capacity <= 0:
        logger.warning(f"Invalid nominal capacity for {battery_id}: {nominal_capacity}")
        df['soh'] = 1.0
        df['rul_cycles'] = 0
        return df
    
    # Compute SOH
    df['soh'] = df['capacity'] / nominal_capacity
    
    # Find EoL cycle (first cycle where SOH <= threshold)
    eol_mask = df['soh'] <= eol_threshold
    eol_cycle_idx = df[eol_mask]['cycle_index'].min() if eol_mask.any() else df['cycle_index'].max()
    
    # Compute RUL
    df['rul_cycles'] = eol_cycle_idx - df['cycle_index']
    
    # Ensure RUL is non-negative
    df['rul_cycles'] = np.maximum(df['rul_cycles'], 0)
    
    logger.info(f"Battery {battery_id}: EoL at cycle {eol_cycle_idx}, max RUL: {df['rul_cycles'].max()}")
    
    return df


def compute_soh_rul_all_batteries(df: pd.DataFrame, eol_threshold: float = 0.7) -> pd.DataFrame:
    """Compute SOH and RUL for all batteries."""
    logger.info("Computing SOH and RUL for all batteries")
    
    result_dfs = []
    
    for battery_id in df['battery_id'].unique():
        battery_df = df[df['battery_id'] == battery_id].copy()
        battery_df = battery_df.sort_values('cycle_index').reset_index(drop=True)
        
        battery_df = compute_soh_rul_for_battery(battery_df, eol_threshold)
        result_dfs.append(battery_df)
    
    combined_df = pd.concat(result_dfs, ignore_index=True)
    
    logger.info(f"Computed SOH/RUL for {len(combined_df)} cycles across {combined_df['battery_id'].nunique()} batteries")
    
    return combined_df


def validate_soh_rul(df: pd.DataFrame) -> None:
    """Validate SOH and RUL computations."""
    logger.info("Validating SOH and RUL computations")
    
    # Check SOH range
    soh_stats = df['soh'].describe()
    logger.info(f"SOH statistics: {soh_stats}")
    
    if df['soh'].min() < 0 or df['soh'].max() > 1.1:
        logger.warning("SOH values outside expected range [0, 1.1]")
    
    # Check RUL range
    rul_stats = df['rul_cycles'].describe()
    logger.info(f"RUL statistics: {rul_stats}")
    
    if df['rul_cycles'].min() < 0:
        logger.warning("Negative RUL values found")
    
    # Check for each battery
    for battery_id in df['battery_id'].unique():
        battery_df = df[df['battery_id'] == battery_id].sort_values('cycle_index')
        
        # RUL should be non-increasing
        if not battery_df['rul_cycles'].is_monotonic_decreasing:
            logger.warning(f"RUL not monotonically decreasing for battery {battery_id}")
        
        # SOH should generally be decreasing
        soh_trend = battery_df['soh'].diff().mean()
        if soh_trend > 0.01:  # Allow small positive trend due to noise
            logger.warning(f"SOH trend positive for battery {battery_id}: {soh_trend:.4f}")


if __name__ == "__main__":
    # Load processed data
    input_file = "data/processed/all_batteries.csv"
    output_file = "data/processed/all_batteries_with_rul.csv"
    
    logger.info(f"Loading data from {input_file}")
    df = pd.read_csv(input_file)
    
    # Compute SOH and RUL
    df_with_rul = compute_soh_rul_all_batteries(df, eol_threshold=0.8)
    
    # Validate results
    validate_soh_rul(df_with_rul)
    
    # Save results
    df_with_rul.to_csv(output_file, index=False)
    logger.info(f"Saved results to {output_file}")
