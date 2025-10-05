"""
Data extraction script for NASA PCoE Lithium-Ion Battery datasets.
Loads and flattens .mat files, extracting discharge cycle data.
"""

import os
import glob
import numpy as np
import pandas as pd
from scipy.io import loadmat
from tqdm import tqdm
from typing import Dict, List, Tuple
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def extract_battery_id(filename: str) -> str:
    """Extract battery ID from filename."""
    return os.path.basename(filename).replace('.mat', '')


def load_mat_file(filepath: str) -> Dict:
    """Load .mat file and return data dictionary."""
    try:
        data = loadmat(filepath)
        return data
    except Exception as e:
        logger.error(f"Error loading {filepath}: {e}")
        return None


def extract_discharge_cycles(data: Dict) -> List[Dict]:
    """Extract discharge cycles from loaded .mat data."""
    cycles = []
    
    if 'cycle' not in data:
        return cycles
    
    cycle_array = data['cycle']
    
    for i, cycle in enumerate(cycle_array[0]):
        cycle_data = cycle[0, 0]
        
        if cycle_data['type'][0] == 'discharge':
            discharge_data = cycle_data['data'][0, 0]
            
            cycle_info = {
                'cycle_index': i,
                'ambient_temperature': cycle_data['ambient_temperature'][0, 0],
                'time': cycle_data['time'][0, 0],
                'voltage_measured': discharge_data['Voltage_measured'][0],
                'current_measured': discharge_data['Current_measured'][0],
                'temperature_measured': discharge_data['Temperature_measured'][0],
                'capacity': discharge_data['Capacity'][0],
                'time_measured': discharge_data['Time'][0]
            }
            cycles.append(cycle_info)
    
    return cycles


def compute_cycle_features(cycle_data: Dict) -> Dict:
    """Compute statistical features for a single cycle."""
    voltage = cycle_data['voltage_measured']
    current = cycle_data['current_measured']
    temperature = cycle_data['temperature_measured']
    capacity = cycle_data['capacity']
    time = cycle_data['time_measured']
    
    features = {
        'voltage_mean': np.mean(voltage),
        'voltage_std': np.std(voltage),
        'voltage_min': np.min(voltage),
        'voltage_max': np.max(voltage),
        'current_mean': np.mean(current),
        'current_std': np.std(current),
        'current_min': np.min(current),
        'current_max': np.max(current),
        'temperature_mean': np.mean(temperature),
        'temperature_std': np.std(temperature),
        'temperature_min': np.min(temperature),
        'temperature_max': np.max(temperature),
        'capacity': capacity[0] if len(capacity) > 0 else 0,
        'duration': np.max(time) - np.min(time) if len(time) > 0 else 0
    }
    
    return features


def process_single_battery(filepath: str) -> pd.DataFrame:
    """Process a single battery .mat file."""
    battery_id = extract_battery_id(filepath)
    logger.info(f"Processing battery {battery_id}")
    
    data = load_mat_file(filepath)
    if data is None:
        return pd.DataFrame()
    
    discharge_cycles = extract_discharge_cycles(data)
    if not discharge_cycles:
        logger.warning(f"No discharge cycles found in {battery_id}")
        return pd.DataFrame()
    
    cycle_features = []
    for cycle in discharge_cycles:
        features = compute_cycle_features(cycle)
        features['battery_id'] = battery_id
        features['cycle_index'] = cycle['cycle_index']
        cycle_features.append(features)
    
    return pd.DataFrame(cycle_features)


def process_all_batteries(data_dir: str, output_dir: str) -> None:
    """Process all .mat files in the data directory."""
    os.makedirs(output_dir, exist_ok=True)
    
    # Find all .mat files
    mat_files = glob.glob(os.path.join(data_dir, "**", "*.mat"), recursive=True)
    logger.info(f"Found {len(mat_files)} .mat files")
    
    all_batteries = []
    
    for filepath in tqdm(mat_files, desc="Processing batteries"):
        battery_df = process_single_battery(filepath)
        
        if not battery_df.empty:
            battery_id = battery_df['battery_id'].iloc[0]
            output_file = os.path.join(output_dir, f"battery_{battery_id}.csv")
            battery_df.to_csv(output_file, index=False)
            all_batteries.append(battery_df)
            logger.info(f"Saved {len(battery_df)} cycles for {battery_id}")
    
    # Combine all batteries
    if all_batteries:
        combined_df = pd.concat(all_batteries, ignore_index=True)
        combined_file = os.path.join(output_dir, "all_batteries.csv")
        combined_df.to_csv(combined_file, index=False)
        logger.info(f"Saved combined dataset with {len(combined_df)} total cycles")
    else:
        logger.error("No battery data processed successfully")


if __name__ == "__main__":
    data_dir = "NASA PCoE Li-Ion Battery Data"
    output_dir = "data/processed"
    
    process_all_batteries(data_dir, output_dir)
