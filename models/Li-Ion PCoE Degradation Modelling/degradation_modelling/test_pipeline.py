"""
Test script to verify the battery RUL prediction pipeline.
"""

import os
import sys
import logging
from pathlib import Path

# Add src to path
sys.path.append(str(Path(__file__).parent))

from parse_mat_files import process_single_battery
from compute_soh_rul import compute_soh_rul_for_battery
from feature_engineering import engineer_all_features
import pandas as pd
import numpy as np

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def test_data_extraction():
    """Test data extraction on a single battery."""
    logger.info("Testing data extraction...")
    
    # Find a sample .mat file
    data_dir = "NASA PCoE Li-Ion Battery Data"
    mat_files = []
    
    for root, dirs, files in os.walk(data_dir):
        for file in files:
            if file.endswith('.mat'):
                mat_files.append(os.path.join(root, file))
                break
        if mat_files:
            break
    
    if not mat_files:
        logger.error("No .mat files found for testing")
        return False
    
    # Test single battery processing
    sample_file = mat_files[0]
    logger.info(f"Testing with file: {sample_file}")
    
    df = process_single_battery(sample_file)
    
    if df.empty:
        logger.error("Data extraction failed - empty DataFrame")
        return False
    
    logger.info(f"Successfully extracted {len(df)} cycles")
    logger.info(f"Columns: {list(df.columns)}")
    
    return True


def test_soh_rul_computation():
    """Test SOH/RUL computation."""
    logger.info("Testing SOH/RUL computation...")
    
    # Create sample data
    sample_data = {
        'battery_id': ['B0005'] * 10,
        'cycle_index': list(range(10)),
        'capacity': [2.0, 1.95, 1.90, 1.85, 1.80, 1.75, 1.70, 1.65, 1.60, 1.55],
        'voltage_mean': [3.7] * 10,
        'temperature_mean': [25.0] * 10
    }
    
    df = pd.DataFrame(sample_data)
    
    # Compute SOH/RUL
    df_with_rul = compute_soh_rul_for_battery(df)
    
    if 'soh' not in df_with_rul.columns or 'rul_cycles' not in df_with_rul.columns:
        logger.error("SOH/RUL computation failed - missing columns")
        return False
    
    logger.info(f"SOH range: {df_with_rul['soh'].min():.3f} - {df_with_rul['soh'].max():.3f}")
    logger.info(f"RUL range: {df_with_rul['rul_cycles'].min():.0f} - {df_with_rul['rul_cycles'].max():.0f}")
    
    return True


def test_feature_engineering():
    """Test feature engineering."""
    logger.info("Testing feature engineering...")
    
    # Create sample data with more cycles
    n_cycles = 50
    sample_data = {
        'battery_id': ['B0005'] * n_cycles,
        'cycle_index': list(range(n_cycles)),
        'capacity': np.linspace(2.0, 1.0, n_cycles) + np.random.normal(0, 0.01, n_cycles),
        'voltage_mean': 3.7 + np.random.normal(0, 0.1, n_cycles),
        'temperature_mean': 25.0 + np.random.normal(0, 2, n_cycles),
        'soh': np.linspace(1.0, 0.5, n_cycles),
        'rul_cycles': list(range(n_cycles-1, -1, -1))
    }
    
    df = pd.DataFrame(sample_data)
    
    # Engineer features
    df_engineered = engineer_all_features(df, window_size=5, sequence_length=3)
    
    if len(df_engineered.columns) <= len(df.columns):
        logger.error("Feature engineering failed - no new features created")
        return False
    
    logger.info(f"Original features: {len(df.columns)}")
    logger.info(f"Engineered features: {len(df_engineered.columns)}")
    logger.info(f"Final dataset size: {len(df_engineered)} rows")
    
    return True


def test_model_imports():
    """Test that model components can be imported."""
    logger.info("Testing model imports...")
    
    try:
        from modeling import HybridLSTMGBMModel, compute_metrics
        from train_model import BatteryRULTrainer
        logger.info("Model imports successful")
        return True
    except ImportError as e:
        logger.error(f"Model import failed: {e}")
        return False


def run_all_tests():
    """Run all tests."""
    logger.info("Running pipeline tests...")
    
    tests = [
        ("Data Extraction", test_data_extraction),
        ("SOH/RUL Computation", test_soh_rul_computation),
        ("Feature Engineering", test_feature_engineering),
        ("Model Imports", test_model_imports)
    ]
    
    results = {}
    
    for test_name, test_func in tests:
        logger.info(f"\n--- {test_name} ---")
        try:
            results[test_name] = test_func()
        except Exception as e:
            logger.error(f"{test_name} failed with exception: {e}")
            results[test_name] = False
    
    # Summary
    logger.info("\n=== TEST SUMMARY ===")
    for test_name, passed in results.items():
        status = "PASS" if passed else "FAIL"
        logger.info(f"{test_name}: {status}")
    
    all_passed = all(results.values())
    logger.info(f"\nOverall: {'ALL TESTS PASSED' if all_passed else 'SOME TESTS FAILED'}")
    
    return all_passed


if __name__ == "__main__":
    run_all_tests()
