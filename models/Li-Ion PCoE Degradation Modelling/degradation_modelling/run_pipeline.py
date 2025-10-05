"""
Main pipeline script to run the complete battery RUL prediction pipeline.
"""

import os
import sys
import logging
from pathlib import Path

# Add src to path
sys.path.append(str(Path(__file__).parent))

from parse_mat_files import process_all_batteries
from compute_soh_rul import compute_soh_rul_all_batteries, validate_soh_rul
from feature_engineering import engineer_all_features
from train_model import BatteryRULTrainer

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


def run_data_extraction(data_dir: str, output_dir: str) -> str:
    """Run data extraction from .mat files."""
    logger.info("=== STEP 1: Data Extraction ===")
    
    os.makedirs(output_dir, exist_ok=True)
    process_all_batteries(data_dir, output_dir)
    
    output_file = os.path.join(output_dir, "all_batteries.csv")
    logger.info(f"Data extraction complete. Output: {output_file}")
    return output_file


def run_soh_rul_computation(input_file: str, output_dir: str) -> str:
    """Run SOH and RUL computation."""
    logger.info("=== STEP 2: SOH and RUL Computation ===")
    
    import pandas as pd
    
    # Load data
    df = pd.read_csv(input_file)
    logger.info(f"Loaded {len(df)} cycles from {df['battery_id'].nunique()} batteries")
    
    # Compute SOH and RUL
    df_with_rul = compute_soh_rul_all_batteries(df, eol_threshold=0.8)
    
    # Validate results
    validate_soh_rul(df_with_rul)
    
    # Save results
    output_file = os.path.join(output_dir, "all_batteries_with_rul.csv")
    df_with_rul.to_csv(output_file, index=False)
    logger.info(f"SOH/RUL computation complete. Output: {output_file}")
    return output_file


def run_feature_engineering(input_file: str, output_dir: str) -> str:
    """Run feature engineering."""
    logger.info("=== STEP 3: Feature Engineering ===")
    
    import pandas as pd
    
    # Load data
    df = pd.read_csv(input_file)
    logger.info(f"Loaded {len(df)} cycles for feature engineering")
    
    # Engineer features
    df_engineered = engineer_all_features(df, window_size=15, sequence_length=10)
    
    # Save results
    output_file = os.path.join(output_dir, "engineered_features.csv")
    df_engineered.to_csv(output_file, index=False)
    logger.info(f"Feature engineering complete. Output: {output_file}")
    return output_file


def run_model_training(input_file: str, model_dir: str) -> dict:
    """Run model training."""
    logger.info("=== STEP 4: Model Training ===")
    
    # Initialize trainer
    trainer = BatteryRULTrainer(
        model_dir=model_dir,
        sequence_length=15,
        window_size=20,
        lstm_hidden_size=128,
        lstm_layers=3,
        lstm_output_size=64
    )
    
    # Run full pipeline
    results = trainer.run_full_pipeline(input_file, use_cv=True)
    
    logger.info("Model training complete!")
    return results


def run_complete_pipeline(data_dir: str = "NASA PCoE Li-Ion Battery Data", 
                         output_dir: str = "data/processed",
                         model_dir: str = "artifacts") -> dict:
    """Run the complete end-to-end pipeline."""
    logger.info("Starting complete battery RUL prediction pipeline")
    
    # Step 1: Data extraction
    raw_data_file = run_data_extraction(data_dir, output_dir)
    
    # Step 2: SOH and RUL computation
    rul_data_file = run_soh_rul_computation(raw_data_file, output_dir)
    
    # Step 3: Feature engineering
    engineered_data_file = run_feature_engineering(rul_data_file, output_dir)
    
    # Step 4: Model training
    results = run_model_training(engineered_data_file, model_dir)
    
    logger.info("=== PIPELINE COMPLETE ===")
    logger.info(f"Final test metrics: {results['test_metrics']}")
    
    return results


def main():
    """Main function to run the pipeline."""
    import argparse
    
    parser = argparse.ArgumentParser(description="Battery RUL Prediction Pipeline")
    parser.add_argument("--data-dir", default="NASA PCoE Li-Ion Battery Data",
                       help="Directory containing .mat files")
    parser.add_argument("--output-dir", default="data/processed",
                       help="Output directory for processed data")
    parser.add_argument("--model-dir", default="artifacts",
                       help="Directory for model artifacts")
    parser.add_argument("--step", choices=["extract", "soh", "features", "train", "all"],
                       default="all", help="Which step to run")
    
    args = parser.parse_args()
    
    if args.step == "extract":
        run_data_extraction(args.data_dir, args.output_dir)
    elif args.step == "soh":
        input_file = os.path.join(args.output_dir, "all_batteries.csv")
        run_soh_rul_computation(input_file, args.output_dir)
    elif args.step == "features":
        input_file = os.path.join(args.output_dir, "all_batteries_with_rul.csv")
        run_feature_engineering(input_file, args.output_dir)
    elif args.step == "train":
        input_file = os.path.join(args.output_dir, "engineered_features.csv")
        run_model_training(input_file, args.model_dir)
    else:  # all
        run_complete_pipeline(args.data_dir, args.output_dir, args.model_dir)


if __name__ == "__main__":
    main()
