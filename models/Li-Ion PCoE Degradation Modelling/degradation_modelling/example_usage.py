"""
Example usage of the battery RUL prediction pipeline.
"""

import os
import sys
from pathlib import Path

# Add src to path
sys.path.append(str(Path(__file__).parent / "src"))

from run_pipeline import run_complete_pipeline
from train_model import BatteryRULTrainer
from modeling import HybridLSTMGBMModel
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def example_full_pipeline():
    """Example: Run the complete pipeline."""
    logger.info("Running complete pipeline example...")
    
    # Run the full pipeline
    results = run_complete_pipeline(
        data_dir="NASA PCoE Li-Ion Battery Data",
        output_dir="data/processed", 
        model_dir="artifacts"
    )
    
    logger.info(f"Pipeline completed! Final metrics: {results['test_metrics']}")
    return results


def example_step_by_step():
    """Example: Run pipeline step by step."""
    logger.info("Running step-by-step example...")
    
    # Step 1: Data extraction
    from run_pipeline import run_data_extraction
    raw_data_file = run_data_extraction(
        "NASA PCoE Li-Ion Battery Data", 
        "data/processed"
    )
    
    # Step 2: SOH/RUL computation
    from run_pipeline import run_soh_rul_computation
    rul_data_file = run_soh_rul_computation(raw_data_file, "data/processed")
    
    # Step 3: Feature engineering
    from run_pipeline import run_feature_engineering
    engineered_data_file = run_feature_engineering(rul_data_file, "data/processed")
    
    # Step 4: Model training
    from run_pipeline import run_model_training
    results = run_model_training(engineered_data_file, "artifacts")
    
    logger.info(f"Step-by-step completed! Final metrics: {results['test_metrics']}")
    return results


def example_load_trained_model():
    """Example: Load a trained model and make predictions."""
    logger.info("Loading trained model example...")
    
    model_dir = "artifacts"
    
    if not os.path.exists(model_dir):
        logger.error(f"Model directory {model_dir} not found. Run training first.")
        return None
    
    # Load the trained model
    model = HybridLSTMGBMModel()
    model.load_model(model_dir)
    
    logger.info("Model loaded successfully!")
    
    # Note: To make predictions, you would need to prepare input data
    # in the same format as the training data (sequences + features)
    
    return model


def example_custom_training():
    """Example: Custom training with different parameters."""
    logger.info("Custom training example...")
    
    # Initialize trainer with custom parameters
    trainer = BatteryRULTrainer(
        model_dir="custom_artifacts",
        sequence_length=15  # Custom sequence length
    )
    
    # Run training on engineered data
    data_file = "data/processed/engineered_features.csv"
    
    if os.path.exists(data_file):
        results = trainer.run_full_pipeline(data_file, use_cv=True)
        logger.info(f"Custom training completed! Metrics: {results['test_metrics']}")
        return results
    else:
        logger.error(f"Data file {data_file} not found. Run feature engineering first.")
        return None


def main():
    """Main example function."""
    print("Battery RUL Prediction Pipeline Examples")
    print("=" * 50)
    
    # Choose which example to run
    examples = {
        "1": ("Complete Pipeline", example_full_pipeline),
        "2": ("Step-by-Step", example_step_by_step), 
        "3": ("Load Trained Model", example_load_trained_model),
        "4": ("Custom Training", example_custom_training)
    }
    
    print("\nAvailable examples:")
    for key, (name, _) in examples.items():
        print(f"{key}. {name}")
    
    choice = input("\nSelect example (1-4): ").strip()
    
    if choice in examples:
        name, func = examples[choice]
        print(f"\nRunning: {name}")
        print("-" * 30)
        try:
            result = func()
            print(f"\n{name} completed successfully!")
        except Exception as e:
            print(f"\n{name} failed: {e}")
    else:
        print("Invalid choice. Please select 1-4.")


if __name__ == "__main__":
    main()
