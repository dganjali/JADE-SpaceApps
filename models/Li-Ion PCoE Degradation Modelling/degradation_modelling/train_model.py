"""
Complete training pipeline for hybrid LSTM-GBM battery RUL prediction.
"""

import os
import json
import numpy as np
import pandas as pd
from sklearn.model_selection import GroupKFold
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import mean_absolute_error, mean_squared_error
import torch
import logging
from typing import Dict, Tuple, List
import joblib
from tqdm import tqdm

from modeling import HybridLSTMGBMModel, compute_metrics
from feature_engineering import engineer_all_features

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class BatteryRULTrainer:
    """Complete training pipeline for battery RUL prediction."""
    
    def __init__(self, model_dir: str = "artifacts", sequence_length: int = 15, 
                 window_size: int = 20, lstm_hidden_size: int = 128, 
                 lstm_layers: int = 3, lstm_output_size: int = 64):
        self.model_dir = model_dir
        self.sequence_length = sequence_length
        self.window_size = window_size
        self.lstm_hidden_size = lstm_hidden_size
        self.lstm_layers = lstm_layers
        self.lstm_output_size = lstm_output_size
        self.model = None
        self.scalers = {}
        self.feature_columns = None
        self.sequence_columns = None
        
        os.makedirs(model_dir, exist_ok=True)
    
    def load_and_prepare_data(self, data_file: str) -> pd.DataFrame:
        """Load and prepare data for training with quality improvements."""
        logger.info(f"Loading data from {data_file}")
        
        df = pd.read_csv(data_file)
        logger.info(f"Loaded {len(df)} cycles from {df['battery_id'].nunique()} batteries")
        
        # Data quality improvements
        df = self._apply_data_quality_filters(df)
        
        # Engineer features with improved parameters
        df_engineered = engineer_all_features(df, 
                                            window_size=self.window_size, 
                                            sequence_length=self.sequence_length)
        
        # Define sequence and static feature columns
        self.sequence_columns = ['capacity', 'voltage_mean', 'temperature_mean', 'soh']
        self.feature_columns = [col for col in df_engineered.columns 
                               if col not in ['battery_id', 'cycle_index', 'rul_cycles'] + self.sequence_columns]
        
        logger.info(f"Sequence columns: {self.sequence_columns}")
        logger.info(f"Feature columns: {len(self.feature_columns)}")
        
        return df_engineered
    
    def _apply_data_quality_filters(self, df: pd.DataFrame) -> pd.DataFrame:
        """Apply data quality filters to improve model performance."""
        logger.info("Applying data quality filters")
        
        initial_count = len(df)
        
        # Filter out batteries with too few cycles (< 50 cycles)
        battery_cycle_counts = df.groupby('battery_id').size()
        valid_batteries = battery_cycle_counts[battery_cycle_counts >= 50].index
        df = df[df['battery_id'].isin(valid_batteries)]
        
        # Remove cycles with unrealistic capacity values
        df = df[(df['capacity'] > 0) & (df['capacity'] < 5.0)]  # Reasonable capacity range
        
        # Remove cycles with extreme voltage values
        df = df[(df['voltage_mean'] > 2.0) & (df['voltage_mean'] < 4.5)]
        
        # Remove cycles with extreme temperature values
        df = df[(df['temperature_mean'] > 15) & (df['temperature_mean'] < 50)]
        
        # Smooth capacity readings with moving average (reduce noise)
        df = df.sort_values(['battery_id', 'cycle_index'])
        df['capacity_smoothed'] = df.groupby('battery_id')['capacity'].transform(
            lambda x: x.rolling(window=3, center=True).mean().fillna(x)
        )
        df['capacity'] = df['capacity_smoothed']
        df = df.drop('capacity_smoothed', axis=1)
        
        filtered_count = len(df)
        logger.info(f"Data quality filtering: {initial_count} -> {filtered_count} cycles ({((initial_count-filtered_count)/initial_count)*100:.1f}% removed)")
        
        return df
    
    def prepare_sequences_and_features(self, df: pd.DataFrame) -> Tuple[np.ndarray, np.ndarray, np.ndarray, List[str]]:
        """Prepare sequences and features for training."""
        logger.info("Preparing sequences and features")
        
        sequences = []
        features = []
        targets = []
        battery_ids = []
        
        for battery_id in tqdm(df['battery_id'].unique(), desc="Preparing sequences"):
            battery_df = df[df['battery_id'] == battery_id].sort_values('cycle_index')
            
            if len(battery_df) < self.sequence_length:
                continue
            
            # Extract sequence data
            sequence_data = battery_df[self.sequence_columns].values
            
            # Create sliding windows
            for i in range(self.sequence_length, len(battery_df)):
                sequence = sequence_data[i-self.sequence_length:i]
                target = battery_df.iloc[i]['rul_cycles']
                
                # Extract current cycle features and ensure numeric types
                current_features = battery_df.iloc[i][self.feature_columns].values
                
                # Convert to numeric, replacing non-numeric values with NaN, then fill with 0
                current_features = pd.Series(current_features).apply(pd.to_numeric, errors='coerce').fillna(0).values
                
                sequences.append(sequence)
                features.append(current_features)
                targets.append(target)
                battery_ids.append(battery_id)
        
        sequences = np.array(sequences)
        features = np.array(features, dtype=np.float64)
        targets = np.array(targets)
        
        logger.info(f"Prepared {len(sequences)} sequences")
        
        return sequences, features, targets, battery_ids
    
    def train_validation_split(self, sequences: np.ndarray, features: np.ndarray, 
                              targets: np.ndarray, battery_ids: List[str], 
                              test_size: float = 0.2) -> Tuple:
        """Split data into train/validation sets by battery."""
        logger.info("Splitting data by battery")
        
        unique_batteries = list(set(battery_ids))
        n_test_batteries = int(len(unique_batteries) * test_size)
        
        # Randomly select test batteries
        np.random.seed(42)
        test_batteries = np.random.choice(unique_batteries, n_test_batteries, replace=False)
        
        # Create masks
        test_mask = np.array([bid in test_batteries for bid in battery_ids])
        train_mask = ~test_mask
        
        # Split data
        X_seq_train, X_seq_test = sequences[train_mask], sequences[test_mask]
        X_feat_train, X_feat_test = features[train_mask], features[test_mask]
        y_train, y_test = targets[train_mask], targets[test_mask]
        
        logger.info(f"Train: {len(X_seq_train)} sequences, Test: {len(X_seq_test)} sequences")
        
        return X_seq_train, X_seq_test, X_feat_train, X_feat_test, y_train, y_test
    
    def train_model(self, X_seq_train: np.ndarray, X_feat_train: np.ndarray, 
                   y_train: np.ndarray, X_seq_test: np.ndarray, 
                   X_feat_test: np.ndarray, y_test: np.ndarray) -> Dict:
        """Train the hybrid LSTM-GBM model."""
        logger.info("Training hybrid LSTM-GBM model")
        
        # Initialize model
        self.model = HybridLSTMGBMModel(sequence_length=self.sequence_length)
        
        # Train LSTM encoder
        logger.info("Training LSTM encoder...")
        lstm_history = self.model.train_lstm_encoder(
            X_seq_train, X_feat_train, y_train,
            epochs=100, batch_size=32, learning_rate=0.0005
        )
        
        # Extract LSTM features for GBM
        logger.info("Extracting LSTM features...")
        lstm_features_train = self.model.extract_lstm_features(X_seq_train)
        lstm_features_test = self.model.extract_lstm_features(X_seq_test)
        
        # Train GBM
        logger.info("Training GBM...")
        gbm_history = self.model.train_gbm(
            lstm_features_train, X_feat_train, y_train
        )
        
        # Make predictions
        logger.info("Making predictions...")
        y_pred_train = self.model.predict(X_seq_train, X_feat_train)
        y_pred_test = self.model.predict(X_seq_test, X_feat_test)
        
        # Compute metrics
        train_metrics = compute_metrics(y_train, y_pred_train)
        test_metrics = compute_metrics(y_test, y_pred_test)
        
        logger.info(f"Train metrics: {train_metrics}")
        logger.info(f"Test metrics: {test_metrics}")
        
        return {
            'train_metrics': train_metrics,
            'test_metrics': test_metrics,
            'lstm_history': lstm_history,
            'gbm_history': gbm_history,
            'predictions': {
                'train': {'true': y_train, 'pred': y_pred_train},
                'test': {'true': y_test, 'pred': y_pred_test}
            }
        }
    
    def cross_validate(self, sequences: np.ndarray, features: np.ndarray, 
                      targets: np.ndarray, battery_ids: List[str], 
                      n_folds: int = 5) -> Dict:
        """Perform cross-validation by battery."""
        logger.info(f"Performing {n_folds}-fold cross-validation")
        
        group_kfold = GroupKFold(n_splits=n_folds)
        cv_results = []
        
        for fold, (train_idx, val_idx) in enumerate(group_kfold.split(sequences, targets, battery_ids)):
            logger.info(f"Fold {fold + 1}/{n_folds}")
            
            # Split data
            X_seq_train, X_seq_val = sequences[train_idx], sequences[val_idx]
            X_feat_train, X_feat_val = features[train_idx], features[val_idx]
            y_train, y_val = targets[train_idx], targets[val_idx]
            
            # Train model for this fold
            fold_model = HybridLSTMGBMModel(sequence_length=self.sequence_length)
            
            # Train LSTM
            fold_model.train_lstm_encoder(X_seq_train, X_feat_train, y_train, epochs=30)
            
            # Extract LSTM features
            lstm_features_train = fold_model.extract_lstm_features(X_seq_train)
            lstm_features_val = fold_model.extract_lstm_features(X_seq_val)
            
            # Train GBM
            fold_model.train_gbm(lstm_features_train, X_feat_train, y_train)
            
            # Predict
            y_pred_val = fold_model.predict(X_seq_val, X_feat_val)
            
            # Compute metrics
            fold_metrics = compute_metrics(y_val, y_pred_val)
            cv_results.append(fold_metrics)
            
            logger.info(f"Fold {fold + 1} metrics: {fold_metrics}")
        
        # Aggregate results
        cv_metrics = {}
        for metric in cv_results[0].keys():
            cv_metrics[f'cv_{metric}_mean'] = np.mean([r[metric] for r in cv_results])
            cv_metrics[f'cv_{metric}_std'] = np.std([r[metric] for r in cv_results])
        
        return cv_metrics
    
    def save_results(self, results: Dict):
        """Save training results and model artifacts."""
        logger.info("Saving results and artifacts")
        
        # Save model
        self.model.save_model(self.model_dir)
        
        # Save feature schema
        feature_schema = {
            'sequence_columns': self.sequence_columns,
            'feature_columns': self.feature_columns,
            'sequence_length': self.sequence_length
        }
        
        with open(os.path.join(self.model_dir, 'feature_schema.json'), 'w') as f:
            json.dump(feature_schema, f, indent=2)
        
        # Save metrics (convert numpy arrays to lists for JSON serialization)
        def convert_numpy(obj):
            if isinstance(obj, np.ndarray):
                return obj.tolist()
            elif isinstance(obj, np.integer):
                return int(obj)
            elif isinstance(obj, np.floating):
                return float(obj)
            elif isinstance(obj, dict):
                return {key: convert_numpy(value) for key, value in obj.items()}
            elif isinstance(obj, list):
                return [convert_numpy(item) for item in obj]
            else:
                return obj
        
        results_serializable = convert_numpy(results)
        
        with open(os.path.join(self.model_dir, 'metrics.json'), 'w') as f:
            json.dump(results_serializable, f, indent=2)
        
        logger.info(f"Results saved to {self.model_dir}")
    
    def run_full_pipeline(self, data_file: str, use_cv: bool = True) -> Dict:
        """Run the complete training pipeline."""
        logger.info("Starting full training pipeline")
        
        # Load and prepare data
        df = self.load_and_prepare_data(data_file)
        
        # Prepare sequences and features
        sequences, features, targets, battery_ids = self.prepare_sequences_and_features(df)
        
        if use_cv:
            # Cross-validation
            cv_results = self.cross_validate(sequences, features, targets, battery_ids)
            logger.info(f"Cross-validation results: {cv_results}")
        
        # Train-test split
        X_seq_train, X_seq_test, X_feat_train, X_feat_test, y_train, y_test = \
            self.train_validation_split(sequences, features, targets, battery_ids)
        
        # Train model
        results = self.train_model(X_seq_train, X_feat_train, y_train, 
                                 X_seq_test, X_feat_test, y_test)
        
        # Save results
        self.save_results(results)
        
        return results


def main():
    """Main training function."""
    # Set random seeds for reproducibility
    np.random.seed(42)
    torch.manual_seed(42)
    
    # Initialize trainer
    trainer = BatteryRULTrainer(model_dir="artifacts", sequence_length=10)
    
    # Run pipeline
    data_file = "data/processed/all_batteries_with_rul.csv"
    results = trainer.run_full_pipeline(data_file, use_cv=True)
    
    logger.info("Training pipeline completed successfully!")
    logger.info(f"Final test metrics: {results['test_metrics']}")


if __name__ == "__main__":
    main()
