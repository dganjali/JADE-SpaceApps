"""
Hybrid LSTM-GBM model architecture for battery RUL prediction.
"""

import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
import numpy as np
import pandas as pd
from sklearn.model_selection import GroupKFold
from sklearn.metrics import mean_absolute_error, mean_squared_error
import xgboost as xgb
import joblib
import json
import logging
from typing import Tuple, Dict, List
import os

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class BatterySequenceDataset(Dataset):
    """Dataset for battery sequence data."""
    
    def __init__(self, sequences: np.ndarray, features: np.ndarray, targets: np.ndarray):
        self.sequences = torch.FloatTensor(sequences)
        self.features = torch.FloatTensor(features)
        self.targets = torch.FloatTensor(targets)
    
    def __len__(self):
        return len(self.sequences)
    
    def __getitem__(self, idx):
        return self.sequences[idx], self.features[idx], self.targets[idx]


class TemporalAttention(nn.Module):
    """Temporal attention mechanism for LSTM outputs."""
    
    def __init__(self, hidden_size: int, attention_size: int = 64):
        super(TemporalAttention, self).__init__()
        self.attention_linear = nn.Linear(hidden_size, attention_size)
        self.context_vector = nn.Linear(attention_size, 1, bias=False)
        
    def forward(self, lstm_outputs):
        # lstm_outputs: (batch_size, seq_len, hidden_size)
        attention_weights = self.attention_linear(lstm_outputs)  # (batch_size, seq_len, attention_size)
        attention_weights = torch.tanh(attention_weights)
        attention_weights = self.context_vector(attention_weights)  # (batch_size, seq_len, 1)
        attention_weights = torch.softmax(attention_weights, dim=1)  # (batch_size, seq_len, 1)
        
        # Apply attention weights
        attended_output = torch.sum(attention_weights * lstm_outputs, dim=1)  # (batch_size, hidden_size)
        return attended_output, attention_weights.squeeze(-1)


class LSTMBatteryEncoder(nn.Module):
    """Enhanced LSTM encoder with temporal attention for battery degradation sequences."""
    
    def __init__(self, input_size: int, hidden_size: int = 128, num_layers: int = 3, 
                 dropout: float = 0.3, output_size: int = 64):
        super(LSTMBatteryEncoder, self).__init__()
        
        self.hidden_size = hidden_size
        self.num_layers = num_layers
        
        # Enhanced LSTM with layer normalization
        self.lstm = nn.LSTM(
            input_size=input_size,
            hidden_size=hidden_size,
            num_layers=num_layers,
            dropout=dropout if num_layers > 1 else 0,
            batch_first=True,
            bidirectional=True
        )
        
        # Layer normalization for stability
        self.layer_norm = nn.LayerNorm(hidden_size * 2)
        
        # Temporal attention mechanism
        self.temporal_attention = TemporalAttention(hidden_size * 2)
        
        # Multi-head self-attention for sequence relationships
        self.self_attention = nn.MultiheadAttention(
            embed_dim=hidden_size * 2,
            num_heads=8,
            dropout=dropout,
            batch_first=True
        )
        
        # Enhanced output projection with residual connection
        self.output_projection = nn.Sequential(
            nn.Linear(hidden_size * 2, hidden_size),
            nn.LayerNorm(hidden_size),
            nn.ReLU(),
            nn.Dropout(dropout),
            nn.Linear(hidden_size, hidden_size // 2),
            nn.LayerNorm(hidden_size // 2),
            nn.ReLU(),
            nn.Dropout(dropout),
            nn.Linear(hidden_size // 2, output_size)
        )
        
        # Initialize weights
        self._init_weights()
    
    def _init_weights(self):
        """Initialize weights for better training stability."""
        for name, param in self.named_parameters():
            if 'weight' in name and param.dim() >= 2:
                nn.init.xavier_uniform_(param)
            elif 'bias' in name:
                nn.init.zeros_(param)
    
    def forward(self, x):
        # LSTM forward pass
        lstm_out, (hidden, cell) = self.lstm(x)
        
        # Layer normalization
        lstm_out = self.layer_norm(lstm_out)
        
        # Self-attention for sequence relationships
        attn_out, _ = self.self_attention(lstm_out, lstm_out, lstm_out)
        
        # Residual connection
        attn_out = attn_out + lstm_out
        
        # Temporal attention with learnable weights
        attended_output, attention_weights = self.temporal_attention(attn_out)
        
        # Output projection
        output = self.output_projection(attended_output)
        
        return output


class HybridLSTMGBMModel:
    """Hybrid LSTM-GBM model for battery RUL prediction."""
    
    def __init__(self, sequence_length: int = 10, lstm_hidden_size: int = 64, 
                 lstm_layers: int = 2, lstm_output_size: int = 32):
        self.sequence_length = sequence_length
        self.lstm_hidden_size = lstm_hidden_size
        self.lstm_layers = lstm_layers
        self.lstm_output_size = lstm_output_size
        
        self.lstm_encoder = None
        self.gbm_model = None
        self.feature_scaler = None
        self.sequence_scaler = None
        
    def prepare_sequences(self, df: pd.DataFrame, sequence_cols: List[str]) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Prepare sequences for LSTM input."""
        logger.info("Preparing sequences for LSTM")
        
        sequences = []
        features = []
        targets = []
        
        for battery_id in df['battery_id'].unique():
            battery_df = df[df['battery_id'] == battery_id].sort_values('cycle_index')
            
            if len(battery_df) < self.sequence_length:
                continue
            
            # Extract sequence data
            sequence_data = battery_df[sequence_cols].values
            
            # Create sliding windows
            for i in range(self.sequence_length, len(battery_df)):
                sequence = sequence_data[i-self.sequence_length:i]
                target = battery_df.iloc[i]['rul_cycles']
                
                # Extract current cycle features (non-sequence)
                current_features = battery_df.iloc[i][
                    [col for col in df.columns if col not in sequence_cols + ['battery_id', 'cycle_index', 'rul_cycles']]
                ].values
                
                sequences.append(sequence)
                features.append(current_features)
                targets.append(target)
        
        return np.array(sequences), np.array(features), np.array(targets)
    
    def train_lstm_encoder(self, sequences: np.ndarray, features: np.ndarray, 
                          targets: np.ndarray, epochs: int = 100, batch_size: int = 32,
                          learning_rate: float = 0.001) -> Dict:
        """Train LSTM encoder."""
        logger.info("Training LSTM encoder")
        
        # Initialize LSTM
        input_size = sequences.shape[2]
        self.lstm_encoder = LSTMBatteryEncoder(
            input_size=input_size,
            hidden_size=self.lstm_hidden_size,
            num_layers=self.lstm_layers,
            output_size=self.lstm_output_size
        )
        
        # Create dataset and dataloader
        dataset = BatterySequenceDataset(sequences, features, targets)
        dataloader = DataLoader(dataset, batch_size=batch_size, shuffle=True)
        
        # Training setup
        optimizer = optim.Adam(self.lstm_encoder.parameters(), lr=learning_rate)
        criterion = nn.MSELoss()
        
        # Training loop
        self.lstm_encoder.train()
        losses = []
        
        for epoch in range(epochs):
            epoch_loss = 0
            for batch_sequences, batch_features, batch_targets in dataloader:
                optimizer.zero_grad()
                
                # Get LSTM embeddings
                lstm_embeddings = self.lstm_encoder(batch_sequences)
                
                # Simple linear regression for initial training
                predictions = torch.sum(lstm_embeddings, dim=1)
                
                loss = criterion(predictions, batch_targets)
                loss.backward()
                optimizer.step()
                
                epoch_loss += loss.item()
            
            avg_loss = epoch_loss / len(dataloader)
            losses.append(avg_loss)
            
            if epoch % 20 == 0:
                logger.info(f"Epoch {epoch}, Loss: {avg_loss:.4f}")
        
        return {'losses': losses}
    
    def extract_lstm_features(self, sequences: np.ndarray) -> np.ndarray:
        """Extract LSTM embeddings for GBM training."""
        logger.info("Extracting LSTM features")
        
        self.lstm_encoder.eval()
        lstm_features = []
        
        with torch.no_grad():
            for i in range(0, len(sequences), 32):  # Process in batches
                batch_sequences = torch.FloatTensor(sequences[i:i+32])
                batch_embeddings = self.lstm_encoder(batch_sequences)
                lstm_features.append(batch_embeddings.numpy())
        
        return np.vstack(lstm_features)
    
    def train_gbm(self, lstm_features: np.ndarray, static_features: np.ndarray, 
                  targets: np.ndarray) -> Dict:
        """Train GBM model on LSTM embeddings + static features."""
        logger.info("Training GBM model")
        
        # Combine LSTM embeddings with static features
        combined_features = np.hstack([lstm_features, static_features])
        
        # Train XGBoost with improved parameters
        self.gbm_model = xgb.XGBRegressor(
            n_estimators=300,
            max_depth=8,
            learning_rate=0.05,
            subsample=0.9,
            colsample_bytree=0.9,
            reg_alpha=0.1,
            reg_lambda=0.1,
            random_state=42
        )
        
        self.gbm_model.fit(combined_features, targets)
        
        # Get feature importance
        feature_importance = self.gbm_model.feature_importances_
        
        return {
            'feature_importance': feature_importance,
            'n_features': combined_features.shape[1]
        }
    
    def predict(self, sequences: np.ndarray, static_features: np.ndarray) -> np.ndarray:
        """Make predictions using the hybrid model."""
        # Extract LSTM features
        lstm_features = self.extract_lstm_features(sequences)
        
        # Combine with static features
        combined_features = np.hstack([lstm_features, static_features])
        
        # Predict with GBM
        predictions = self.gbm_model.predict(combined_features)
        
        return predictions
    
    def save_model(self, model_dir: str):
        """Save the complete hybrid model."""
        os.makedirs(model_dir, exist_ok=True)
        
        # Save LSTM encoder
        torch.save(self.lstm_encoder.state_dict(), 
                  os.path.join(model_dir, 'lstm_encoder.pth'))
        
        # Save GBM model
        joblib.dump(self.gbm_model, 
                   os.path.join(model_dir, 'gbm_model.joblib'))
        
        # Save model metadata
        metadata = {
            'sequence_length': self.sequence_length,
            'lstm_hidden_size': self.lstm_hidden_size,
            'lstm_layers': self.lstm_layers,
            'lstm_output_size': self.lstm_output_size
        }
        
        with open(os.path.join(model_dir, 'model_metadata.json'), 'w') as f:
            json.dump(metadata, f, indent=2)
        
        logger.info(f"Model saved to {model_dir}")
    
    def load_model(self, model_dir: str):
        """Load the complete hybrid model."""
        # Load metadata
        with open(os.path.join(model_dir, 'model_metadata.json'), 'r') as f:
            metadata = json.load(f)
        
        # Initialize LSTM
        self.lstm_encoder = LSTMBatteryEncoder(
            input_size=metadata['sequence_length'],
            hidden_size=metadata['lstm_hidden_size'],
            num_layers=metadata['lstm_layers'],
            output_size=metadata['lstm_output_size']
        )
        
        # Load weights
        self.lstm_encoder.load_state_dict(
            torch.load(os.path.join(model_dir, 'lstm_encoder.pth'))
        )
        
        # Load GBM
        self.gbm_model = joblib.load(
            os.path.join(model_dir, 'gbm_model.joblib')
        )
        
        logger.info(f"Model loaded from {model_dir}")


def compute_metrics(y_true: np.ndarray, y_pred: np.ndarray) -> Dict:
    """Compute evaluation metrics."""
    mae = mean_absolute_error(y_true, y_pred)
    rmse = np.sqrt(mean_squared_error(y_true, y_pred))
    
    # sMAPE (symmetric Mean Absolute Percentage Error)
    smape = np.mean(2 * np.abs(y_true - y_pred) / (np.abs(y_true) + np.abs(y_pred) + 1e-8)) * 100
    
    return {
        'mae': mae,
        'rmse': rmse,
        'smape': smape
    }


if __name__ == "__main__":
    # This would be called from the training pipeline
    logger.info("Model architecture defined. Use train_model.py for training.")

