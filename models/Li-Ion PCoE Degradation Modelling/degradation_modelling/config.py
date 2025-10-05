"""
Configuration file for battery RUL prediction pipeline.
"""

# Data paths
DATA_DIR = "NASA PCoE Li-Ion Battery Data"
OUTPUT_DIR = "data/processed"
MODEL_DIR = "artifacts"

# Feature engineering parameters
WINDOW_SIZE = 15
SEQUENCE_LENGTH = 10
EMA_ALPHA = 0.3

# Model parameters
LSTM_HIDDEN_SIZE = 64
LSTM_LAYERS = 2
LSTM_OUTPUT_SIZE = 32
LSTM_DROPOUT = 0.2

# Training parameters
BATCH_SIZE = 32
LEARNING_RATE = 0.001
LSTM_EPOCHS = 50
GBM_ESTIMATORS = 200
GBM_MAX_DEPTH = 6
GBM_LEARNING_RATE = 0.1

# Validation parameters
TEST_SIZE = 0.2
N_FOLDS = 5
RANDOM_STATE = 42

# SOH/RUL parameters
EOL_THRESHOLD = 0.8
INITIAL_CYCLES = 5

# Feature columns for sequences
SEQUENCE_COLUMNS = ['capacity', 'voltage_mean', 'temperature_mean', 'soh']

# Logging
LOG_LEVEL = "INFO"
