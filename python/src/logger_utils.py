"""Logging utilities with reproducibility support."""

import logging
import random

# Set default logging level
DEFAULT_LOG_LEVEL: str = "INFO"
DEFAULT_SEED: int = 42

# Global logger instance
logger = logging.getLogger(__name__)

# Try to import optional dependencies
try:
    import numpy as np
    NUMPY_AVAILABLE = True
except ImportError:
    NUMPY_AVAILABLE = False

try:
    import torch
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False


def setup_logging(level: str = DEFAULT_LOG_LEVEL) -> None:
    """Set up logging configuration.

    Args:
        level: Logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL)
    """
    logging.basicConfig(
        level=getattr(logging, level.upper()),
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )


def set_seeds(seed: int = DEFAULT_SEED) -> None:
    """Set random seeds for reproducibility.

    Args:
        seed: Random seed value for Python random and numpy
    """
    if seed < 0:
        error_msg = f"Seed must be non-negative, got {seed}"
        raise ValueError(error_msg)

    random.seed(seed)

    # Set numpy seed if available
    if NUMPY_AVAILABLE:
        # Use the modern numpy random generator
        rng = np.random.default_rng(seed)
        # Note: In production, you might want to store the generator
        # in a module-level variable or pass it to functions that need it
        logger.debug("NumPy random generator created with seed: %d", seed)
    else:
        logger.warning("NumPy not available, skipping numpy seed setting")

    # Set torch seed if available
    if TORCH_AVAILABLE:
        torch.manual_seed(seed)
        if torch.cuda.is_available():
            torch.cuda.manual_seed_all(seed)

    logger.info("Seeds set: %d", seed)


def get_logger(name: str) -> logging.Logger:
    """Get a logger instance with the specified name.

    Args:
        name: Logger name (usually __name__)

    Returns:
        Configured logger instance
    """
    return logging.getLogger(name)
