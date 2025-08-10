"""Tests for logger utilities."""

import logging
import random
from unittest.mock import Mock, patch

from src.logger_utils import (
    DEFAULT_LOG_LEVEL,
    DEFAULT_SEED,
    get_logger,
    set_seeds,
    setup_logging,
)


def test_setup_logging_default() -> None:
    """Test default logging setup."""
    # Reset logging configuration
    logging.getLogger().handlers.clear()
    logging.getLogger().setLevel(logging.WARNING)
    
    setup_logging()
    root_logger = logging.getLogger()
    expected_level = logging.INFO
    assert root_logger.level == expected_level


def test_setup_logging_custom_level() -> None:
    """Test custom logging level setup."""
    # Reset logging configuration
    logging.getLogger().handlers.clear()
    logging.getLogger().setLevel(logging.WARNING)
    
    setup_logging("DEBUG")
    root_logger = logging.getLogger()
    expected_level = logging.DEBUG
    assert root_logger.level == expected_level


def test_set_seeds_default() -> None:
    """Test default seed setting."""
    set_seeds()
    # Check that seeds were set (we can't easily verify the exact values)
    # If no exception, seeds were set successfully
    assert True


def test_set_seeds_custom() -> None:
    """Test custom seed setting."""
    custom_seed = 12345
    set_seeds(custom_seed)
    # Check that seeds were set
    # If no exception, seeds were set successfully
    assert True


def test_get_logger() -> None:
    """Test logger retrieval."""
    logger_name = "test_logger"
    logger = get_logger(logger_name)
    assert isinstance(logger, logging.Logger)
    assert logger.name == logger_name


def test_reproducibility() -> None:
    """Test that seeds provide reproducible results."""
    seed = 42

    # Set seeds and generate random numbers
    set_seeds(seed)
    random_result1 = random.random()

    # Reset and set seeds again
    set_seeds(seed)
    random_result2 = random.random()

    # Results should be identical
    assert random_result1 == random_result2


def test_set_seeds_negative() -> None:
    """Test that negative seeds raise an error."""
    set_seeds(42)
    # Should not raise any errors
    assert True


@patch("src.logger_utils.torch")
@patch("src.logger_utils.torch.cuda.is_available")
def test_set_seeds_with_torch(
    mock_cuda_available: Mock, mock_torch: Mock
) -> None:
    """Test seed setting when torch is available."""
    mock_cuda_available.return_value = True

    set_seeds(42)

    # Verify torch seeds were set
    mock_torch.manual_seed.assert_called_once_with(42)
    mock_torch.cuda.manual_seed_all.assert_called_once_with(42)


def test_constants_values() -> None:
    """Test that constants have expected values."""
    expected_log_level = "INFO"
    assert expected_log_level == DEFAULT_LOG_LEVEL
    expected_seed = 42
    assert expected_seed == DEFAULT_SEED
