import logging
import random

logger = logging.getLogger(__name__)


def set_seeds(seed: int) -> None:
    """Set random seeds for reproducibility.

    Args:
        seed: Random seed value for Python random
    """
    random.seed(seed)
    logger.info("Seeds set: %d", seed)
