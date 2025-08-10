# Robotics Python Package

A Python package for robotics research, scientific computing, and analysis with a focus on reproducibility and code quality.

## Features

- **Physical Constants**: Comprehensive collection of physical constants with proper citations and units
- **Logging Utilities**: Configurable logging with reproducibility features
- **Quality Assurance**: Built-in quality checks and testing frameworks
- **Type Safety**: Full type hints with mypy strict mode support

## Installation

### Using pip
```bash
pip install -r requirements.txt
```

### Using conda
```bash
conda env create -f environment.yml
conda activate robotics
```

## Quick Start

```python
from src.constants import GRAVITY_M_S2, GOLF_BALL_MASS_KG
from src.logger_utils import setup_logging, set_seeds

# Set up logging
setup_logging("INFO")

# Set seeds for reproducibility
set_seeds(42)

# Use constants
print(f"Gravity: {GRAVITY_M_S2} m/s²")
print(f"Golf ball mass: {GOLF_BALL_MASS_KG} kg")
```

## Package Structure

```
python/
├── src/
│   ├── __init__.py          # Package initialization
│   ├── constants.py         # Physical constants with citations
│   └── logger_utils.py      # Logging and reproducibility utilities
├── tests/
│   ├── test_constants.py    # Constants validation tests
│   └── test_logger_utils.py # Logger utilities tests
├── requirements.txt          # pip dependencies
└── environment.yml          # conda environment
```

## Constants

All physical constants include:
- **Value**: With appropriate precision
- **Units**: In square brackets
- **Source**: Citation to authoritative reference

### Examples

```python
from src.constants import GRAVITY_M_S2, GOLF_BALL_MASS_KG

# Standard gravity per ISO 80000-3:2006
GRAVITY_M_S2 = 9.80665  # [m/s²]

# USGA Rule 5-1 (1.620 oz max)
GOLF_BALL_MASS_KG = 0.04593  # [kg]
```

## Testing

Run the full test suite:

```bash
# Run all tests
pytest

# Run with coverage
pytest --cov=src --cov-report=term-missing

# Run specific test file
pytest tests/test_constants.py -v
```

## Code Quality

This package follows strict quality standards:

- **No placeholders**: No TODO, FIXME, or pass statements
- **Type safety**: Full type hints with mypy strict mode
- **Documentation**: Comprehensive docstrings for all functions
- **Testing**: 100% test coverage with negative test cases

### Quality Checks

```bash
# Linting
ruff check .

# Type checking
mypy . --strict

# Quality validation
python quality_check_script.py
```

## Contributing

1. Create a feature branch: `git checkout -b feat/your-feature`
2. Make your changes following the quality standards
3. Add tests for new functionality
4. Run all quality checks
5. Submit a pull request

## License

This project is licensed under the MIT License - see the [LICENSE](../LICENSE) file for details.
