# Robotics Research Repository

A comprehensive repository for robotics research, featuring MATLAB implementations for RRT path planning and a Python package for scientific computing with strict quality standards.

## Repository Structure

```
Robotics/
├── python/                    # Python package with quality standards
│   ├── src/                  # Source code
│   ├── tests/                # Comprehensive test suite
│   └── README.md             # Python package documentation
├── matlab/                   # MATLAB implementations
│   ├── Matlab RRT Path Planner/  # RRT algorithms and examples
│   └── Modern Robotics Homework/ # Course materials and projects
├── scripts/                  # Development and safety scripts
├── data/                     # Data storage (empty, ready for use)
├── output/                   # Output storage (empty, ready for use)
└── .cursorrules.md           # Strict AI coding guidelines
```

## Quick Start

### Python Package
```bash
cd python
pip install -r requirements.txt
pytest  # Run tests
```

### MATLAB
```bash
cd matlab
# Open MATLAB and run run_all.m for RRT examples
```

### Development Setup
```bash
# Install pre-commit hooks
bash scripts/setup_precommit.sh

# Create development branch
git checkout -b feat/your-feature
```

## Quality Standards

This repository follows strict quality standards defined in `.cursorrules.md`:

- **No placeholders**: No TODO, FIXME, or pass statements
- **Type safety**: Full type hints with mypy strict mode
- **Documentation**: Comprehensive docstrings and citations
- **Testing**: 100% test coverage with negative test cases
- **Constants**: All values properly cited with units and sources

## Daily Safety

- Commit every ~30 minutes (`wip:` if tests fail)
- End-of-day snapshot: `bash scripts/snapshot.sh`
- Big AI refactor? Create `backup/before-ai-<desc>` branch first

## Reproducibility

- MATLAB: `matlab/run_all.m` should regenerate results
- Python: Environment pinned via `python/requirements.txt` or `python/environment.yml`
- All constants properly cited with authoritative sources

## Contributing

1. Follow the `.cursorrules.md` guidelines strictly
2. Create feature branches: `git checkout -b feat/description`
3. Ensure all quality checks pass before submitting
4. Include comprehensive tests for new functionality

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
