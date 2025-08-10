"""Example test file."""

EXPECTED_SUM = 4


def test_sanity() -> None:
    """Test basic sanity check that 2+2=4."""
    result = 2 + 2
    return result == EXPECTED_SUM
