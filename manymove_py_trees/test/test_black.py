"""Ensure Python sources stay Black-formatted."""

import pytest

# Skip the test if ament_black is not available in the environment
pytest.importorskip("ament_black")
from ament_black.main import main


@pytest.mark.linter
@pytest.mark.black
def test_black():
    """Check that Black would make no changes."""
    rc = main(argv=["--check", "--diff", "."])
    assert rc == 0, "Black formatting issues detected"
