"""Ensure Python sources stay Black-formatted."""

import pytest

try:
    from ament_black.main import main
except ImportError:
    main = None


@pytest.mark.linter
@pytest.mark.black
def test_black():
    """Check that Black would make no changes."""
    if main is None:
        pytest.skip('ament_black not available')
    rc = main(argv=['.'])
    assert rc == 0, 'Black formatting issues detected'
