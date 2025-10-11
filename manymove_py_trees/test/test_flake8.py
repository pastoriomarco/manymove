# Copyright 2017 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Check the package's compliance with the flake8 ruleset."""

import pytest
from pathlib import Path
from ament_flake8.main import main_with_errors


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    """Execute flake8 against the repository-level configuration."""
    # Point to repo-level setup.cfg to keep one source of truth
    config = Path(__file__).resolve().parents[2] / "setup.cfg"
    rc, errors = main_with_errors(argv=["--config", str(config)])
    assert rc == 0, "Found %d code style errors / warnings:\n" % len(
        errors
    ) + "\n".join(errors)
