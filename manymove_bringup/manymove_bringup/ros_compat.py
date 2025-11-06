"""ROS distro compatibility helpers for launch-time tweaks."""

from __future__ import annotations

import os
from typing import Iterable

from ament_index_python.packages import get_package_share_directory

LEGACY_MOVEIT_ADAPTER_DISTROS = {
    # Distros that still expect request/response adapters as a single string
    'foxy',
    'galactic',
    'humble',
    'iron',
}

_LEGACY_OVERRIDE_ENV = 'MANYMOVE_FORCE_LEGACY_MOVEIT_ADAPTER_FORMAT'


def use_legacy_moveit_adapter_format() -> bool:
    """Return True if MoveIt expects adapter plugins encoded as a single string."""
    override = os.getenv(_LEGACY_OVERRIDE_ENV, '').strip().lower()
    if override in ('1', 'true', 'yes', 'on'):
        return True
    if override in ('0', 'false', 'no', 'off'):
        return False

    ros_distro = os.getenv('ROS_DISTRO', '').strip().lower()
    if not ros_distro:
        # Default to the modern behaviour if the distro is unknown; this matches ROS Jazzy+.
        return False
    return ros_distro in LEGACY_MOVEIT_ADAPTER_DISTROS


def resolve_package_file(package_name: str, candidates: Iterable[str]) -> str:
    """Return the first relative path under a package share directory that exists."""
    share_dir = get_package_share_directory(package_name)
    for relative in candidates:
        candidate_path = os.path.join(share_dir, relative)
        if os.path.exists(candidate_path):
            return relative
    candidate_list = ', '.join(candidates)
    raise FileNotFoundError(
        f"None of the candidate files [{candidate_list}] exist in package '{package_name}'."
    )


def resolve_moveit_controller_config(package_name: str) -> str:
    """Pick the correct MoveIt controller config file for the given package."""
    return resolve_package_file(
        package_name,
        (
            os.path.join('config', 'controllers.yaml'),
            os.path.join('config', 'ros2_controllers.yaml'),
            os.path.join('config', 'moveit_controllers.yaml'),
        ),
    )
