"""ROS distro compatibility helpers for launch-time tweaks."""

from __future__ import annotations

import os

LEGACY_MOVEIT_ADAPTER_DISTROS = {
    # Distros that still expect request/response adapters as a single string
    'foxy',
    'galactic',
    'humble',
    'iron',
}


def use_legacy_moveit_adapter_format() -> bool:
    """Return True if MoveIt expects adapter plugins encoded as a single string."""
    ros_distro = os.getenv('ROS_DISTRO', '').strip().lower()
    if not ros_distro:
        # Default to legacy behaviour if the distro is unknown
        return True
    return ros_distro in LEGACY_MOVEIT_ADAPTER_DISTROS

