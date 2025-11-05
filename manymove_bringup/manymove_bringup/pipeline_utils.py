# Copyright 2025 Flexin Group SRL
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Flexin Group SRL nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
"""Utility helpers for preparing MoveIt pipeline dictionaries."""

from __future__ import annotations

from typing import Any


def _fix_request_adapter_prefix(value: str) -> str:
    """Use the ROS 2 MoveIt adapter namespace (default_planner_request_adapters)."""
    return value.replace(
        'default_planning_request_adapters/', 'default_planner_request_adapters/'
    )


def normalize_pipeline_config(data: Any) -> Any:
    """Convert MoveIt adapter lists into newline-separated strings in-place."""
    if isinstance(data, dict):
        for key in list(data.keys()):
            value = data[key]

            if key == 'planning_plugins' and isinstance(value, list):
                if value and 'planning_plugin' not in data:
                    data['planning_plugin'] = value[0]
                data.pop('planning_plugins', None)
                continue

            if key in ('request_adapters', 'response_adapters'):
                new_value = value
                if isinstance(new_value, list):
                    new_value = '\n'.join(new_value)

                if isinstance(new_value, str) and key == 'request_adapters':
                    new_value = _fix_request_adapter_prefix(new_value)

                data[key] = new_value
            else:
                normalize_pipeline_config(value)
    elif isinstance(data, list):
        for item in data:
            normalize_pipeline_config(item)
    return data
