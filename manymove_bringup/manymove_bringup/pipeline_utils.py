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

from .ros_compat import use_legacy_moveit_adapter_format


def _fix_request_adapter_prefix(value: str) -> str:
    """Use the ROS 2 MoveIt adapter namespace (default_planner_request_adapters)."""
    return value.replace(
        'default_planning_request_adapters/', 'default_planner_request_adapters/'
    )


def _string_to_list(value: str) -> list[str]:
    """Split whitespace/newline separated adapter strings into a list."""
    tokens: list[str] = []
    for chunk in value.replace('\n', ' ').split(' '):
        entry = chunk.strip()
        if entry:
            tokens.append(entry)
    return tokens


def normalize_pipeline_config(data: Any) -> Any:
    """Normalize MoveIt adapter fields to match the expected format for the current distro."""
    if isinstance(data, dict):
        for key in list(data.keys()):
            value = data[key]

            if key == 'planning_plugins' and isinstance(value, list):
                if value and 'planning_plugin' not in data:
                    data['planning_plugin'] = value[0]
                data.pop('planning_plugins', None)
                continue

            if key in ('request_adapters', 'response_adapters'):
                legacy_format = use_legacy_moveit_adapter_format()
                new_value: Any = value

                if isinstance(value, list):
                    processed_list = [
                        _fix_request_adapter_prefix(item) if key == 'request_adapters' else item
                        for item in value
                    ]
                    if legacy_format:
                        new_value = '\n'.join(processed_list)
                    else:
                        new_value = processed_list
                elif isinstance(value, str):
                    if legacy_format:
                        new_value = (
                            _fix_request_adapter_prefix(value)
                            if key == 'request_adapters'
                            else value
                        )
                    else:
                        processed_list = _string_to_list(value)
                        if key == 'request_adapters':
                            processed_list = [
                                _fix_request_adapter_prefix(item) for item in processed_list
                            ]
                        new_value = processed_list

                data[key] = new_value
            else:
                normalize_pipeline_config(value)
    elif isinstance(data, list):
        for item in data:
            normalize_pipeline_config(item)
    return data
