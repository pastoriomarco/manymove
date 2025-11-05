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


_LEGACY_ADAPTER_PREFIX = 'default_planner_request_adapters/'
_MODERN_ADAPTER_PREFIX = 'default_planning_request_adapters/'


def _normalize_request_adapter_prefix(value: str, legacy: bool) -> str:
    """Ensure adapter prefixes match the expected naming convention for the distro."""
    if legacy:
        return value.replace(_MODERN_ADAPTER_PREFIX, _LEGACY_ADAPTER_PREFIX)
    return value.replace(_LEGACY_ADAPTER_PREFIX, _MODERN_ADAPTER_PREFIX)


def _maybe_fix_request_adapter(value: str, legacy: bool) -> str:
    """Apply adapter prefix adjustments when required for the current distro."""
    return _normalize_request_adapter_prefix(value, legacy)


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
    legacy_format = use_legacy_moveit_adapter_format()

    if isinstance(data, dict):
        for key in list(data.keys()):
            value = data[key]

            if key == 'planning_plugins' and isinstance(value, list):
                if legacy_format:
                    if value and 'planning_plugin' not in data:
                        data['planning_plugin'] = value[0]
                    data.pop('planning_plugins', None)
                else:
                    # Ensure nested structures keep list semantics
                    data[key] = list(value)
                continue

            if key == 'planning_plugins' and isinstance(value, str):
                if legacy_format:
                    if 'planning_plugin' not in data:
                        data['planning_plugin'] = value
                    data.pop('planning_plugins', None)
                else:
                    data[key] = _string_to_list(value)
                continue

            if key == 'planning_plugin' and not legacy_format:
                if 'planning_plugins' not in data and isinstance(value, str):
                    data['planning_plugins'] = [value]

            if key in ('request_adapters', 'response_adapters'):
                new_value: Any = value

                if isinstance(value, list):
                    processed_list = [
                        _maybe_fix_request_adapter(item, legacy_format)
                        if key == 'request_adapters'
                        else item
                        for item in value
                    ]
                    if legacy_format:
                        new_value = '\n'.join(processed_list)
                    else:
                        new_value = processed_list
                elif isinstance(value, str):
                    if legacy_format:
                        new_value = (
                            _maybe_fix_request_adapter(value, legacy_format)
                            if key == 'request_adapters'
                            else value
                        )
                    else:
                        processed_list = _string_to_list(value)
                        if key == 'request_adapters':
                            processed_list = [
                                _maybe_fix_request_adapter(item, legacy_format)
                                for item in processed_list
                            ]
                        new_value = processed_list

                data[key] = new_value
            else:
                normalize_pipeline_config(value)
    elif isinstance(data, list):
        for item in data:
            normalize_pipeline_config(item)
    return data
