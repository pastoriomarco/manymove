#!/usr/bin/env bash

# Abort if shell is not interactive
if [[ "$-" != *i* ]]; then
  return 0 2>/dev/null || exit 0
fi

WORKSPACE_ROOT="${MANYMOVE_WS:-/opt/manymove_ws}"
OVERLAY="${WORKSPACE_ROOT}/install/setup.bash"

# Avoid re-sourcing within the same shell session
if [[ -n "${MANYMOVE_OVERLAY_SOURCED:-}" ]]; then
  return 0 2>/dev/null || exit 0
fi

if [[ -f "${OVERLAY}" ]]; then
  # shellcheck disable=SC1090
  source "${OVERLAY}"
  export MANYMOVE_OVERLAY_SOURCED=1
fi

return 0 2>/dev/null || exit 0
