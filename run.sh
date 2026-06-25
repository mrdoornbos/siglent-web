#!/usr/bin/env bash
#
# Launch the Siglent web mirror/control server.
#
# USBTMC allows only one connection at a time, so this stops any server that is
# already running and waits for it to release the USB before starting a new one.
# Otherwise the new instance fails to claim the device ("Access denied").
#
# Any arguments are passed through to `scope serve`, for example:
#   ./run.sh --port 9000 --host 0.0.0.0
#
set -euo pipefail
cd "$(dirname "$0")"

if pgrep -f "scope serve" >/dev/null 2>&1; then
  echo "Stopping existing server..."
  pkill -TERM -f "scope serve" || true
  for _ in $(seq 1 15); do
    pgrep -f "scope serve" >/dev/null 2>&1 || break
    sleep 1
  done
fi

echo "Starting Siglent scope server..."
exec uv run scope serve "$@"
