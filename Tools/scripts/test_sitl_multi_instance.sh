#!/usr/bin/env bash
# Launch 5 SITL instances and confirm they all start and stay running.
# AP_FLAKE8_CLEAN (N/A - shell script)
set -e
BINARY="${1:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)/build/sitl/bin/arducopter}"
trap 'kill $(jobs -p) 2>/dev/null' EXIT
SITL_INSTANCE_COUNT=5
export SITL_INSTANCE_COUNT
for i in 0 1 2 3 4; do
    "${BINARY}" --instance "${i}" --speedup 100 --wipe --home "-35.363261,149.165230,584,353" --model "+" --uartA "tcp:0" 2>&1 | grep --line-buffered SharedMem &
done
sleep 10
kill -0 $(jobs -p) && echo "PASS: instances running"
