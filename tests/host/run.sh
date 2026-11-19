#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
BUILD="${TMPDIR:-/tmp}/gimbal-host-tests"
mkdir -p "$BUILD"
INC=(
  -I"$ROOT/src/app"
  -I"$ROOT/src/device"
  -I"$ROOT/src/domain"
  -I"$ROOT/src/infra"
  -I"$ROOT/src/service"
  -I"$ROOT/src/service/assemble"
  -I"$ROOT/src/device/bus_motor"
  -I"$ROOT/src/device/bus_motor/dm_motor"
)
CFLAGS=(-std=c11 -Wall -Wextra -Werror -O2)

gcc "${CFLAGS[@]}" "${INC[@]}" \
  "$ROOT/tests/host/test_dm_mit_frame.c" \
  "$ROOT/src/device/bus_motor/dm_motor/dm_motor_core.c" \
  "$ROOT/src/device/bus_motor/dm_motor/dm_motor_protocol_v4.c" \
  -lm -o "$BUILD/test_dm_mit_frame"
"$BUILD/test_dm_mit_frame"

gcc "${CFLAGS[@]}" "${INC[@]}" \
  "$ROOT/tests/host/test_dm_mit_dispatch.c" \
  "$ROOT/src/device/bus_motor/bus_motor.c" \
  "$ROOT/src/device/bus_motor/dm_motor.c" \
  "$ROOT/src/device/bus_motor/dm_motor/dm_motor_core.c" \
  "$ROOT/src/device/bus_motor/dm_motor/dm_motor_protocol_v3.c" \
  "$ROOT/src/device/bus_motor/dm_motor/dm_motor_protocol_v4.c" \
  -lm -o "$BUILD/test_dm_mit_dispatch"
"$BUILD/test_dm_mit_dispatch"

gcc "${CFLAGS[@]}" "${INC[@]}" \
  "$ROOT/tests/host/test_rearm_indicator.c" \
  "$ROOT/src/app/app_runtime.c" \
  -o "$BUILD/test_rearm_indicator"
"$BUILD/test_rearm_indicator"

echo "All host tests passed"
