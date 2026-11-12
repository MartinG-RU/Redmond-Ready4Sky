#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "$0")/../.." && pwd)"
dir="$(mktemp -d "${TMPDIR:-/tmp}/r4s-regression.XXXXXX")"
trap 'rm -rf "$dir"' EXIT
flags=(-std=c++20 -g -O1 -fsanitize=address,undefined -fno-omit-frame-pointer -DUSE_ESP32)

# Real driver implementations against lightweight ESPHome/BLE stubs.
# vptr is disabled only here because the stub R4SEngine has no real RTTI.
c++ "${flags[@]}" -fno-sanitize=vptr -I"$root/tests/native/stubs" \
  "$root/components/ready4sky/commands.cpp" \
  "$root/components/skyplug/skyplug.cpp" \
  "$root/components/skycoffee/skycoffee.cpp" \
  "$root/components/skysmoke/skysmoke.cpp" \
  "$root/tests/native/regression.cpp" -o "$dir/drivers"
"$dir/drivers"

# Real BLE engine against a separate, simplified single-threaded IDF simulator.
c++ "${flags[@]}" -I"$root/tests/native/engine_stubs" \
  "$root/components/ready4sky/ready4sky.cpp" \
  "$root/components/ready4sky/commands.cpp" \
  "$root/tests/native/engine_regression.cpp" -o "$dir/engine"
"$dir/engine"
