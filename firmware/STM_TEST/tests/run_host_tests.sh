#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
test_binary="${TMPDIR:-/tmp}/stm_test_hil_logic_test"

cc -std=gnu11 -Wall -Wextra -Werror \
  -I"${project_dir}/Core/Inc" \
  "${project_dir}/Core/Src/hil_logic.c" \
  "${project_dir}/tests/test_hil_logic.c" \
  -lm -o "${test_binary}"
"${test_binary}"
