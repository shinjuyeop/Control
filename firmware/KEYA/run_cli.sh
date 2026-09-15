#!/usr/bin/env bash
set -euo pipefail

tool_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)

if command -v uv >/dev/null 2>&1; then
    exec uv run --isolated --with-requirements "$tool_dir/requirements-cli.txt" \
        python "$tool_dir/keya_v43_config.py" "$@"
fi

exec python3 "$tool_dir/keya_v43_config.py" "$@"
