#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/../.." && pwd)"
doxyfile="${repo_root}/docs/Doxyfile"

if ! command -v doxygen >/dev/null 2>&1; then
    echo "error: doxygen is not installed." >&2
    echo "install with: brew install doxygen" >&2
    exit 127
fi

cd "${repo_root}"
mkdir -p docs/api
doxygen "${doxyfile}"

echo
echo "generated: ${repo_root}/docs/api/html/index.html"
