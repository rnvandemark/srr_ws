#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"

WS_CLEAN=0
WS_J="-j1"
POSITIONAL=()
while [[ $# -gt 0 ]]; do
    key="$1"
    case $key in
        -j*)
            WS_J="$key"; shift ;;
        --clean)
            WS_CLEAN=1; shift ;;
		*) echo "WARNING: Unknown parameter passed: $1";
        POSITIONAL+=("$1"); shift ;;
    esac
done
set -- "${POSITIONAL[@]}"

pushd "$SCRIPT_DIR" >/dev/null 2>&1
[[ WS_CLEAN -eq 1 ]] && rm -rf install build devel
catkin_make install "$WS_J"
popd >/dev/null 2>&1
