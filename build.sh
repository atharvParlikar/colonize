#!/usr/bin/env bash
set -euo pipefail

preset="${1:-debug}"

case "${preset}" in
    debug|release)
        cmake --preset "${preset}"
        cmake --build --preset "${preset}"
        ;;
    run)
        cmake --preset debug
        cmake --build --preset debug
        ./build/debug/fps-shooter
        ;;
    *)
        echo "Usage: $0 [debug|release|run]"
        exit 1
        ;;
esac
