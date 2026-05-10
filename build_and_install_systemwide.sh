#!/bin/bash

set -e
INSTALL_FOLDER=/opt/xbot/lib

while [[ $# -gt 0 ]]; do
    case "$1" in
        --install-dir)
            INSTALL_FOLDER="$2"
            shift 2
            ;;
        *)
            echo "Unknown option: $1"
            echo "Usage: $0 [--install-dir <path>]"
            exit 1
            ;;
    esac
done

echo "Installing to: ${INSTALL_FOLDER}"

cd "$(dirname "$0")"
mkdir -p build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=${INSTALL_FOLDER}
cmake --build .
cmake --install .