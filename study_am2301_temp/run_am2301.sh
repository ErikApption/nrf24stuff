#!/bin/bash
# Script to run rpi_am2301.py using Poetry

# Resolve the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cd "$SCRIPT_DIR"

# Execute the Python script with any passed arguments
poetry run python rpi_am2301.py "$@"
