#!/bin/bash
# Script to activate virtual environment and run rpi_am2301.py from any path

# Resolve the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="$SCRIPT_DIR/.venv"
PYTHON_SCRIPT="$SCRIPT_DIR/rpi_am2301.py"

# Check if virtual environment exists
if [ ! -d "$VENV_DIR" ]; then
    echo "Virtual environment not found at $VENV_DIR"
    exit 1
fi

# Activate virtual environment
source "$VENV_DIR/bin/activate"

# Execute the Python script with any passed arguments
python "$PYTHON_SCRIPT" "$@"
