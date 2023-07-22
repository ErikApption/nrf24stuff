#!/usr/bin/env bash
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
set -e
#echo "${SCRIPT_DIR}"
source "${SCRIPT_DIR}/opi_sensors/bin/activate"
python -u ${SCRIPT_DIR}/rpi_bedroom.py