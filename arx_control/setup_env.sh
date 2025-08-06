#!/bin/bash
# ARX Control Environment Setup Script
# Source this script before running ARX control programs

# Get the directory of this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# Set library paths for ARX libraries
export LD_LIBRARY_PATH="$PROJECT_ROOT/arx_control/lib/arx_r5_src:$LD_LIBRARY_PATH"
export LD_LIBRARY_PATH="$PROJECT_ROOT/arx_control/lib:$LD_LIBRARY_PATH"
export LD_LIBRARY_PATH="/usr/local/lib:$LD_LIBRARY_PATH"

# Add Python path for the arx_r5_python module
if [ -d "$PROJECT_ROOT/arx_control/lib/arx_r5_python" ]; then
    export PYTHONPATH="$PROJECT_ROOT/arx_control/lib/arx_r5_python:$PYTHONPATH"
fi

# Add the arx_control module to Python path
export PYTHONPATH="$PROJECT_ROOT:$PYTHONPATH"

echo "ARX Control environment configured."
echo "Library paths set for: $PROJECT_ROOT/arx_control/lib/"
echo "Python path includes: $PROJECT_ROOT"