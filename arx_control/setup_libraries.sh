#!/bin/bash
# Setup script to download or build ARX libraries for the current platform

echo "Setting up ARX libraries for $(uname -s)..."

# Detect platform
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    echo "Linux detected"
    # Option: Download pre-built Linux binaries
    # wget https://your-server.com/arx-libs-linux.tar.gz
    # tar -xzf arx-libs-linux.tar.gz -C arx_control/lib/
    
    # Or copy from R5 SDK if available
    if [ -d "/home/vassar/code/R5/py/ARX_R5_python/bimanual/lib" ]; then
        echo "Copying libraries from R5 SDK..."
        # Create lib directory if it doesn't exist
        mkdir -p arx_control/lib
        cp -r /home/vassar/code/R5/py/ARX_R5_python/bimanual/lib/* arx_control/lib/
        # Copy the Python module
        cp -r /home/vassar/code/R5/py/ARX_R5_python/bimanual/api/arx_r5_python arx_control/lib/
        # Create __init__.py for the Python module if it doesn't exist
        if [ ! -f "arx_control/lib/arx_r5_python/__init__.py" ]; then
            cat > arx_control/lib/arx_r5_python/__init__.py << 'EOF'
# ARX R5 Python Module
# This module provides Python bindings for the ARX R5 robot control library

# Import the compiled extension module
try:
    from .arx_r5_python import *
except ImportError:
    # Try the other Python version
    try:
        from . import arx_r5_python
        # Import all symbols from the module
        for attr in dir(arx_r5_python):
            if not attr.startswith('_'):
                globals()[attr] = getattr(arx_r5_python, attr)
    except ImportError as e:
        raise ImportError(f"Failed to import arx_r5_python extension module: {e}")
EOF
        fi
    else
        echo "ERROR: R5 SDK not found. Please install the ARX R5 SDK first."
        exit 1
    fi
    
elif [[ "$OSTYPE" == "darwin"* ]]; then
    echo "macOS detected"
    echo "ERROR: ARX libraries for macOS are not yet available."
    echo "Please contact ARX support for macOS binaries."
    exit 1
else
    echo "Unsupported platform: $OSTYPE"
    exit 1
fi

echo "Library setup complete!"