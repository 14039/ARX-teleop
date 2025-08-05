#!/bin/bash
# Setup script to download or build ARX libraries for the current platform

echo "Setting up ARX libraries for $(uname -s)..."

# Detect platform
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    echo "Linux detected"
    # Option: Download pre-built Linux binaries
    # wget https://your-server.com/arx-libs-linux.tar.gz
    # tar -xzf arx-libs-linux.tar.gz -C arx_control/lib/
    
    # Check if libraries are already present in this repo
    if [ -d "arx_control/lib/arx_r5_python" ] && [ -f "arx_control/lib/arx_r5_src/libarx_r5_src.so" ]; then
        echo "✓ ARX libraries already present in repository"
    # Fallback: copy from R5 SDK if available
    elif [ -d "/home/vassar/code/R5/py/ARX_R5_python/bimanual/lib" ]; then
        echo "Copying libraries from R5 SDK..."
        mkdir -p arx_control/lib
        cp -r /home/vassar/code/R5/py/ARX_R5_python/bimanual/lib/* arx_control/lib/
        echo "Libraries copied. Consider committing them to make this repo self-contained."
    else
        echo "ERROR: ARX libraries not found."
        echo "Options:"
        echo "1. Install the ARX R5 SDK and run this script again"
        echo "2. Copy libraries manually to arx_control/lib/"
        echo "3. Download pre-built libraries (if available)"
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