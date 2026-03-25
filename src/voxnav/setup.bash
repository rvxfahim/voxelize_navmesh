#!/bin/bash
# Source this file to set up the voxnav workspace

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Source the ROS2 workspace overlay
if [ -f "$SCRIPT_DIR/install/setup.bash" ]; then
    source "$SCRIPT_DIR/install/setup.bash"
else
    echo "Warning: Workspace not built yet. Run 'colcon build' first."
fi