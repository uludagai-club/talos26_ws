#!/bin/bash
# Saves the map using both slam_toolbox serialization and map_server format

MAP_NAME="${1:-my_map}"
SAVE_DIR="${2:-.}"

echo "========================================"
echo "2D SLAM Map Saving Script"
echo "========================================"
echo "Map name: $MAP_NAME"
echo "Save directory: $SAVE_DIR"
echo ""

# Create save directory if it doesn't exist
mkdir -p "$SAVE_DIR"

# Method 1: Save using map_server (standard format - .pgm + .yaml)
echo "[1/2] Saving map using map_server (standard format)..."
rosrun map_server map_saver -f "${SAVE_DIR}/${MAP_NAME}" map:=/map
if [ $? -eq 0 ]; then
    echo "  ✓ Saved: ${SAVE_DIR}/${MAP_NAME}.pgm and ${SAVE_DIR}/${MAP_NAME}.yaml"
else
    echo "  ✗ Failed to save with map_server. Is /map topic publishing?"
    echo "    Check with: rostopic echo /map -n1"
fi

# Method 2: Save using slam_toolbox serialization (for resuming mapping later)
echo ""
echo "[2/2] Saving map using slam_toolbox serialization..."
rosservice call /slam_toolbox/serialize_map "filename: '${SAVE_DIR}/${MAP_NAME}'" 2>/dev/null
if [ $? -eq 0 ]; then
    echo "  ✓ Serialized: ${SAVE_DIR}/${MAP_NAME}.posegraph and ${SAVE_DIR}/${MAP_NAME}.data"
else
    echo "  ✗ slam_toolbox serialization failed or not available"
    echo "    This is optional - map_server format should work for navigation"
fi

echo ""
echo "========================================"
echo "Map saving complete!"
echo "========================================"
echo ""
echo "Files saved:"
ls -la "${SAVE_DIR}/${MAP_NAME}"* 2>/dev/null || echo "No files found"
