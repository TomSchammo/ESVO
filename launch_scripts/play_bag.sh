#!/bin/bash
#
# Bag Player for ESVO
# Usage: ./play_bag.sh <bag_path> [rate]
#
# Example:
#   ./play_bag.sh ~/vm/bags/rpg_bin_edited
#   ./play_bag.sh ~/vm/bags/rpg_bin_edited 0.3
#

BAG_PATH="${1:-}"
RATE="${2:-0.5}"

if [[ -z "$BAG_PATH" ]]; then
    echo "Usage: $0 <bag_path> [rate]"
    echo ""
    echo "Available bags:"
    ls -d ~/vm/bags/*/ 2>/dev/null || echo "  No bags found in ~/vm/bags/"
    exit 1
fi

if [[ ! -d "$BAG_PATH" ]]; then
    echo "Error: Bag not found: $BAG_PATH"
    exit 1
fi

source /opt/ros/jazzy/setup.bash

echo "Playing: $BAG_PATH"
echo "Rate: ${RATE}x"
echo "Press Space to pause, Ctrl+C to stop"
echo ""

ros2 bag play "$BAG_PATH" --clock -r "$RATE"
