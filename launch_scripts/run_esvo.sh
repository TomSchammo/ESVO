#!/bin/bash
#
# ESVO System Launcher
# Usage: ./run_esvo.sh [dataset]
#   dataset: rpg (default), upenn, hkust, dsec
#
# Example:
#   ./run_esvo.sh rpg
#   ./run_esvo.sh upenn
#

set -e

# Default dataset
DATASET="${1:-rpg}"

# Paths
ROS2_WS="$HOME/ros2_ws"
ESVO_SHARE="$ROS2_WS/install/esvo_core/share/esvo_core"
CALIB_DIR="$ESVO_SHARE/calib/$DATASET"
MAPPING_CFG="$ESVO_SHARE/cfg/mapping/mapping_${DATASET}.yaml"
TRACKING_CFG="$ESVO_SHARE/cfg/tracking/tracking_${DATASET}.yaml"
TS_CFG="$ESVO_SHARE/cfg/time_surface/ts_parameters.yaml"

# Validate dataset
if [[ ! -d "$CALIB_DIR" ]]; then
    echo "Error: Calibration directory not found: $CALIB_DIR"
    echo "Available datasets:"
    ls "$ESVO_SHARE/calib/"
    exit 1
fi

if [[ ! -f "$MAPPING_CFG" ]]; then
    echo "Error: Mapping config not found: $MAPPING_CFG"
    exit 1
fi

if [[ ! -f "$TRACKING_CFG" ]]; then
    echo "Error: Tracking config not found: $TRACKING_CFG"
    exit 1
fi

echo "============================================"
echo "ESVO System - Dataset: $DATASET"
echo "Calibration: $CALIB_DIR"
echo "Mapping cfg: $MAPPING_CFG"
echo "Tracking cfg: $TRACKING_CFG"
echo "============================================"

# Kill any existing processes
echo "Stopping any existing ESVO processes..."
pkill -f esvo_time_surface 2>/dev/null || true
pkill -f camera_info_publisher 2>/dev/null || true
pkill -f sync_timer 2>/dev/null || true
pkill -f esvo_Mapping 2>/dev/null || true
pkill -f esvo_Tracking 2>/dev/null || true
sleep 1

# Source ROS2
source /opt/ros/jazzy/setup.bash
source "$ROS2_WS/install/setup.bash"

echo "Starting ESVO nodes..."

# Camera info publisher
ros2 run esvo_time_surface camera_info_publisher.py --ros-args \
    -p calib_dir:="$CALIB_DIR" \
    -p use_sim_time:=true &
sleep 0.5

# Time surface - Left
ros2 run esvo_time_surface esvo_time_surface --ros-args \
    -r events:=/davis/left/events \
    -r camera_info:=/davis/left/camera_info \
    -r time_surface:=/TS_left \
    -r sync:=/sync \
    -p use_sim_time:=true &
sleep 0.2

# Time surface - Right
ros2 run esvo_time_surface esvo_time_surface --ros-args \
    -r events:=/davis/right/events \
    -r camera_info:=/davis/right/camera_info \
    -r time_surface:=/TS_right \
    -r sync:=/sync \
    -p use_sim_time:=true &
sleep 0.2

# Sync timer
ros2 run esvo_core sync_timer_node.py --ros-args \
    -p rate_hz:=50.0 \
    -p use_sim_time:=true &
sleep 0.2

# Mapping node
ros2 run esvo_core esvo_Mapping --ros-args \
    --params-file "$MAPPING_CFG" \
    -r time_surface_left:=/TS_left \
    -r time_surface_right:=/TS_right \
    -r events_left:=/davis/left/events \
    -r events_right:=/davis/right/events \
    -r stamped_pose:=/esvo_tracking/pose_pub \
    -p calibInfoDir:="$CALIB_DIR" \
    -p use_sim_time:=true &
sleep 0.5

# Tracking node
ros2 run esvo_core esvo_Tracking --ros-args \
    --params-file "$TRACKING_CFG" \
    -r time_surface_left:=/TS_left \
    -r time_surface_right:=/TS_right \
    -r events_left:=/davis/left/events \
    -r stamped_pose:=/esvo_tracking/pose_pub \
    -r pointcloud:=/esvo_mapping/pointcloud_local \
    -r gt_pose:=/optitrack/davis_stereo \
    -p calibInfoDir:="$CALIB_DIR" \
    -p use_sim_time:=true &

echo ""
echo "============================================"
echo "ESVO system started!"
echo ""
echo "Now play a bag in another terminal:"
echo "  ./play_bag.sh ~/vm/bags/rpg_bin_edited"
echo ""
echo "Monitor output:"
echo "  ros2 topic echo /esvo_tracking/pose_pub"
echo "  ros2 topic hz /esvo_mapping/pointcloud_local"
echo ""
echo "Press Ctrl+C to stop all nodes"
echo "============================================"

# Wait for Ctrl+C
trap "echo 'Stopping...'; pkill -f esvo; pkill -f camera_info_publisher; pkill -f sync_timer; exit 0" INT
wait
