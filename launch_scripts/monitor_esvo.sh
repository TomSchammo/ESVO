#!/bin/bash
#
# Monitor ESVO output
# Usage: ./monitor_esvo.sh [mode]
#   mode: pose (default), pointcloud, topics, all
#

MODE="${1:-pose}"

source /opt/ros/jazzy/setup.bash

case "$MODE" in
    pose)
        echo "Monitoring pose estimates (Ctrl+C to stop)..."
        ros2 topic echo /esvo_tracking/pose_pub
        ;;
    pointcloud)
        echo "Monitoring pointcloud rate..."
        ros2 topic hz /esvo_mapping/pointcloud_local
        ;;
    topics)
        echo "Active topics:"
        ros2 topic list
        echo ""
        echo "Topic rates:"
        ros2 topic hz /TS_left /TS_right /esvo_tracking/pose_pub /esvo_mapping/pointcloud_local 2>/dev/null
        ;;
    all)
        echo "=== Active Topics ==="
        ros2 topic list
        echo ""
        echo "=== Topic Info ==="
        for topic in /TS_left /TS_right /esvo_tracking/pose_pub /esvo_mapping/pointcloud_local; do
            echo "--- $topic ---"
            timeout 2 ros2 topic hz "$topic" 2>/dev/null || echo "  (no data)"
        done
        ;;
    *)
        echo "Usage: $0 [pose|pointcloud|topics|all]"
        exit 1
        ;;
esac
