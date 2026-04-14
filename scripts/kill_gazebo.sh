#!/bin/bash
# Kill ALL Gazebo / gz sim processes. Run this before launching.
echo "🧹 Killing all Gazebo processes..."
pkill -9 -f "gz sim" 2>/dev/null
pkill -9 -f "ruby.*gz" 2>/dev/null
pkill -9 -f "parameter_bridge" 2>/dev/null
pkill -9 -f "rviz2" 2>/dev/null
pkill -9 -f "slam_toolbox" 2>/dev/null
pkill -9 -f "robot_state_publisher" 2>/dev/null
sleep 2
remaining=$(ps aux | grep -E "gz sim" | grep -v grep | wc -l)
if [ "$remaining" -eq 0 ]; then
    echo "✅ All clean. Safe to launch."
else
    echo "⚠️  $remaining processes still running. Try: sudo kill -9 \$(pgrep -f 'gz sim')"
fi
