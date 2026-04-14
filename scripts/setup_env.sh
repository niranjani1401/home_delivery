#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
# Auto-detects ROS distro and sets up environment correctly
# Works on both Jazzy (Ubuntu 24.04) and Humble (Ubuntu 22.04)
# ─────────────────────────────────────────────────────────────────────────────

if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
    echo "✅ Sourced ROS 2 Jazzy"
elif [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "✅ Sourced ROS 2 Humble"
else
    echo "❌ No ROS 2 installation found!"
    exit 1
fi

export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/$ROS_DISTRO/share/turtlebot3_gazebo/models

if [ -f ~/tb3_delivery_ws/install/setup.bash ]; then
    source ~/tb3_delivery_ws/install/setup.bash
    echo "✅ Sourced tb3_delivery workspace"
fi

echo "🤖 Environment ready. ROS_DISTRO=$ROS_DISTRO  TURTLEBOT3_MODEL=$TURTLEBOT3_MODEL"
