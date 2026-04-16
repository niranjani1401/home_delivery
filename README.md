# 🏠 Home Cleaning Robot (TurtleBot3 + ROS 2)

Autonomous indoor home-service cleaning robot built with **TurtleBot3**, **Nav2**, and **SLAM Toolbox**.  
Supports both:

- **ROS 2 Humble** (Ubuntu 22.04)
- **ROS 2 Jazzy** (Ubuntu 24.04)

The project includes:

- Autonomous multi-location cleaning
- Emergency stop (manual + obstacle-triggered)
- Cleaning queue and history
- Battery simulation + auto return-to-base
- Terminal-based Robot UI
- Automatic map generation script (`generate_map.py`)
- Navigation launch for simulation and autonomous cleaning

---

## 📦 Repository Structure

```text
home_cleaning/
├── scripts/
│   └── setup_env.sh
└── src/
    └── tb3_delivery/
        ├── package.xml
        ├── setup.py
        ├── launch/
        │   ├── mapping.launch.py
        │   └── navigation.launch.py
        ├── config/
        ├── maps/
        ├── worlds/
        └── tb3_delivery/
            ├── delivery_node.py
            ├── emergency_stop_node.py
            ├── robot_ui.py
            └── generate_map.py
```

---

## Features

- ✅ ROS 2 package using `ament_python`
- ✅ Works on both Humble and Jazzy
- ✅ Handles Gazebo Classic (Humble) and new Gazebo (`ros_gz_sim`, Jazzy)
- ✅ Autonomous cleaning at predefined locations (kitchen, living_room, bedroom, bathroom, charging_station)
- ✅ Emergency stop via keyboard and LaserScan safety threshold
- ✅ Cleaning task cancel support
- ✅ UI for status, battery, queue, and history
- ✅ Automatic map generation workflow

---

## 🧰 Requirements

- TurtleBot3 simulation dependencies
- Nav2
- SLAM Toolbox
- Gazebo stack (depends on distro)
- `colcon`, `rosdep`

---

# 🐢 ROS 2 Humble Setup (Ubuntu 22.04)

## 1) Install ROS 2 Humble + tools

```bash
sudo apt update
sudo apt install -y software-properties-common curl git
sudo add-apt-repository universe -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
| sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions python3-rosdep python3-vcstool
```

## 2) Install project dependencies (Humble)

```bash
sudo apt update
sudo apt install -y \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-turtlebot3-gazebo \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-twist-stamper
```

## 3) Create workspace and clone repository

```bash
mkdir -p ~/tb3_delivery_ws/src
cd ~/tb3_delivery_ws/src
git clone https://github.com/niranjani1401/home_delivery.git
```

## 4) Install package dependencies + build

```bash
sudo rosdep init || true
rosdep update

cd ~/tb3_delivery_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

## 5) Source environment (every new terminal)

```bash
source /opt/ros/humble/setup.bash
source ~/tb3_delivery_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
```

---

# 🐬 ROS 2 Jazzy Setup (Ubuntu 24.04)

## 1) Install ROS 2 Jazzy + tools

```bash
sudo apt update
sudo apt install -y software-properties-common curl git
sudo add-apt-repository universe -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /env/os-release && echo $UBUNTU_CODENAME) main" \
| sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-jazzy-desktop python3-colcon-common-extensions python3-rosdep python3-vcstool
```

## 2) Install project dependencies (Jazzy)

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-turtlebot3-gazebo \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-twist-stamper
```

## 3) Create workspace and clone repository

```bash
mkdir -p ~/tb3_delivery_ws/src
cd ~/tb3_delivery_ws/src
git clone https://github.com/niranjani1401/home_delivery.git
```

## 4) Install package dependencies + build

```bash
sudo rosdep init || true
rosdep update

cd ~/tb3_delivery_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

## 5) Source environment (every new terminal)

```bash
source /opt/ros/jazzy/setup.bash
source ~/tb3_delivery_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
```

---

# 🗺️ Map Generation (Automatic, no teleop)

Use your automatic map generator script:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/tb3_delivery_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 run tb3_delivery generate_map
```

If `generate_map` is not exposed as a console script, run directly:

```bash
python3 ~/tb3_delivery_ws/src/home_delivery/src/tb3_delivery/tb3_delivery/generate_map.py
```

Generated map files should be saved in:

```text
src/tb3_delivery/maps/
```

---

# 🧭 Run Navigation + Cleaning

## Terminal 1: Start navigation stack

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/tb3_delivery_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch tb3_delivery navigation.launch.py
```

## Terminal 2: Start Robot UI

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/tb3_delivery_ws/install/setup.bash
ros2 run tb3_delivery robot_ui
```

---

# 🧪 Useful Test Commands

Send cleaning request manually:

```bash
ros2 topic pub /delivery_request std_msgs/msg/String "{data: 'kitchen'}" --once
```

Trigger emergency stop:

```bash
ros2 topic pub /emergency_stop std_msgs/msg/Bool "{data: true}" --once
```

Clear emergency stop:

```bash
ros2 topic pub /emergency_stop std_msgs/msg/Bool "{data: false}" --once
```

Cancel cleaning:

```bash
ros2 topic pub /cancel_delivery std_msgs/msg/Bool "{data: true}" --once
```

---

# ⚠️ Troubleshooting

## 1) `Package 'tb3_delivery' not found`
- You forgot to source workspace:
  ```bash
  source ~/tb3_delivery_ws/install/setup.bash
  ```

## 2) Gazebo models not loading (Humble)
- Ensure:
  ```bash
  export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
  ```

## 3) `generate_map` command not found
- Use direct python execution:
  ```bash
  python3 ~/tb3_delivery_ws/src/home_delivery/src/tb3_delivery/tb3_delivery/generate_map.py
  ```

## 4) Build errors after dependency changes
```bash
cd ~/tb3_delivery_ws
rm -rf build install log
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install
```
