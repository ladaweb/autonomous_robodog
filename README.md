# autonomous_robodog

Unitree Go2 EDU – ROS 2 + Gazebo Classic (Jammy/Humble) – End-to-End Guide

Tested on: Ubuntu 22.04 (jammy), ROS 2 Humble, Gazebo Classic 11
Goal: Bring up a Unitree Go2 EDU simulation, load controllers, and drive via /cmd_vel or a demo script.
Notes:

Gazebo Classic is EOL; it still works for this stack.

CHAMP/Go2 sim often exposes effort/trajectory controls. Walking gaits may require a gait node (high-level control).

We bias to Cyclone DDS to avoid FastDDS shared-memory issues.

Table of Contents

Prerequisites & System Setup

Workspace & Build

Environment Setup for Every Terminal

Bring-Up Options

Option A: Stack Launch (Recommended)

Option B: Manual Bring-Up (Debug)

Spawning the Robot

Robot State Publisher in a Namespace (if needed)

Controllers

Verify Topics & Drive

Runaway Recovery (Pause/Teleport/Delete/Respawn/Reset)

Helper Scripts

Troubleshooting & Gotchas

Appendix: Creating the go2_demos Package

1) Prerequisites & System Setup
1.1 Install tools and add keyrings/repos (ROS 2 + OSRF)
sudo apt-get update
sudo apt-get install -y curl wget gnupg lsb-release

# ROS 2 key → keyring
sudo mkdir -p /usr/share/keyrings
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
sudo chmod 644 /usr/share/keyrings/ros-archive-keyring.gpg

# OSRF (Gazebo) key → keyring
wget -qO- https://packages.osrfoundation.org/gazebo/gazebo.asc \
  | sudo gpg --dearmor -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
sudo chmod 644 /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

# Sources
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
| sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
| sudo tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null


If apt update complains about third-party repos (e.g., Spotify/Parrot), temporarily disable those *.list files in /etc/apt/sources.list.d/.

1.2 Install ROS 2 Humble + Gazebo + dev tools
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  python3-colcon-common-extensions python3-rosdep \
  xacro git

# rosdep
sudo rosdep init || true
rosdep update

1.3 Use system Python
# In shells where you build/run ROS:
conda deactivate 2>/dev/null || true
which python3   # expect: /usr/bin/python3

2) Workspace & Build
mkdir -p ~/go2_ws/src
cd ~/go2_ws/src
# (Place your Go2/CHAMP repos here, e.g., unitree-go2-ros2, champ_*)

cd ~/go2_ws
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install

# Source the overlay now (and each new terminal)
source ~/go2_ws/install/setup.bash

3) Environment Setup for Every Terminal
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.bash
source ~/go2_ws/install/setup.bash


Consider putting the two source lines in your ~/.bashrc. Keep RMW_IMPLEMENTATION explicit in shells where you run.

4) Bring-Up Options
Option A: Stack Launch (Recommended)
# Terminal A (keep in foreground)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.bash
source ~/go2_ws/install/setup.bash

ros2 launch go2_config gazebo.launch.py

Option B: Manual Bring-Up (Debug)
# Terminal A (keep in foreground; do NOT Ctrl+Z)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.bash
export GAZEBO_MODEL_DATABASE_URI=""

gzserver --verbose \
  -s libgazebo_ros_init.so \
  -s libgazebo_ros_factory.so \
  -s libgazebo_ros_api_plugin.so \
  /usr/share/gazebo-11/worlds/empty.world


Optional GUI:

gzclient --verbose
# GUI: View → Reset View (Ctrl+R), select 'go2' in the left tree, right-click → Follow

5) Spawning the Robot

If your launch already spawns the robot, skip this. Otherwise:

# Terminal B
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.bash
source ~/go2_ws/install/setup.bash

# Sanity
ros2 service list | grep /spawn_entity
ros2 topic echo --once /robot_description

# Spawn (namespaced to /go2 to avoid collisions)
ros2 run gazebo_ros spawn_entity.py -entity go2 -topic /robot_description -robot_namespace /go2

6) Robot State Publisher in a Namespace (if needed)

If gazebo_ros2_control complains about robot_state_publisher not found in /go2, run a namespaced RSP that feeds your URDF via params file (robust against quoting):

# Extract URDF cleanly (handles multi-doc YAML from ros2 topic echo)
ros2 topic echo --once /robot_description \
| python3 - <<'PY'
import sys,yaml
docs=list(yaml.safe_load_all(sys.stdin.read()))
for d in docs:
    if isinstance(d, dict) and 'data' in d:
        open('/tmp/go2.urdf','w').write(d['data']); print('OK'); break
PY

# Create params YAML embedding the URDF
{
  echo "/go2/robot_state_publisher:";
  echo "  ros__parameters:";
  echo "    use_sim_time: true";
  echo "    robot_description: |";
  sed 's/^/      /' /tmp/go2.urdf;
} > /tmp/go2_rsp.yaml

# Run RSP in /go2 (leave running)
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args --params-file /tmp/go2_rsp.yaml

7) Controllers

Controller manager path depends on your launch/namespacing:

# Pick the correct one for your setup:
CTRL_MGR=/controller_manager
# or:
# CTRL_MGR=/go2/controller_manager

# What’s loaded?
ros2 control list_controllers --controller-manager $CTRL_MGR
ros2 control list_hardware_interfaces --controller-manager $CTRL_MGR

# Joint state broadcaster (if not already active)
ros2 run controller_manager spawner joint_state_broadcaster --controller-manager $CTRL_MGR

# Try ONE motion controller (stop at first success)
for C in joint_trajectory_controller position_trajectory_controller \
         forward_position_controller joint_group_effort_controller; do
  ros2 run controller_manager spawner "$C" --controller-manager $CTRL_MGR && break
done


Seeing “already loaded”/“failed loading” is fine as long as you end up with one motion controller ACTIVE.

8) Verify Topics & Drive
# Use namespaced topics if your launch created them (e.g., /go2/odom, /go2/cmd_vel)
ros2 topic list | grep -E '^/(odom|cmd_vel)$'
ros2 topic info /cmd_vel         # should show Subscribers: ≥1
ros2 topic hz /odom              # ~50 Hz typical

Quick nudge (Twist)
ros2 topic pub -r 5 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.0}}"

Teleop (keyboard)
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/cmd_vel  -p scale_linear:=0.1 -p scale_angular:=0.5

Your square demo
ros2 run go2_demos square_driver \
  --ros-args -p cmd_vel_topic:=/cmd_vel -p odom_topic:=/odom
# (use /go2/… topics if namespaced)


If /cmd_vel has subscribers but nothing moves, your active controller may be trajectory-based. Start your gait/bridge node from CHAMP/Go2 so /cmd_vel translates to joint commands, or publish a JointTrajectory directly.

9) Runaway Recovery (Pause/Teleport/Delete/Respawn/Reset)

Pause physics or stop motion:

ros2 service call /gazebo/pause_physics std_srvs/srv/Empty "{}"
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{}"
ros2 service call /gazebo/unpause_physics std_srvs/srv/Empty "{}"


Teleport back to origin:

ros2 service call /gazebo/set_entity_state gazebo_msgs/srv/SetEntityState \
"{state: {name: go2,
          pose: {position: {x: 0.0, y: 0.0, z: 0.275}, orientation: {w: 1.0}},
          twist: {}}}"


Delete & respawn:

ros2 service call /gazebo/delete_entity gazebo_msgs/srv/DeleteEntity "{name: go2}"
ros2 run gazebo_ros spawn_entity.py -entity go2 -topic /robot_description -robot_namespace /go2


Reset world:

ros2 service call /gazebo/reset_world std_srvs/srv/Empty "{}"

10) Helper Scripts

Save these in, e.g., ~/Documents/research/robodog/scripts/, then chmod +x them.

10.1 reset_all.sh
#!/usr/bin/env bash
set -euo pipefail
pkill -f 'ros2 launch' 2>/dev/null || true
pkill -f gzserver      2>/dev/null || true
pkill -f gzclient      2>/dev/null || true
pkill -f spawn_entity  2>/dev/null || true
pkill -f controller_manager 2>/dev/null || true
pkill -f robot_state_publisher 2>/dev/null || true
sudo rm -f /dev/shm/fastrtps_* /dev/shm/fastdds_* /dev/shm/rtps_* 2>/dev/null || true
ros2 daemon stop >/dev/null 2>&1 || true
ros2 daemon start
echo "[reset] Done."

10.2 drive_forward.sh
#!/usr/bin/env bash
set -euo pipefail
TOPIC="${1:-/cmd_vel}"; DUR="${DUR:-4}"; SPEED="${SPEED:-0.15}"; ANG="${ANG:-0.0}"; RATE="${RATE:-5}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
source /opt/ros/humble/setup.bash
source ~/go2_ws/install/setup.bash 2>/dev/null || true
trap 'ros2 topic pub -1 "$TOPIC" geometry_msgs/msg/Twist "{}" >/dev/null 2>&1 || true' EXIT
timeout "${DUR}s" ros2 topic pub -r "${RATE}" "$TOPIC" geometry_msgs/msg/Twist \
  "{linear: {x: ${SPEED}}, angular: {z: ${ANG}}}"

10.3 respawn_go2.sh
#!/usr/bin/env bash
set -euo pipefail
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.bash
source ~/go2_ws/install/setup.bash
until ros2 service list | grep -q '/gazebo/pause_physics'; do sleep 0.5; done
ros2 service call /gazebo/pause_physics std_srvs/srv/Empty "{}" >/dev/null
ros2 service call /gazebo/delete_entity gazebo_msgs/srv/DeleteEntity "{name: go2}" >/dev/null || true
for i in {1..10}; do ros2 topic list | grep -qx /robot_description && break; sleep 0.5; done
ros2 run gazebo_ros spawn_entity.py -entity go2 -topic /robot_description -robot_namespace /go2
ros2 service call /gazebo/unpause_physics std_srvs/srv/Empty "{}" >/dev/null
echo "[respawn] Done."


Usage:

./reset_all.sh
# bring up sim (launch or manual)
./drive_forward.sh /cmd_vel
# if it runs away:
./respawn_go2.sh

11) Troubleshooting & Gotchas

FastDDS SHM errors (RTPS_TRANSPORT_SHM):
Use Cyclone DDS and clean SHM.

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
sudo rm -f /dev/shm/fastrtps_* /dev/shm/fastdds_* /dev/shm/rtps_*
sudo chmod 1777 /dev/shm


Port 11345 in use (Gazebo Classic master):

pgrep -fa gzserver
sudo ss -lptn 'sport = :11345'
sudo kill -9 <PID>


/robot_description from ros2 topic echo looks weird (multi-doc YAML):
Use the Python yaml.safe_load_all snippet above.

robot_state_publisher: robot_description must not be empty:
Feed it via params YAML (/tmp/go2_rsp.yaml) not an inline arg.

/controller_manager/list_controllers timeouts:
You’re hitting the wrong path. Try /controller_manager or /go2/controller_manager depending on your launch.

/cmd_vel has subscribers but robot doesn’t move:
Your active controller is trajectory-based. Start your gait/bridge node (CHAMP), or publish a JointTrajectory directly to the controller topic.

GUI warns about model database:
Set GAZEBO_MODEL_DATABASE_URI="" or ignore (local models still load).

Conda/pyenv interfering with python3:
Ensure which python3 → /usr/bin/python3 when building/running ROS.

12) Appendix: Creating the go2_demos Package

If you want your own nodes (e.g., a square driver) in the workspace:

cd ~/go2_ws/src
ros2 pkg create --build-type ament_python go2_demos --dependencies rclpy geometry_msgs nav_msgs

# Add your script (e.g., square_driver.py) into go2_demos/go2_demos/, and
# expose it via setup.py entry_points:
# entry_points={
#   'console_scripts': [
#       'square_driver = go2_demos.square_driver:main',
#   ],
# },

cd ~/go2_ws
colcon build --symlink-install
source ~/go2_ws/install/setup.bash

# Run it
ros2 run go2_demos square_driver \
  --ros-args -p cmd_vel_topic:=/cmd_vel -p odom_topic:=/odom

Quickstart (TL;DR)
# One-time:
#  - install ROS/Gazebo + keys + tools
#  - build workspace
# Every terminal:
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.bash
source ~/go2_ws/install/setup.bash

# Bring up:
ros2 launch go2_config gazebo.launch.py        # (Terminal A)

# Verify & drive (Terminal B):
ros2 topic list | grep -E '^/(odom|cmd_vel)$'
ros2 topic hz /odom
ros2 topic info /cmd_vel
ros2 topic pub -r 5 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
# or:
ros2 run go2_demos square_driver --ros-args -p cmd_vel_topic:=/cmd_vel -p odom_topic:=/odom
