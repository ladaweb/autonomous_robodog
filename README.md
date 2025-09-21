# Unitree Go2 EDU — ROS 2 Humble + Gazebo Classic (Ubuntu 22.04) — End‑to‑End Guide

**Goal:** Clean install, build, bring up sim, control the robot, and recover when it “runs away.”  
**Tested with:** Ubuntu 22.04 (jammy), ROS 2 Humble, Gazebo Classic 11.

> Gazebo **Classic** is deprecated but still works for this stack. We prefer **Cyclone DDS** to avoid FastDDS SHM issues.

---

## TL;DR (Quickstart)

```bash
# Every terminal you use for ROS
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.bash
source ~/go2_ws/install/setup.bash

# Launch everything (Terminal A, foreground)
ros2 launch go2_config gazebo.launch.py

# Drive (Terminal B)
ros2 topic info /cmd_vel          # should show Subscribers: ≥1
ros2 topic hz /odom               # ~50 Hz
ros2 topic pub -r 5 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
# or your demo:
ros2 run go2_demos square_driver --ros-args -p cmd_vel_topic:=/cmd_vel -p odom_topic:=/odom
```

If it runs away:
```bash
ros2 service call /gazebo/pause_physics std_srvs/srv/Empty "{}"
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{}"
ros2 service call /gazebo/unpause_physics std_srvs/srv/Empty "{}"
```

To *stop everything* and reset, use **Stop‑All** snippet in [§9](#9-stop-all-processes-one-shot) or the provided `stop_all.sh` in [§10.1](#101-stop_allsh).

---

## 1) Prerequisites & System Setup

### 1.1 Add official APT keyrings & repos (ROS 2 + OSRF)

```bash
sudo apt-get update
sudo apt-get install -y curl wget gnupg lsb-release

# Keyrings
sudo mkdir -p /usr/share/keyrings

# ROS 2 key
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
sudo chmod 644 /usr/share/keyrings/ros-archive-keyring.gpg

# OSRF (Gazebo) key
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
```

> If `apt update` errors on unrelated third‑party repos (Spotify/Parrot/etc.), temporarily rename their `.list` files in `/etc/apt/sources.list.d/` and re‑run `sudo apt update`.

### 1.2 Install ROS 2 Humble, Gazebo Classic, build tools

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  python3-colcon-common-extensions python3-rosdep \
  xacro git

# Initialize rosdep (idempotent)
sudo rosdep init || true
rosdep update
```

### 1.3 Ensure you use **system Python** (not conda/pyenv)

```bash
# In any shell you use for ROS builds/runs:
conda deactivate 2>/dev/null || true
which python3   # expect: /usr/bin/python3
```

---

## 2) Workspace & Build

```bash
mkdir -p ~/go2_ws/src
cd ~/go2_ws/src
# Place/clone your Unitree/CHAMP packages here (e.g., unitree-go2-ros2, champ_*).

cd ~/go2_ws
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install

# Source overlay (do this in every ROS terminal)
source ~/go2_ws/install/setup.bash
```

---

## 3) Environment (do this in EVERY terminal)

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.bash
source ~/go2_ws/install/setup.bash
```

> You may add the two `source` lines to `~/.bashrc`. Keep the `RMW_IMPLEMENTATION` export explicit for clarity.

---

## 4) Bring‑Up Options

### 4.1 Option A — Use your stack launch (recommended)

```bash
# Terminal A (keep running, don’t Ctrl+Z)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.bash
source ~/go2_ws/install/setup.bash

ros2 launch go2_config gazebo.launch.py
```

### 4.2 Option B — Manual (debug)

```bash
# Terminal A
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.bash
export GAZEBO_MODEL_DATABASE_URI=""   # avoid network fetch

gzserver --verbose \
  -s libgazebo_ros_init.so \
  -s libgazebo_ros_factory.so \
  -s libgazebo_ros_api_plugin.so \
  /usr/share/gazebo-11/worlds/empty.world
```

**Optional GUI:**
```bash
gzclient --verbose
# GUI: View → Reset View (Ctrl+R), select 'go2', right‑click → Follow
```

---

## 5) Spawn the Robot

If your launch already spawns it, skip this.

```bash
# Terminal B
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.bash
source ~/go2_ws/install/setup.bash

# Sanity
ros2 service list | grep /spawn_entity
ros2 topic echo --once /robot_description

# Spawn (namespaced is tidy)
ros2 run gazebo_ros spawn_entity.py -entity go2 -topic /robot_description -robot_namespace /go2
```

---

## 6) Namespaced `robot_state_publisher` (only if plugin complains)

If `gazebo_ros2_control` says `robot_state_publisher` not available in `/go2`, run a **/go2** RSP fed by a URDF via params (robust to quoting).

```bash
# Extract clean URDF from /robot_description (handles multi‑doc YAML)
ros2 topic echo --once /robot_description \
| python3 - <<'PY'
import sys,yaml
docs=list(yaml.safe_load_all(sys.stdin.read()))
for d in docs:
    if isinstance(d, dict) and 'data' in d:
        open('/tmp/go2.urdf','w').write(d['data']); print('OK'); break
PY

# Build params YAML
{
  echo "/go2/robot_state_publisher:";
  echo "  ros__parameters:";
  echo "    use_sim_time: true";
  echo "    robot_description: |";
  sed 's/^/      /' /tmp/go2.urdf;
} > /tmp/go2_rsp.yaml

# Run RSP (keep running)
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args --params-file /tmp/go2_rsp.yaml
```

---

## 7) Controllers

Your controller manager path depends on launch/namespacing. Pick the correct one:

```bash
CTRL_MGR=/controller_manager
# or
# CTRL_MGR=/go2/controller_manager

# Inspect
ros2 control list_controllers --controller-manager $CTRL_MGR
ros2 control list_hardware_interfaces --controller-manager $CTRL_MGR

# Joint state broadcaster (if not active)
ros2 run controller_manager spawner joint_state_broadcaster --controller-manager $CTRL_MGR

# Try ONE motion controller (stop at first success)
for C in joint_trajectory_controller position_trajectory_controller \
         forward_position_controller joint_group_effort_controller; do
  ros2 run controller_manager spawner "$C" --controller-manager $CTRL_MGR && break
done
```

> It’s fine to see “already loaded/failed loading” messages as long as you end up with **one** motion controller **ACTIVE**.

---

## 8) Verify & Drive

```bash
# Use namespaced topics if your launch uses them (e.g., /go2/odom, /go2/cmd_vel)
ros2 topic list | grep -E '^/(odom|cmd_vel)$'
ros2 topic info /cmd_vel          # Subscribers: ≥1 means something is listening
ros2 topic hz /odom               # ~50 Hz typical
```

**Quick nudge:**
```bash
ros2 topic pub -r 5 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.0}}"
```

**Keyboard teleop:**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/cmd_vel -p scale_linear:=0.1 -p scale_angular:=0.5
```

**Your square demo:**
```bash
ros2 run go2_demos square_driver \
  --ros-args -p cmd_vel_topic:=/cmd_vel -p odom_topic:=/odom
# (Use /go2/... topics if namespaced)
```

> If `/cmd_vel` has subscribers but the robot doesn’t move, your active controller may be trajectory‑based. Start your gait/bridge node (CHAMP/Go2), or publish a `JointTrajectory` directly.

---

## 9) Stop All Processes (one‑shot)

Run this anytime to nuke ROS/Gazebo leftovers and free port 11345:

```bash
pkill -f 'ros2 launch'            2>/dev/null || true
pkill -f gzserver                 2>/dev/null || true
pkill -f gzclient                 2>/dev/null || true
pkill -f spawn_entity.py          2>/dev/null || true
pkill -f controller_manager       2>/dev/null || true
pkill -f robot_state_publisher    2>/dev/null || true
pkill -f champ_gazebo/contact_sensor 2>/dev/null || true

# Free Gazebo master port if still listening
if sudo ss -lptn 'sport = :11345' 2>/dev/null | grep -q LISTEN; then
  sudo ss -lptn 'sport = :11345'
  PID=$(sudo ss -lptn 'sport = :11345' | awk '/LISTEN/ {print $NF}' | sed 's/users:(("//; s/")).*$//; s/,.*$//')
  sudo kill -9 "$PID" 2>/dev/null || true
fi

# Clean DDS shared memory (safe)
sudo rm -f /dev/shm/fastrtps_* /dev/shm/fastdds_* /dev/shm/rtps_* 2>/dev/null || true
sudo chmod 1777 /dev/shm

# Optional: clear caches
rm -rf ~/.gazebo ~/.ignition 2>/dev/null || true

# Restart ROS discovery
ros2 daemon stop  >/dev/null 2>&1 || true
ros2 daemon start >/dev/null 2>&1 || true

# Verify
pgrep -fa 'gz(server|client)|gazebo|gazebo_ros' || echo "[ok] no gazebo/ros procs"
sudo ss -lptn 'sport = :11345' || true
```

---

## 10) Helper Scripts

Put these in e.g. `~/Documents/research/robodog/scripts/` and `chmod +x` them.

### 10.1 `stop_all.sh`

```bash
#!/usr/bin/env bash
set -euo pipefail
pkill -f 'ros2 launch'            2>/dev/null || true
pkill -f gzserver                 2>/dev/null || true
pkill -f gzclient                 2>/dev/null || true
pkill -f spawn_entity.py          2>/dev/null || true
pkill -f controller_manager       2>/dev/null || true
pkill -f robot_state_publisher    2>/dev/null || true
pkill -f champ_gazebo/contact_sensor 2>/dev/null || true
if sudo ss -lptn 'sport = :11345' 2>/dev/null | grep -q LISTEN; then
  sudo ss -lptn 'sport = :11345'
  PID=$(sudo ss -lptn 'sport = :11345' | awk '/LISTEN/ {print $NF}' | sed 's/users:(("//; s/")).*$//; s/,.*$//')
  sudo kill -9 "$PID" 2>/dev/null || true
fi
sudo rm -f /dev/shm/fastrtps_* /dev/shm/fastdds_* /dev/shm/rtps_* 2>/dev/null || true
sudo chmod 1777 /dev/shm
rm -rf ~/.gazebo ~/.ignition 2>/dev/null || true
ros2 daemon stop  >/dev/null 2>&1 || true
ros2 daemon start >/dev/null 2>&1 || true
pgrep -fa 'gz(server|client)|gazebo|gazebo_ros' || echo "[ok] no gazebo/ros procs"
sudo ss -lptn 'sport = :11345' || true
echo "[done] all stopped."
```

### 10.2 `drive_forward.sh`

```bash
#!/usr/bin/env bash
set -euo pipefail
TOPIC="${1:-/cmd_vel}"; DUR="${DUR:-4}"; SPEED="${SPEED:-0.15}"; ANG="${ANG:-0.0}"; RATE="${RATE:-5}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
source /opt/ros/humble/setup.bash
source ~/go2_ws/install/setup.bash 2>/dev/null || true
trap 'ros2 topic pub -1 "$TOPIC" geometry_msgs/msg/Twist "{}" >/dev/null 2>&1 || true' EXIT
timeout "${DUR}s" ros2 topic pub -r "${RATE}" "$TOPIC" geometry_msgs/msg/Twist \
  "{linear: {x: ${SPEED}}, angular: {z: ${ANG}}}"
```

### 10.3 `respawn_go2.sh`

```bash
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
```

**Usage:**
```bash
./stop_all.sh
# (re-)launch your sim
./drive_forward.sh /cmd_vel         # or /go2/cmd_vel
# if it runs away:
./respawn_go2.sh
```

---

## 11) Troubleshooting

- **FastDDS SHM errors** (`RTPS_TRANSPORT_SHM`) → use Cyclone DDS & clean SHM:
  ```bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  sudo rm -f /dev/shm/fastrtps_* /dev/shm/fastdds_* /dev/shm/rtps_*; sudo chmod 1777 /dev/shm
  ```

- **Port 11345 busy** (Gazebo master):
  ```bash
  pgrep -fa gzserver
  sudo ss -lptn 'sport = :11345'
  sudo kill -9 <PID>
  ```

- **`/robot_description` from `ros2 topic echo` is multi‑doc YAML** → use the Python `yaml.safe_load_all` snippet in §6.

- **`robot_state_publisher`: “robot_description must not be empty”** → feed via params YAML (`/tmp/go2_rsp.yaml`).

- **Controller manager timeouts** → wrong path; try `/controller_manager` vs `/go2/controller_manager`.

- **`/cmd_vel` has subscribers but robot doesn’t move** → your active controller may be trajectory‑based. Start the gait/bridge node or publish a `JointTrajectory` directly to the JTC topic.

- **Model DB warnings in GUI** → set `GAZEBO_MODEL_DATABASE_URI=""` or ignore (local models still load).

- **Conda/pyenv interfering** → ensure `which python3` → `/usr/bin/python3` before building.

---

## 12) Appendix — Create `go2_demos` (optional)

```bash
cd ~/go2_ws/src
ros2 pkg create --build-type ament_python go2_demos --dependencies rclpy geometry_msgs nav_msgs

# Add script go2_demos/go2_demos/square_driver.py and expose in setup.py:
# entry_points={'console_scripts': ['square_driver = go2_demos.square_driver:main']}

cd ~/go2_ws
colcon build --symlink-install
source ~/go2_ws/install/setup.bash

ros2 run go2_demos square_driver \
  --ros-args -p cmd_vel_topic:=/cmd_vel -p odom_topic:=/odom
```
