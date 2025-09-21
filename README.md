# Unitree Go2 EDU — ROS 2 Humble + Gazebo Classic (Ubuntu 22.04)

This README is organized so you can **get running, fast** each session, and only then dig into scripts, troubleshooting, and full install details.

---

## A) Every Time You Want to Run (Clean, Minimal Steps)

> Assumes everything is already installed and your workspace is built.  
> We’ll use **two terminals**: one for the simulator, one for your commands/scripts.

### A.1 Configure each terminal (copy-paste at the top of *every* terminal)

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.bash
source ~/go2_ws/install/setup.bash
```

> If your stack uses namespaced topics (e.g. `/go2/cmd_vel`, `/go2/odom`), remember to use those names in step A.3/A.4.

---

### A.2 Terminal A — Start the simulator (leave it running in the foreground)

```bash
ros2 launch go2_config gazebo.launch.py
```

(If your launch already spawns the robot, you can skip A.3.)

---

### A.3 Terminal B — Spawn the robot (only if your launch didn’t already)

```bash
# Still make sure the terminal is configured (A.1)
ros2 service list | grep /spawn_entity              # sanity check
ros2 topic echo --once /robot_description           # should print XML-like string
ros2 run gazebo_ros spawn_entity.py -entity go2 -topic /robot_description -robot_namespace /go2
```

> If spawn says “waiting for service to become available…”, your simulator isn’t running *with ROS API* (make sure Terminal A is on `gazebo.launch.py` and not paused/stopped; do not Ctrl+Z).

---

### A.4 Terminal B — Run a **simple script** (publish `/cmd_vel` for a few seconds)

**Option 1: one-liner Twist publisher (quick nudge):**
```bash
ros2 topic info /cmd_vel          # should show "Subscription count: ≥1"
ros2 topic hz /odom               # ~50 Hz typical
ros2 topic pub -r 5 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.15}, angular: {z: 0.0}}"
# Ctrl+C to stop; or send a single zero Twist:
# ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{}"
```

**Option 2: your demo node (square path):**
```bash
ros2 run go2_demos square_driver \
  --ros-args -p cmd_vel_topic:=/cmd_vel -p odom_topic:=/odom
```

> If your topics are namespaced (e.g. `/go2/cmd_vel`, `/go2/odom`), pass those in the parameters above.

**If the robot runs away**:  
Pause → stop Twist → unpause:
```bash
ros2 service call /gazebo/pause_physics std_srvs/srv/Empty "{}"
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{}"
ros2 service call /gazebo/unpause_physics std_srvs/srv/Empty "{}"
```

**If you need to respawn**:
```bash
ros2 service call /gazebo/pause_physics std_srvs/srv/Empty "{}"
ros2 service call /gazebo/delete_entity gazebo_msgs/srv/DeleteEntity "{name: go2}"
ros2 run gazebo_ros spawn_entity.py -entity go2 -topic /robot_description -robot_namespace /go2
ros2 service call /gazebo/unpause_physics std_srvs/srv/Empty "{}"
```

---

### A.5 Clean stop (recommended before a fresh run)

```bash
# Stop everything & free Gazebo port (11345)
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
ros2 daemon stop  >/dev/null 2>&1 || true
ros2 daemon start >/dev/null 2>&1 || true
```

---

## B) Helper Scripts (optional, but handy)

Put these in e.g. `~/Documents/research/robodog/scripts/` and `chmod +x` them.

### B.1 `stop_all.sh`
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

### B.2 `drive_forward.sh`
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

### B.3 `respawn_go2.sh`
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

**Usage pattern:**
```bash
./stop_all.sh
# Start sim (Terminal A): ros2 launch go2_config gazebo.launch.py
# Then drive in Terminal B:
./drive_forward.sh /cmd_vel
# If it runs off:
./respawn_go2.sh
```

---

## C) Troubleshooting (the greatest hits)

- **FastDDS SHM errors** (`RTPS_TRANSPORT_SHM`) → use Cyclone + clean SHM:
  ```bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  sudo rm -f /dev/shm/fastrtps_* /dev/shm/fastdds_* /dev/shm/rtps_*; sudo chmod 1777 /dev/shm
  ```

- **Gazebo port 11345 busy**:
  ```bash
  pgrep -fa gzserver
  sudo ss -lptn 'sport = :11345'
  sudo kill -9 <PID>
  ```

- **`waiting for service /spawn_entity`** → Terminal A isn’t running the sim with ROS API (use `ros2 launch go2_config gazebo.launch.py`; don’t suspend it).

- **`/robot_description` from `ros2 topic echo` looks odd** → it’s multi-doc YAML; don’t worry if it prints with `---` separators.

- **`robot_state_publisher: robot_description must not be empty`** → feed URDF via params file (see full version in the install section below).

- **`/controller_manager/list_controllers` timeouts** → wrong manager path; try `/controller_manager` vs `/go2/controller_manager` depending on how you launched.

- **`/cmd_vel` has subscribers but robot doesn’t move** → your active controller is trajectory-based; start the gait/bridge node (CHAMP/Go2) or publish a `JointTrajectory` directly to the controller topic.

- **GUI model DB warnings** → set `GAZEBO_MODEL_DATABASE_URI=""` or ignore (local models still load).

- **Conda/pyenv interference** → ensure `which python3` → `/usr/bin/python3` when building/running.

---

## D) Full Installation & Downloads (reference)

> Only needed when setting up a new machine; otherwise skip to A).

### D.1 Add keyrings & repos

```bash
sudo apt-get update
sudo apt-get install -y curl wget gnupg lsb-release

sudo mkdir -p /usr/share/keyrings

curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
sudo chmod 644 /usr/share/keyrings/ros-archive-keyring.gpg

wget -qO- https://packages.osrfoundation.org/gazebo/gazebo.asc \
  | sudo gpg --dearmor -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
sudo chmod 644 /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
| sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
| sudo tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null
```

### D.2 Install ROS 2 Humble + Gazebo + tools

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  python3-colcon-common-extensions python3-rosdep \
  xacro git

sudo rosdep init || true
rosdep update
```

### D.3 Workspace

```bash
mkdir -p ~/go2_ws/src
cd ~/go2_ws/src
# clone your Unitree/CHAMP repos here

cd ~/go2_ws
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install

# Each terminal:
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.bash
source ~/go2_ws/install/setup.bash
```

### D.4 Optional: Namespaced robot_state_publisher with URDF params

```bash
# Extract URDF from topic (multi-doc safe)
ros2 topic echo --once /robot_description \
| python3 - <<'PY'
import sys,yaml
docs=list(yaml.safe_load_all(sys.stdin.read()))
for d in docs:
    if isinstance(d, dict) and 'data' in d:
        open('/tmp/go2.urdf','w').write(d['data']); print('OK'); break
PY

# Params YAML
{
  echo "/go2/robot_state_publisher:";
  echo "  ros__parameters:";
  echo "    use_sim_time: true";
  echo "    robot_description: |";
  sed 's/^/      /' /tmp/go2.urdf;
} > /tmp/go2_rsp.yaml

# Run it:
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args --params-file /tmp/go2_rsp.yaml
```

---

## E) Appendix — Create `go2_demos` (optional)

```bash
cd ~/go2_ws/src
ros2 pkg create --build-type ament_python go2_demos --dependencies rclpy geometry_msgs nav_msgs

# Add file: go2_demos/go2_demos/square_driver.py
# In setup.py, expose it:
# entry_points={'console_scripts': ['square_driver = go2_demos.square_driver:main']}

cd ~/go2_ws
colcon build --symlink-install
source ~/go2_ws/install/setup.bash

ros2 run go2_demos square_driver \
  --ros-args -p cmd_vel_topic:=/cmd_vel -p odom_topic:=/odom
```

---

### One last reminder (for smooth runs)
- Configure *each* terminal with A.1.
- Start sim in **Terminal A** (A.2).
- **Terminal B** for spawn (if needed) and scripts (A.3/A.4).
- Use the **clean stop** block (A.5) or `stop_all.sh` when resetting.
