# Unitree Go2 EDU — ROS 2 Humble + Gazebo Classic (Ubuntu 22.04)

This README is split into two parts so you can **run fast each session**:
- **Part 1:** Simulator (Gazebo Classic + ROS 2)
- **Part 2:** Physical Robot (Go2 EDU via Unitree SDK, optional ROS bridge)

> Tip: When in doubt, run one thing at a time. Keep an E-stop handy for the real robot.

---

## Part 1 — Run with the Simulator (Gazebo Classic + ROS 2)

### 1) Every session: configure each terminal
Copy-paste at the top of **every** terminal:
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.bash
source ~/go2_ws/install/setup.bash
```
> If your topics are namespaced (e.g. `/go2/cmd_vel`, `/go2/odom`), use those names below.

---

### 2) Terminal A — Start the simulator (leave running)
```bash
ros2 launch go2_config gazebo.launch.py
```
> If your launch already spawns the robot, you can skip the next step.

---

### 3) Terminal B — Spawn the robot (only if not auto-spawned)
```bash
# Sanity
ros2 service list | grep /spawn_entity
ros2 topic echo --once /robot_description

# Spawn into Gazebo
ros2 run gazebo_ros spawn_entity.py -entity go2 -topic /robot_description -robot_namespace /go2
```
If it says “waiting for /spawn_entity…”, make sure Terminal A is still running and not paused/suspended.

---

### 4) Run a simple script or command

#### 4.1 Quick nudge (no code)
```bash
ros2 topic info /cmd_vel        # should show "Subscription count: ≥1"
ros2 topic hz /odom             # should tick (~50 Hz), or try /odom/local if present

# Move forward slowly, Ctrl+C to stop
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.15}, angular: {z: 0.0}}"

# Emergency zero
# ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{}"
```

#### 4.2 Run **move.py** (forward 1 m → left 90° → right 90°)
**File location:** `~/Documents/research/robodog/autonomous_robodog/go2_demos/go2_demos/move.py`

**One-time make executable:**
```bash
chmod +x ~/Documents/research/robodog/autonomous_robodog/go2_demos/go2_demos/move.py
```

**Run (from anywhere):**
```bash
~/Documents/research/robodog/autonomous_robodog/go2_demos/go2_demos/move.py   --ros-args   -p cmd_vel_topic:=/cmd_vel   -p odom_topic:=/odom   -p odom_reliable:=false
# If your active odom is different (e.g. /odom/local), swap it:
#   -p odom_topic:=/odom/local
```

Alternative (without chmod):
```bash
python3 ~/Documents/research/robodog/autonomous_robodog/go2_demos/go2_demos/move.py   --ros-args -p cmd_vel_topic:=/cmd_vel -p odom_topic:=/odom -p odom_reliable:=false
```

**If it doesn’t move:**
```bash
ros2 service call /gazebo/unpause_physics std_srvs/srv/Empty "{}"
ros2 topic info /cmd_vel     # Subscription count ≥ 1
ros2 topic info /odom        # Publisher count ≥ 1 (or check /odom/local)
ros2 topic hz /odom          # should tick; else try /odom/local
```

---

### 5) Clean stop / reset (recommended before a fresh run)
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

**Emergency / runaway fix (without full restart):**
```bash
ros2 service call /gazebo/pause_physics std_srvs/srv/Empty "{}"
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{}"
ros2 service call /gazebo/unpause_physics std_srvs/srv/Empty "{}"

# Respawn just the robot if needed
ros2 service call /gazebo/pause_physics std_srvs/srv/Empty "{}"
ros2 service call /gazebo/delete_entity gazebo_msgs/srv/DeleteEntity "{name: go2}"
ros2 run gazebo_ros spawn_entity.py -entity go2 -topic /robot_description -robot_namespace /go2
ros2 service call /gazebo/unpause_physics std_srvs/srv/Empty "{}"
```

---

### 6) Optional helper scripts

**`stop_all.sh`**
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

**`drive_forward.sh`**
```bash
#!/usr/bin/env bash
set -euo pipefail
TOPIC="${1:-/cmd_vel}"; DUR="${DUR:-4}"; SPEED="${SPEED:-0.15}"; ANG="${ANG:-0.0}"; RATE="${RATE:-5}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
source /opt/ros/humble/setup.bash
source ~/go2_ws/install/setup.bash 2>/dev/null || true
trap 'ros2 topic pub -1 "$TOPIC" geometry_msgs/msg/Twist "{}" >/dev/null 2>&1 || true' EXIT
timeout "${DUR}s" ros2 topic pub -r "${RATE}" "$TOPIC" geometry_msgs/msg/Twist   "{linear: {x: ${SPEED}}, angular: {z: ${ANG}}}"
```

**`respawn_go2.sh`**
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

---

### 7) Simulator troubleshooting (quick hits)

- **RTPS_TRANSPORT_SHM errors** → using CycloneDDS & clean SHM usually fixes:
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
- **“waiting for /spawn_entity”** → make sure you launched `go2_config gazebo.launch.py` and didn’t suspend it.
- **`/cmd_vel` subscribers present but no motion** → controller may be trajectory-based; ensure your go2/CHAMP controllers are active as per your bringup.

---

## Part 2 — Run with the **Real Robot** (Go2 EDU)

> **Safety:** Clear area, spotter + E-stop. Start with tiny velocities.

### 1) Network + clean shell
```bash
# New terminal, no conda
conda deactivate 2>/dev/null || true
unset LD_PRELOAD
unset LD_LIBRARY_PATH

# Verify link: replace 'enp58s0' with your NIC if different
ip addr show enp58s0 | grep inet
ping -c 2 192.168.12.1
```
> Use the app/RC to put the robot in **Sport** and **Stand** (enabled).

### 2) Unitree SDK control (no ROS needed)

Go to the binaries and point the runtime loader to the SDK’s libs:
```bash
cd ~/Documents/research/robodog/unitree_sdk2/build/bin
export LD_LIBRARY_PATH=$PWD/../lib:$PWD/../thirdparty/install/lib
```

Run **non-moving** first (state only), then stand, then gentle motion:
```bash
# Read-only telemetry
sudo -E ./go2_robot_state_client enp58s0

# Safe: stand up / relax
sudo -E ./go2_stand_example enp58s0

# Gentle sport client (follow on-screen prompts; start with vx=0.10, yaw=0.0, 1–2 s)
sudo -E ./go2_sport_client enp58s0
```

**If you see `free(): invalid pointer`:**
- You’re likely linking wrong libs (e.g., to system `fmt/spdlog`). Check:
  ```bash
  ldd ./go2_stand_example | egrep 'fmt|spdlog'
  ```
  Paths must point **inside** your `unitree_sdk2/build/...`. If not, keep only the `LD_LIBRARY_PATH` above and try again (or rebuild with RPATH).

**Stopping:**
- Preferred: **Ctrl+C** in that terminal, or E-stop on the remote/app.
- Lost terminal?
  ```bash
  pgrep -fa 'go2_(sport_client|stand_example|robot_state_client)'
  sudo -E kill -INT <PID>   # then -TERM, then -KILL if needed
  ```

### 3) (Optional) Control the physical robot via **ROS 2** (`/cmd_vel`)
If you have a **ROS bridge** running on the robot or a PC (e.g., a Unitree ROS2 bringup) that exposes `/cmd_vel` and publishes odometry:

**On your laptop:**
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.bash

# Verify topics from the bridge:
ros2 topic list | grep -E '/cmd_vel|/odom'

# Nudge forward:
ros2 topic pub -r 5 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.10}}"

# Or run move.py (adjust topics if namespaced, e.g., /go2/cmd_vel, /go2/odom)
python3 ~/Documents/research/robodog/autonomous_robodog/go2_demos/go2_demos/move.py   --ros-args -p cmd_vel_topic:=/cmd_vel -p odom_topic:=/odom -p odom_reliable:=false
```

**If `/cmd_vel` has no subscribers or no odom is published:** your bridge isn’t running or isn’t exposing the expected topics—start/adjust the bridge on the robot per its bringup instructions, then re-check `ros2 topic list`.

---

## Appendix — Full installation (reference, only for new machines)

### Add keyrings & repos
```bash
sudo apt-get update
sudo apt-get install -y curl wget gnupg lsb-release
sudo mkdir -p /usr/share/keyrings

curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key   | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
sudo chmod 644 /usr/share/keyrings/ros-archive-keyring.gpg

wget -qO- https://packages.osrfoundation.org/gazebo/gazebo.asc   | sudo gpg --dearmor -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
sudo chmod 644 /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null
```

### Install ROS 2 + Gazebo + tools
```bash
sudo apt update
sudo apt install -y   ros-humble-desktop   ros-humble-gazebo-ros-pkgs   ros-humble-ros2-control ros-humble-ros2-controllers   python3-colcon-common-extensions python3-rosdep   xacro git

sudo rosdep init || true
rosdep update
```

### Workspace setup (once)
```bash
mkdir -p ~/go2_ws/src
cd ~/go2_ws/src
# clone your Unitree/CHAMP repos here

cd ~/go2_ws
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
```

> Afterwards, start each terminal with:
> ```bash
> export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
> source /opt/ros/humble/setup.bash
> source ~/go2_ws/install/setup.bash
> ```

---

### One last reminder
- **Simulator:** Start sim (A), spawn if needed (B), run scripts (C), clean stop (D).
- **Real robot:** Use Unitree SDK first (stand → small nudges), optional ROS bridge afterward.
- Keep the area clear, go slow, E-stop within reach.
