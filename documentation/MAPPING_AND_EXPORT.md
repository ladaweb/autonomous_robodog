UNITREE SLAM – QUICK MAPPING & MAP EXPORT GUIDE

============================================
1. START SLAM SERVICE
============================================

export LD_LIBRARY_PATH=/unitree/module/unitree_slam/third_party_lib:/unitree/module/unitree_slam/lib:$LD_LIBRARY_PATH

cd /unitree/module/unitree_slam/bin
unset DISPLAY
QT_QPA_PLATFORM=offscreen ./unitree_slam

Wait until you see:
"server started. name:slam_operate"

Leave this running.


============================================
2. START LIDAR DRIVER
============================================

XT16:
cd /unitree/module/unitree_slam/bin
./xt16_driver

MID360 (if used):
./mid360_driver

Leave this running.


============================================
3. START KEY CONTROL
============================================

cd /unitree/module/unitree_slam/bin
./keyDemo eth0


============================================
4. START MAPPING
============================================

Press:
q

Output:
"Successfully started mapping"

Move the robot normally around the room.
Do NOT spin or move too fast.


============================================
5. STOP MAPPING & SAVE MAP
============================================

Press:
w

Output:
"Save pcd successfully"


============================================
6. FIND THE SAVED MAP
============================================

find /home -name "*.pcd" 2>/dev/null

Typical result:
/home/unitree/test.pcd


============================================
7. VIEW MAP ON ROBOT (OPTIONAL)
============================================

pcl_viewer /home/unitree/test.pcd


============================================
8. COPY MAP TO LAPTOP
============================================

On your laptop:

scp unitree@ROBOT_IP:/home/unitree/test.pcd .

Example:
scp unitree@192.168.123.18:/home/unitree/test.pcd .


============================================
9. VIEW MAP ON LAPTOP
============================================

pcl_viewer test.pcd

or open with CloudCompare


============================================
KEY CONTROLS SUMMARY
============================================

q  -> start mapping
w  -> stop & save map
f  -> clear task list
Ctrl+C -> exit


============================================
NOTES
============================================

- You do NOT need RViz to generate maps
- Small maps are expected for tests
- LiDAR captures environment as you move
- Avoid long sessions (>30 min) on dock PC
