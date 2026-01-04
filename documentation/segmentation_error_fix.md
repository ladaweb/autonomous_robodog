## Fix: `unitree_slam` segfault / `free(): invalid pointer` (CycloneDDS `libddsc*` mismatch)

### Symptom
Running:
- `/unitree/module/unitree_slam/bin/unitree_slam`

crashed with:
- `Segmentation fault (core dumped)` or `free(): invalid pointer`

### Root cause
`unitree_slam` was dynamically linking against an incompatible `CycloneDDS` runtime (`libddsc.so.0` / `libddscxx.so.0`) from `/usr/local/lib`.

### What fixed it (steps)

1) **Back up the conflicting DDS libraries from `/usr/local/lib`**
```bash
sudo mkdir -p /usr/local/lib/dds_backup
sudo mv /usr/local/lib/libddsc* /usr/local/lib/dds_backup/
sudo ldconfig```

###
2) **Create a local library folder for SLAM and point it to the working DDS libs**

sudo mkdir -p /unitree/module/unitree_slam/third_party_lib

# libddsc.so.0 -> use the Unitree SDK aarch64 build (works)
sudo ln -sf /home/unitree/unitree_sdk2-main/thirdparty/lib/aarch64/libddsc.so.0 \
  /unitree/module/unitree_slam/third_party_lib/libddsc.so.0

# libddscxx.so.0 -> use the known-good one from the backup (15MB aarch64 ELF)
sudo ln -sf /usr/local/lib/dds_backup/libddscxx.so \
  /unitree/module/unitree_slam/third_party_lib/libddscxx.so.0`

 3) **Run SLAM with the local library path first**

cd /unitree/module/unitree_slam/bin
export LD_LIBRARY_PATH=/unitree/module/unitree_slam/third_party_lib:/unitree/module/unitree_slam/lib:$LD_LIBRARY_PATH
unset DISPLAY
export QT_QPA_PLATFORM=offscreen
./unitree_slam

**Expected result**

unitree_slam starts successfully and prints:

xt16 lidar ysn check success!

server start ...

server started. name:slam_operate ...

