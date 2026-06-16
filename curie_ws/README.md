# Curie Workspace

Welcome to the Curie WS! All of our custom (and modified) ROS2 packages are located here.

For more info on ROS2 (Humble), click [here](https://docs.ros.org/en/humble/index.html).

### Build
Ensure you are in the `curie_ws` directory,
```bash
cd /path/to/curie/curie_ws
```
Install all dependencies:
```bash
# Only need to call once
# ---------------------------------------------------
sudo apt update && sudo apt install python3-rosdep -y
sudo rosdep init
rosdep update
# ---------------------------------------------------
rosdep install --from-paths src --ignore-src -y
```
Build and Source ROS packages:
```bash
colcon build --symlink-install
source install/setup.bash
```
`source install/setup.bash` can be added to the `.bashrc` for your convenience.

### If Something Goes Wrong
If there is no obvious trace of an error, clean the WS and rebuild.
```bash
rm -rf build/ install/ log/ && colcon build --symlink-install
```

In the event the build crashes or the terminal freezes during a build (which can occur for very large packages), limit the number of cores used and, optionally, build them one at a time using `--executor sequential`
```bash
MAKEFLAGS="-j $(($(nproc) - 4))" colcon build --symlink-install --executor sequential
```

