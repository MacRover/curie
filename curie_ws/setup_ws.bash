#!/bin/bash

SOURCE_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd)
BIN_PATH="src/curie_hw_control/lib/spark_mmrt/bin"
DEPS="python3-colcon-common-extensions python3-rosdep libncurses5-dev libncursesw5-dev ros-humble-cyclonedds ros-humble-rmw-cyclonedds-cpp"

# One time setup
if [[ "$PATH" != *"$SOURCE_DIR/$BIN_PATH"* ]]; then
    # First populate submodules before installing dependencies
    git submodule update --init --recursive
    sudo apt update && sudo apt install $DEPS -y
    sudo rosdep init
    rosdep update
    rosdep install --from-paths $SOURCE_DIR/src --ignore-src -y

    echo "export PATH=\"${SOURCE_DIR}/${BIN_PATH}:\$PATH"\" >> "$HOME/.bashrc"
    if [[ -d "$SOURCE_DIR/$BIN_PATH" ]]; then
        echo "Added $SOURCE_DIR/$BIN_PATH to PATH"
    else
        echo "Warning: $SOURCE_DIR/$BIN_PATH does not exist, please ensure the WS is built"
    fi
    # Needed since moveit and nav2 uses Cyclone DDS
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> "$HOME/.bashrc"
    
    echo "Setup complete, source your .bashrc for changes to take effect"
else
    rosdep install --from-paths $SOURCE_DIR/src --ignore-src -y
fi