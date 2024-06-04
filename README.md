# Repository for ROS2 and Axcend Focus LC

# VS code addons

Use color blocks to see sections of related code

## Workflow for building ros2 package and getting them on the board

### Setup

Open the Ubuntu WSL VM and build server VM

source /opt/ros/humble/setup.bash

source install/setup.bash

### Clean Workspace

1. cd /mnt/c/Users/gupyf/Documents/GitHub/ros2_ws
2. rm -r build install log
3. cd /mnt/c/Users/gupyf/Documents/GitHub/ros2_ws/src/axcend_focus
4. code .

### Build Package
1. cd /mnt/c/Users/gupyf/Documents/GitHub/ros2_ws
2. colcon build --packages-select axcend_focus_custom_interfaces
3. colcon build --packages-select axcend_focus_front_panel_button

### Source Workspace
1. source install/setup.bash 

### Update Changelog
0. cd /mnt/c/Users/gupyf/Documents/GitHub/ros2_ws/src/axcend_focus
1. catkin_generate_changelog
2. Commit the files
3. catkin_prepare_release
4. bloom-release --rosdistro foxy axcend_focus
5. https://github.com/grahas/axcend_focus-release.git

https://github.com/grahas/axcend_focus.git

### Rebuild Package Index

In build-server VM ->

0. In the rosdistro workspace in the VM
1. export ROSDISTRO_INDEX_URL=file:///home/axcend/Documents/GitHub/rosdistro/index-v4.yaml
2. update the version number in distribution.yaml
3. cd ~/Documents/GitHub/rosdistro/foxy
4. rosdistro_build_cache file:///home/axcend/Documents/GitHub/rosdistro/index-v4.yaml foxy

### Generate the Recipe

In build-server VM ->
0. In the same terminal as before
1. superflore-gen-oe-recipes --dry-run --ros-distro foxy --only axcend_focus_custom_interfaces axcend_focus_front_panel_button axcend_focus_launch axcend_focus_legacy_compatibility_layer axcend_focus_ros2_firmware_bridge --output-repository-path ~/Documents/GitHub/test-meta-ros


### Update Existing Recipe

1. rm -r /home/axcend/OSTL-k/layers/meta-ros/meta-ros2-foxy/generated-recipes/axcend-focus && cp -r /home/axcend/Documents/GitHub/test-meta-ros/meta-ros2-foxy/generated-recipes/axcend-focus /home/axcend/OSTL-k/layers/meta-ros/meta-ros2-foxy/generated-recipes/axcend-focus
2. update bbappend version number

### Bake Changes

1. bitbake axcend-focus-custom-interfaces
3. bitbake axcend-focus-front-panel-button
4. bitbake axcend-focus-launch
5. bitbake axcend-focus-legacy-compatibility-layer
6. bitbake axcend-focus-ros2-firmware-bridge
7. bitbake axcend-focus-test-utils-package

### Git repo
1. Change git repo back to private

### Copy Changes to Board

1. Update version number if following command
2. scp /home/axcend/OSTL-k/build-openstlinuxweston-stm32mp1/tmp-glibc/deploy/deb/cortexa7t2hf-neon-vfpv4/axcend-focus-custom-interfaces_3.0.8-1-r0.0_armhf.deb root@192.168.1.188:/tmp

### Install Changes

1. Update version number in following command
2. dpkg -i /tmp/axcend-focus-custom-interfaces_3.0.8-1-r0.0_armhf.deb

## Transfer Bridge to Board

### Start

1. cd /mnt/c/Users/gupyf/Documents/GitHub/firmware_octavo/OSD32MP157C-512M-BAA_MinimalConfig/CA7/Bridge
2. scp *.tcl root@192.168.1.188:/axcend/bridge/

## Create a new package

cd /mnt/c/Users/gupyf/Documents/GitHub/ros2_ws/src/axcend_focus
Examples:
the axcend_focus prefix is for package organization on superflore

ros2 pkg create --build-type ament_cmake --node-name firmware_bridge axcend_focus_ros2_firmware_bridge_cpp
ros2 pkg create --build-type ament_python --node-name front_panel_button_controller axcend_focus_front_panel_button
ros2 pkg create --build-type ament_python --node-name legacy_compatibility_interface axcend_focus_legacy_compatibility_layer
ros2 pkg create --build-type ament_python axcend_focus_launch

ros2 pkg create --build-type ament_cmake --node-name my_node my_package


## Set the URL for superflore build

git remote set-url origin https://github.com/grahas/axcend_focus

Set for gitlab
git remote set-url origin https://gitlab.com/axcend/v3-hw-and-sw/axcend_focus

# Map network drive to board

\\sshfs\root@192.168.7.1\..\..
password is root

rsync -avz --exclude '.git/' /mnt/c/Users/gupyf/Documents/GitHub/ros2_ws/src/axcend_focus/axcend_focus_ros2_firmware_bridge root@192.168.7.1:/axcend/axcend_focus_ros2_firmware_bridge/

From WSL ros2 environment
colcon test --packages-select axcend_focus_legacy_compatibility_layer --output-on-failure

# Install package dependencies

After adding them to the package.xml file
In WSL ROS2 environment
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -y

# Test a specific command

test the legacy_compatibility_layer: 
colcon test --packages-select axcend_focus_legacy_compatibility_layer --event-handlers console_direct+

test the packet transcoder: 
colcon test --packages-select axcend_focus_ros2_firmware_bridge --event-handlers console_direct+

To test without rebuilding in ROS2 you can  use
colcon build --symlink-install

# Build a specific package
colcon build --packages-select axcend_focus_ros2_firmware_bridge 
colcon build --packages-select axcend_focus_custom_interfaces 
colcon build --packages-select axcend_focus_launch
colcon build --packages-select axcend_focus_test_utils --symlink-install
colcon build --symlink-install

# Launch the nodes
export ENVIRONMENT=development
export ENVIRONMENT=production
ros2 launch axcend_focus_launch application_launch.py

# Openinging the workspace
open from the axcend_focus directory that has all the packages as the root of the workspace.
cd /mnt/c/Users/gupyf/Documents/GitHub/ros2_ws
source install/setup.bash
cd /mnt/c/Users/gupyf/Documents/GitHub/ros2_ws/src/axcend_focus
code .
This will make all the paths work