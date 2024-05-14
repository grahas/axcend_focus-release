# Repository for ROS2 and Axcend Focus LC

## Workflow for building ros2 package and getting them on the board

## Setup

Open the Ubuntu WSL VM and build server VM

### Clean workspace

1. cd /mnt/c/Users/gupyf/Documents/GitHub/ros2_ws
2. rm -r build install log

### Build package

1. colcon build --packages-select axcend_focus_custom_interfaces

### Update Changelog

catkin_generate_changelog

catkin_prepare_release

bloom-release --rosdistro foxy axcend_focus

https://github.com/grahas/axcend_focus-release.git

https://github.com/grahas/axcend_focus.git

### Rebuild package index

In build-server VM ->

1. export ROSDISTRO_INDEX_URL=file:///home/axcend/Documents/GitHub/rosdistro/index-v4.yaml
2. update the version number in distribution.yaml
3. cd ~/Documents/GitHub/rosdistro/foxy
4. rm foxy-cache.yaml && rm foxy-cache.yaml.gz
5. rosdistro_build_cache file:///home/axcend/Documents/GitHub/rosdistro/index-v4.yaml foxy

### Generate the recipe

In build-server VM ->

1. superflore-gen-oe-recipes --dry-run --ros-distro foxy --only axcend_focus_custom_interfaces --output-repository-path ~/Documents/GitHub/test-meta-ros

### Update Existing Recipe

1. rm -r /home/axcend/OSTL-k/layers/meta-ros/meta-ros2-foxy/generated-recipes/axcend-focus && cp -r /home/axcend/Documents/GitHub/test-meta-ros/meta-ros2-foxy/generated-recipes/axcend-focus /home/axcend/OSTL-k/layers/meta-ros/meta-ros2-foxy/generated-recipes/axcend-focus
2. update bbappend version number

### Bake changes

bitbake axcend-focus-custom-interfaces

### Copy Changes to Board

scp /home/axcend/OSTL-k/build-openstlinuxweston-stm32mp1/tmp-glibc/deploy/deb/cortexa7t2hf-neon-vfpv4/axcend-focus-custom-interfaces_3.0.7-1-r0.0_armhf.deb root@192.168.1.188:/tmp

### Install Changes

dpkg -i /tmp/axcend-focus-custom-interfaces_3.0.7-1-r0.0_armhf.deb
