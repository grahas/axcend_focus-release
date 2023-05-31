# Repository for ROS2 and Axcend Focus LC


Clean workspace


```
rm -r build install log
```

Build package

colcon build --packages-select axcend_focus_msgs

catkin_generate_changelog

catkin_prepare_release

bloom-release --rosdistro foxy axcend_focus

https://github.com/grahas/axcend_focus-release.git

https://github.com/grahas/axcend_focus.git
