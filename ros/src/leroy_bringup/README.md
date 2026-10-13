# leroy_bringup

Top-level ROS 2 bringup package for Leroy.

## Launch

```bash
ros2 launch leroy_bringup robot.launch.py
```

Optional:

```bash
ros2 launch leroy_bringup robot.launch.py use_depthart:=false
ros2 launch leroy_bringup robot.launch.py video_device:=/dev/video2
```

This launch starts:
- `leroy_description` / robot_state_publisher
- `leroy_bridge`
- front USB camera
- image rectification
- DepthART through the `depthart_ros` package launch file

The existing `exec.sh` should remain responsible for entering/starting the container and
sourcing `/opt/ros/jazzy/setup.bash` and `/leroy_ws/install/setup.bash`. This package is
responsible for ROS graph bringup.
