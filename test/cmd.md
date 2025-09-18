## Test moveit2 services

Clone the piper_ros repository (if you have already cloned it, you can skip this step):

```bash
cd src
git clone -b humble https://github.com/agilexrobotics/piper_ros.git
cd ../
colcon build
source install/setup.bash
```

Start the moveit2 node:

```bash
ros2 launch piper_no_gripper_moveit demo.launch.py
```

To test the moveit2 services, we can use the following commands:

```bash
ros2 service call /compute_fk moveit_msgs/srv/GetPositionFK "$(cat test/fk.yaml)"
```

```bash
ros2 service call /compute_ik moveit_msgs/srv/GetPositionIK "$(cat test/ik.yaml)"
```

```bash
ros2 service call /compute_cartesian_path moveit_msgs/srv/GetCartesianPath "$(cat test/traj.yaml)"
```