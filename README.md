# ROS2 personal repository

It is a collection of packages for different robots (mobile and manipulators)

I will use this README to add notes and useful commands for ROS:

## For URDF
$ ros2 launch urdf_tutorial display.launch.py model:=($PATH TO MODEL)
$ ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro )"

## For the new gazebo
- Launch a new gazebo instance in an empty world
    ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="empty.sdf -r"
- Spawn a new robot inside the simulator:
    ros2 run ros_gz_sim create -topic /robot_description
- For bridging topics
    ros2 run ros_gz_bridge paramater_bridge --ros-args -p config_file:="PATH_TO_CONFIG.yaml"

## Fix BUG libgeometric_shapes.so.2.1.3 not found for moveit
- (Optional) Install plocate
    * sudo apt-get install plocate
    * (Optional) Update locate db
    * sudo updatedb
    * locate libgeometric_shapes.so
- Change directory to where libgeometric_shapes.so was found
    * cd /opt/humble/lib/
- Create a symlink
    * sudo ln -s libgeometric_shapes.so.2.3.x libgeometric_shapes.so.2.1.3
- Confirm the creation of the symlink
    * ls -l | grep libgeometric_shapes