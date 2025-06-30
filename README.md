# ROS2 personal repository

It is a collection of packages for different robots (mobile and manipulators)

I will use this README to add notes and useful commands for ROS:

## For URDF
$ ros2 launch urdf_tutorial display.launch.py model:=($PATH TO MODEL)
$ ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro )"

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