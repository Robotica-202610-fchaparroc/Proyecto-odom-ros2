export MENTORPI_HOME=$PWD
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro simple.urdf.xacro)"
