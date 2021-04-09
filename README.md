# autonomous_vehicle ros1&ros2

# Working flow

1. Open three terminal nameing NO1, NO2, NO3

2. In the terminal NO1, run：
   cd autonomous_vehicle/ros1
   source ~/ros.sh
   catkin_make
   source devel/setup.bash

3. In the terminal NO2, run:
   cd autonomous_vehicle/ros2
   source ~/ros2.sh
   colcon build --symlink-install --packages-skip ros1_bridge
   source install/setup.bash

4. In the terminal NO3, run:
   cd autonomous_vehicle/ros2
   source ~/ros12.sh
   colcon build --packages-select ros1_bridge --cmake-force-configure
   source install/setup.bash

5. In the terminal NO1, run:
   roslaunch autonomous_vehicle lets_go.launch
   
6. IN the terminal NO3, run:
   ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
   
   
