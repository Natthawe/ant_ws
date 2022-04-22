<license>Apache License 2.0</license>

<!-- colcon -->
sudo apt install python3-colcon-common-extensions

<!-- create pkg python  -->
ros2 pkg create --build-type ament_cmake <you_pkg_name>

<!-- create pkg python  -->
ros2 pkg create --build-type ament_python <you_pkg_name> --dependencies rclpy <pkg_name>

ros2 pkg create obstacle_avoidance --build-type ament_cmake --dependencies rclcpp

<!-- find pkg install in directory src -->
rosdep install -i --from-path src --rosdistro galactic -y

<!-- build select pkg -->
colcon build --packages-select <pkg_name>

<!-- build all pkg -->
colcon build
colcon build --symlink-install

<!-- install -->
. install/setup.bash
source install/setup.bash

<!-- check ros2 CMD -->
ros2 doctor

rosdep install -y -r -q --from-paths src --ignore-src --rosdistro <ros2-distro>

<!-- ROSDEP -->

sudo apt-get install python3-rosdep -y
sudo rosdep init
rosdep update
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
rosdep install --from-paths src --ignore-src -r -y

<!-- solve rplidar -->
sudo chmod +x /dev/ttyUSB0
sudo chmod 777 /dev/ttyUSB0
sudo adduser [$USER] dialout