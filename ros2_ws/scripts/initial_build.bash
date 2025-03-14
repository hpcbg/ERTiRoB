sudo rm -rf install
sudo rm -rf build
sudo rm -rf log
sudo rm -rf src/micro_ros_setup
source /opt/ros/jazzy/setup.bash
sudo apt install ros-jazzy-rosbridge-suite
sudo apt install ros-jazzy-urdf-launch
mkdir src/micro_ros_setup
cd src/micro_ros_setup
git clone -b jazzy https://github.com/micro-ROS/micro_ros_setup.git
cd ../..
rosdep update && rosdep install --from-paths src --ignore-src -y
colcon build
source install/local_setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.sh
. ./scripts/build.bash