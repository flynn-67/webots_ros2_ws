터미널 1
#~/webots_ros2_ws
source install/local_setup.bash
#webot, rviz 실행 
ros2 launch webots_ros2_turtlebot robot_launch.py nav:=true

터미널 2
#~/webots_ros2_ws
source install/local_setup.bash
#사진찍는 카메라 실행
ros2 run turtlebot3_camera_service image_service_server

터미널 3
#~/webots_ros2_ws/src/py_bt_ros
source ~/webots_ros2_ws/install/local_setup.bash
python3 main.py 
<img width="742" height="765" alt="image" src="https://github.com/user-attachments/assets/6bdf1033-4383-4cee-b52b-415c4ed98245" />
