<debug 용 빌드 및 실행>
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug
ros2 launch factr_controller robot_arm_debug.launch.py 

TODO:
1. 튜닝
2. q_dot_dot = J^+ (x_d_dot_dot - J_dot q_dot) 이거 말고 q -> x 구하기

