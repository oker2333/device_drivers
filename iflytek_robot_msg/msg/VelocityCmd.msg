#ifly_robot_velocity
uint8 velocity_model #速度模式  60：机器人整体速度  61：机器人单轮速度  62：紧急刹车
geometry_msgs/Twist entirety_vel #设置机器人整体速度
float64 left_single_wheel_vel #设置机器人左轮速度
float64 right_single_wheel_vel #设置机器人右轮速度