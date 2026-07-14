# 车辆纵向加速度/转角速度控制和可选开环期望状态。
std_msgs/Header header
float64 acc
float64 steer_rate

# true 时直接把 state 作为期望状态并绕过闭环控制。
std_msgs/Bool is_openloop
State state
