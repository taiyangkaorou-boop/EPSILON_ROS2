# float64 行主序占用栅格；data 期望长度为 width*height。
std_msgs/Header header

# 地图宽度 [cell]。
uint32 width

# 地图高度 [cell]。
uint32 height

# 分辨率 [m/cell]。
float32 resolution

# cell(0,0) 在世界坐标中的原点位姿。
geometry_msgs/Pose origin

# 从 (0,0) 开始按行存储；占用概率 [0,100]，unknown=-1。
float64[] data
