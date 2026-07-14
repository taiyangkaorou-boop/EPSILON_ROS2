# 原始 Lane 拓扑、换道关系、行为标签和二维中心线。
std_msgs/Header header

int32 id
int32 dir

# 纵向后继/前驱 Lane ID。
int32[] child_id
int32[] father_id

# 左右相邻 Lane ID 及是否允许换入。
int32 l_lane_id
bool l_change_avbl

int32 r_lane_id
bool r_change_avbl

# 未强类型化的 Lane 行为字符串。
string behavior

float32 length

geometry_msgs/Point start_point
geometry_msgs/Point final_point

geometry_msgs/Point[] points
