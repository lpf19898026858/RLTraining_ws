# [修改] 在消息定义中添加常量
# Drone Status Constants
uint8 UNKNOWN=0
uint8 ACTIVE=1
uint8 NEEDS_BACKUP=2
uint8 OFFLINE=3

# Message Fields
string drone_id
uint8 status
int32 last_completed_task_index
geometry_msgs/Pose last_known_pose
