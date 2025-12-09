# [修改] 在消息定义中添加常量
# Task Status Constants
uint8 PENDING=0
uint8 IN_PROGRESS=1
uint8 COMPLETED=2
uint8 NEEDS_BACKUP=3 # 注意：我把FAILED改成了NEEDS_BACKUP以匹配你的C++代码

# Message Fields
int32 task_id
nlp_drone_control/Action action_details
uint8 status
string assigned_drone_id
