# RLEvent.msg

string event_type         # 事件类型，例如 "move", "reached_target", "collision", "out_of_bounds"
float32[] data            # 附带数据，例如 [distance, velocity, angle]
bool done                 # 是否终止当前 episode

