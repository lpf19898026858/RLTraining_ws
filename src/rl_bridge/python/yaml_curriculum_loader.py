# yaml_curriculum_loader.py
import yaml

def _collect_stages(param_node):
    """
    param_node: environment_parameters[target_spawn_range_x] 或 ..._z 或 local_target_range
    结构： { curriculum: [ {name, completion_criteria:{threshold,min_lesson_length}, value}, ... ] }
    """
    result = []
    if not param_node: 
        return result
    cur = param_node.get("curriculum", [])
    for item in cur:
        name = item.get("name", "Stage")
        cc = item.get("completion_criteria") or {}
        threshold = float(cc.get("threshold", 0.0))
        min_len = int(cc.get("min_lesson_length", 1000))
        value = float(item.get("value", 5.0))
        result.append((name, threshold, min_len, value))
    return result

def load_three_axes_from_yaml(yaml_path: str):
    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f)

    envp = data.get("environment_parameters", {})
    x_list = _collect_stages(envp.get("target_spawn_range_x"))
    z_list = _collect_stages(envp.get("target_spawn_range_z"))
    l_list = _collect_stages(envp.get("local_target_range"))

    if not(x_list and z_list and l_list):
        raise ValueError("YAML 未解析到完整的 X/Z/Local 课程，请检查 environment_parameters 节点。")

    return x_list, z_list, l_list

