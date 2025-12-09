import numpy as np
import math

# ===============================
# 参数表（来自 Unity Inspector）
# ===============================
PARAMS = {
    # ---- Terminal ----
    "arrival_reward": 12.0,
    "landing_bonus": 2.4,
    "failure_penalty": -12.0,

    # ---- Shaping ----
    "potential_k": 1.9,
    "align_k": 0.02,
    "speed_k": 0.03,
    "dir_k": 0.006,
    "evasive_k": 0.17,

    "approach_zone_ratio": 0.8,
    "terminal_zone_ratio": 0.4,

    # ---- Penalties ----
    "time_penalty": -0.002,
    "excessive_ang_vel_penalty": -0.01,
    "max_allowed_ang_vel": 20.0,
    "stagnation_penalty": -0.01,
    "obstacle_stagnation_penalty": -0.09,
    "action_change_penalty": -0.0006,
    "energy_penalty": -0.0003,
    "obstacle_proximity_penalty": -0.03,
    "min_safe_obstacle_distance": 1.5,
}
_prev_distance = None
def reset_reward_state():
    """在每个 episode 开始时调用，清空内部状态。"""
    global _prev_distance
    _prev_distance = None
    
def compute_reward(event_type, data):
    """
    data = [distance, speed, alignment, altitude, ang_vel, prox_dist, vel_diff, obstacle_ahead, step_time]
    其中：
      distance: 当前与目标距离
      speed: 当前速度
      alignment: 当前朝向对准度
      altitude: 当前高度
      ang_vel: 当前角速度 (deg/s)
      prox_dist: 最近障碍距离
      vel_diff: 当前速度变化幅度
      obstacle_ahead: 1 表示前方有障碍
      step_time: 每步时间
    """
    global _prev_distance
    p = PARAMS
    r_total = 0.0
    done = False

    # ==================================================
    # 1️⃣ Terminal Rewards
    # ==================================================
    if event_type == "reached_target":
        # Ra = ra + rb * exp(-v)
        v = max(0.0, min(5.0, data[1]))  # 当前速度
        r_arrival = p["arrival_reward"] + p["landing_bonus"] * math.exp(-v)
        r_total += r_arrival
        done = True

    elif event_type == "collision":
        r_total += p["failure_penalty"]
        done = True

    elif event_type == "out_of_bounds":
        r_total += p["failure_penalty"]
        done = True
    
    elif event_type == "timeout":
        distance = float(data[0])
        r_total += -10.0 if distance > 3.0 else -5.0
        #r_total=0.0
        done = True

    # ==================================================
    # 2️⃣ Process-Guiding Shaping Rewards
    # ==================================================
    elif event_type == "move":
        distance = float(data[0])
        speed = float(data[1])
        alignment = float(data[2])
        ang_vel = float(data[4])
        prox_dist = float(data[5]) if len(data) > 5 else float("inf")
        vel_diff = float(data[6]) if len(data) > 6 else 0.0
        obstacle_ahead = int(data[7]) if len(data) > 7 else 0

        # (1) Potential: 距离缩短奖励
        if _prev_distance is None:
            delta_d = 0.0
        else:
            delta_d = _prev_distance - distance
        _prev_distance = distance
        r_potential = p["potential_k"] * delta_d

        # (2) Align: 对准奖励
        r_align = p["align_k"] * (alignment ** 2) if alignment > 0 else 0.0

        # (3) Speed: 速度控制奖励
        v_ideal = 2.0
        r_speed = p["speed_k"] * math.exp(-abs(speed - v_ideal))

        # (4) Dir: 飞行方向奖励
        r_dir = p["dir_k"] * alignment if alignment > 0 else 0.0

        # (5) Evasive: 避障奖励
        r_evasive = p["evasive_k"] if obstacle_ahead == 1 and abs(speed) > 0.3 else 0.0

        r_total += r_potential + r_align + r_speed + r_dir + r_evasive

        # ==================================================
        # 3️⃣ Flight Quality Optimization Rewards / Penalties
        # ==================================================
        ang_vel = data[4]
        vel_diff = data[6]
        d_obs = prox_dist
        d_safe = p["min_safe_obstacle_distance"]

        # (1) 时间惩罚
        r_total += p["time_penalty"]

        # (2) 姿态角速度惩罚
        if ang_vel > p["max_allowed_ang_vel"]:
            r_total += p["excessive_ang_vel_penalty"]

        # (3) 障碍接近惩罚
        if d_obs < d_safe:
            prox_factor = ((d_safe - d_obs) / d_safe) ** 2
            r_total += p["obstacle_proximity_penalty"] * prox_factor

        # (4) 动作变化惩罚（平滑性）
        r_total += p["action_change_penalty"] * abs(vel_diff)

        # (5) 能耗惩罚
        r_total += p["energy_penalty"] * (speed ** 2)

        # (6) 停滞惩罚
        if speed < 0.1:
            r_total += p["stagnation_penalty"]

        # (7) 障碍前停滞惩罚
        if obstacle_ahead == 1 and speed < 0.1:
            r_total += p["obstacle_stagnation_penalty"]

    else:
        pass
        
    if done:
        _prev_distance = None

    return float(r_total), done

