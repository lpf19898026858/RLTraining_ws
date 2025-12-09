#!/usr/bin/env python3
import rospy
import numpy as np
import threading
import time
import signal
import sys
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
from gymnasium import Env, spaces
from rl_bridge.msg import RLObservation, RLAction, RLReward, RLDone, RLReset
from stable_baselines3.common.monitor import Monitor
from icm import ICM  # === 新增 ===

# ============================================================
#   基础 ROS → Unity 环境封装
# ============================================================
class UnityEnv(Env):
    def __init__(self):
        super(UnityEnv, self).__init__()

        if not rospy.core.is_initialized():
            rospy.init_node('ppo_trainer', anonymous=True, disable_signals=True)

        # --- ROS 通信接口 ---
        self.pub_action = rospy.Publisher('/rl/action', RLAction, queue_size=1)
        self.pub_reset = rospy.Publisher('/rl/reset', RLReset, queue_size=1)
        rospy.Subscriber('/rl/observation', RLObservation, self.obs_callback)
        rospy.Subscriber('/rl/reward', RLReward, self.reward_callback)
        rospy.Subscriber('/rl/done', RLDone, self.done_callback)

        # --- Gym 环境接口 ---
        self.observation_space = spaces.Box(low=-10, high=10, shape=(4,), dtype=np.float32)
        self.action_space = spaces.Box(low=-5, high=5, shape=(2,), dtype=np.float32)

        # --- 状态变量 ---
        self.obs = np.zeros(4, dtype=np.float32)
        self.reward = 0.0
        self.done = False

        # --- 同步事件 ---
        self.new_obs_event = threading.Event()
        self.new_obs_event.clear()

        # 通信节流（10Hz）
        self.last_publish_time = time.time()
        self.publish_interval = 0.1  # 秒

        device = "cuda" if torch.cuda.is_available() else "cpu"
        self.icm = ICM(obs_dim=4, act_dim=2, lr=1e-3, device=device)
        self.icm_coef_start, self.icm_coef_end = 0.02, 0.001
        self.total_steps, self.step_count = 8_000_000, 0
        self.device = device

        # ## PATCH: 追踪“最近一次观测的时间戳”，用于卡死检测
        self.last_obs_stamp = time.time()

    # ========== ROS 回调 ==========
    def obs_callback(self, msg):
        self.obs = np.array([msg.x, msg.y, msg.vx, msg.vy], dtype=np.float32)
        self.last_obs_stamp = time.time()      # ## PATCH
        self.new_obs_event.set()

    def reward_callback(self, msg):
        self.reward = msg.reward

    def done_callback(self, msg):
        self.done = msg.done

    # ========== Gym 接口 ==========
    def step(self, action):
        now = time.time()
        if now - self.last_publish_time < self.publish_interval:
            time.sleep(self.publish_interval - (now - self.last_publish_time))
        self.last_publish_time = time.time()

        # 发动作前，清理“等待新观测”事件
        self.new_obs_event.clear()
        self.pub_action.publish(RLAction(ax=float(action[0]), ay=float(action[1])))

        # 等待 Unity 回传“下一时刻观测”
        # ## PATCH: 加强健壮性——两级超时：
        #  - 短超时：仅警告（允许网络抖动）
        #  - 长超时：触发软 reset（防卡死）
        got_obs = self.new_obs_event.wait(timeout=1.0)
        if not got_obs:
            rospy.logwarn("Timeout waiting for new observation from Unity (1.0s).")
            if time.time() - self.last_obs_stamp > 5.0:
                rospy.logerr("No observation for >5s; soft reset environment.")
                self._soft_reset()            # 软重置（见下）

        # ICM 内在奖励 ===
        obs_prev = self.obs.copy()
        obs_next = np.array([msg for msg in self.obs], dtype=np.float32)
        s = torch.tensor(obs_prev, dtype=torch.float32, device=self.device).unsqueeze(0)
        s_next = torch.tensor(obs_next, dtype=torch.float32, device=self.device).unsqueeze(0)
        a = torch.tensor(action, dtype=torch.float32, device=self.device).unsqueeze(0)

        with torch.no_grad():
            a_hat, phi_ns, phi_ns_hat = self.icm.forward(s, s_next, a)
            reward_int = ((phi_ns_hat - phi_ns) ** 2).mean().item()

        # ICM 系数衰减
        progress = min(1.0, self.step_count / self.total_steps)
        coef = self.icm_coef_start + (self.icm_coef_end - self.icm_coef_start) * progress
        self.step_count += 1

        # ## PATCH: 把这一步的 reward 拿走后立即清零，避免跨步“黏连”
        reward_ext = float(self.reward)
        self.reward = 0.0
        
        total_reward = self.reward + coef * reward_int

        return self.obs, total_reward, self.done, False, {"reward_ext": self.reward, "reward_int": reward_int}

    def reset(self, *, seed=None, options=None):
        # 训练重置的唯一入口：由 SB3 驱动到达这里，再由 ROS 发给 Unity
        self.done = False
        self.reward = 0.0
        self.new_obs_event.clear()

        self.pub_reset.publish(RLReset(reset=True))  # <- 唯一 Reset 出口（ROS 端）
        rospy.sleep(0.05)  # 给通信少量时间

        if not self.new_obs_event.wait(timeout=2.0):
            rospy.logwarn("Timeout waiting for initial observation after reset; soft reset again.")
            self._soft_reset()

        return self.obs, {}
        
    # ## PATCH: 软 reset 封装（超时兜底）
    def _soft_reset(self):
        self.done = False
        self.reward = 0.0
        self.new_obs_event.clear()
        self.pub_reset.publish(RLReset(reset=True))
        if not self.new_obs_event.wait(timeout=2.0):
            rospy.logerr("Soft reset also timed out — please check Unity/ROS bridge.")

# ============================================================
#   动态衰减 Callback（学习率 + 熵系数 + ICM）
# ============================================================
class HyperDecayCallback:
    def __init__(self, model, total_steps):
        self.model = model
        self.total_steps = total_steps
        self.lr_start, self.lr_end = 3e-4, 1e-5
        self.ent_start, self.ent_end = 1e-2, 1e-4
        self.icm_start, self.icm_end = 0.02, 0.001
        self.last_log = 0

    def update(self, step):
        progress = min(1.0, step / self.total_steps)
        new_lr = self.lr_start + (self.lr_end - self.lr_start) * progress
        new_ent = self.ent_start + (self.ent_end - self.ent_start) * progress
        new_icm = self.icm_start + (self.icm_end - self.icm_start) * progress

        # 写回 PPO 优化器
        for pg in self.model.policy.optimizer.param_groups:
            pg["lr"] = new_lr
        self.model.ent_coef = new_ent

        # 写回 ICM 系数
        env = self.model.env.envs[0].env
        env.icm_coef_start = new_icm

        if step - self.last_log >= 50_000:
            rospy.loginfo(f"[Decay] step={step:,} | lr={new_lr:.6f} | ent={new_ent:.6f} | icm={new_icm:.6f}")
            self.last_log = step


# ============================================================
#   主函数
# ============================================================
if __name__ == '__main__':
    rospy.loginfo("Starting PPO Trainer (Optimized Unity-ROS loop + ICM)")

    try:
        # ✅ 环境封装与归一化
        env = DummyVecEnv([lambda: Monitor(UnityEnv())])
        env = VecNormalize(env, norm_obs=True, norm_reward=True, clip_obs=10.)

        # ✅ PPO 配置
        model = PPO(
            "MlpPolicy",
            env,
            verbose=1,
            device="cuda",
            learning_rate=3e-4,
            n_steps=2048,
            batch_size=256,
            gamma=0.99,
            gae_lambda=0.95,
            ent_coef=0.01,
            clip_range=0.2,
            tensorboard_log="/home/lpf/docker_shared/rltraining_ws/src/rl_bridge/logs/"
        )

        rospy.loginfo("PPO Training started...")
        rospy.sleep(1.0)
        env.training = True
        env.norm_reward = False

        decay = HyperDecayCallback(model, total_steps=10_000_000)

        # === 新增 Ctrl+C 处理 ===
        def handle_interrupt(sig, frame):
            rospy.logwarn("\nCtrl+C pressed — saving model before exit...")
            model.save("/home/lpf/docker_shared/rltraining_ws/src/rl_bridge/config/ppo_ball_interrupt")
            env.save("/home/lpf/docker_shared/rltraining_ws/src/rl_bridge/config/vec_normalize_interrupt.pkl")
            rospy.signal_shutdown("User interrupt")
            sys.exit(0)
        signal.signal(signal.SIGINT, handle_interrupt)

        # === 训练主循环 ===
        total_steps = 1000_000
        for step_block in range(0, total_steps, 2048):
            model.learn(total_timesteps=2048, reset_num_timesteps=False)
            decay.update(step_block)

        # === 保存结果 ===
        model.save("/home/lpf/docker_shared/rltraining_ws/src/rl_bridge/config/ppo_ball_sync")
        env.save("/home/lpf/docker_shared/rltraining_ws/src/rl_bridge/config/vec_normalize.pkl")
        rospy.loginfo("PPO training finished and normalization saved.")

    except KeyboardInterrupt:
        rospy.logwarn("\nCtrl+C pressed — saving current model before exit...")
        model.save("/home/lpf/docker_shared/rltraining_ws/src/rl_bridge/config/ppo_ball_interrupt")
        env.save("/home/lpf/docker_shared/rltraining_ws/src/rl_bridge/config/vec_normalize_interrupt.pkl")

    finally:
        rospy.signal_shutdown("Training completed or interrupted")
        print("ROS node terminated cleanly.")

