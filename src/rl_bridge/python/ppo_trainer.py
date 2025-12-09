# ppo_trainer.py
import os
import rospy
import gymnasium as gym
import numpy as np
import torch
import signal
import sys
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback
from stable_baselines3.common.policies import ActorCriticPolicy
from stable_baselines3.common.monitor import Monitor   
from rl_env import DroneRLEnv
from icm import ICM
from torch.utils.tensorboard import SummaryWriter
from curriculum_manager import CurriculumManager  
from stable_baselines3.common.vec_env import VecNormalize
import time
import re

# -------- Schedules --------
def make_linear_schedule(start, end):
    def f(progress_remaining: float):  # 1 -> 0
        return start + (end - start) * (1 - progress_remaining)
    return f

# -------- Curiosity Wrapper --------
class CuriosityEnvWrapper(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, base_env: DroneRLEnv,
                 icm_coef_start=0.02, icm_coef_end=0.001,
                 total_steps=8_000_000, device="cpu"):
        super().__init__()
        self.env = base_env
        self.observation_space = gym.spaces.Box(
            low=-np.inf, high=np.inf, shape=(self.env.obs_dim,), dtype=np.float32
        )
        self.action_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(self.env.action_dim,), dtype=np.float32
        )
        self.device = device
        self.bins = 5
        self.bin_edges = np.linspace(-1.0, 1.0, self.bins + 1)
        self.icm = ICM(obs_dim=self.env.obs_dim,
                       act_dim=self._discretize_dim(),
                       feature_dim=128, lr=1e-3, device=device)
        self.icm_coef_sched = make_linear_schedule(icm_coef_start, icm_coef_end)
        self.total_steps = total_steps
        self._global_step = 0
        self._last_obs = None

    def _discretize_dim(self):
        return self.bins ** 4

    def _action_to_index(self, a: np.ndarray) -> int:
        idxs = []
        for i in range(4):
            bin_id = np.digitize(a[i], self.bin_edges) - 1
            bin_id = int(np.clip(bin_id, 0, self.bins - 1))
            idxs.append(bin_id)
        return idxs[0]*(self.bins**3) + idxs[1]*(self.bins**2) + idxs[2]*self.bins + idxs[3]

    def reset(self, *, seed=None, options=None):
        if seed is not None:
            np.random.seed(seed)
        obs = self.env.reset()
        self._last_obs = obs.copy()
        return obs, {}

    def step(self, action):
        obs_next, r_ext, done, info = self.env.step(action)
        a_idx = self._action_to_index(action)
        s = torch.tensor(self._last_obs, dtype=torch.float32, device=self.device).unsqueeze(0)
        s_next = torch.tensor(obs_next, dtype=torch.float32, device=self.device).unsqueeze(0)
        a_idx_t = torch.tensor([a_idx], dtype=torch.long, device=self.device)
        _, _, _, intrinsic_vec = self.icm.update(
            s, s_next, a_idx_t, act_dim=self._discretize_dim(), beta=0.2
        )
        r_int = float(intrinsic_vec.mean().item())
        #r = r_ext + coef * r_int
        progress_remaining = max(1e-6, 1.0 - self._global_step / float(self.total_steps))
        coef = self.icm_coef_sched(progress_remaining)
        r = r_ext + coef * r_int
        #self.logger.record("train/intrinsic_reward_mean", r_int)        
        self._global_step += 1
        self._last_obs = obs_next.copy()
        terminated = bool(done)
        truncated = False
        return obs_next, r, terminated, truncated, info

# -------- 动态衰减 --------
class HyperDecayCallback(BaseCallback):
    """
    全局线性衰减版本：
    在整个训练过程中，学习率、熵系数(entropy coef)、
    以及好奇心系数(curiosity coef)随训练进度线性下降。
    """

    def __init__(self,
                 ent_start=1e-4, ent_end=1e-5,
                 lr_start=3e-4, lr_end=1e-5,
                 curiosity_start=0.02, curiosity_end=0.001,
                 total_steps=8_000_000):
        super().__init__()
        self.ent_start = ent_start
        self.ent_end = ent_end
        self.lr_start = lr_start
        self.lr_end = lr_end
        self.curiosity_start = curiosity_start
        self.curiosity_end = curiosity_end
        self.total_steps = total_steps

        self.ent_current = ent_start
        self.lr_current = lr_start
        self.curiosity_current = curiosity_start

    def _on_step(self) -> bool:
        # 当前全局进度
        progress = min(1.0, self.num_timesteps / self.total_steps)
        progress_remaining = 1.0 - progress  # SB3 习惯用“剩余进度”

        # === 学习率线性衰减 ===
        self.lr_current = self.lr_start + (self.lr_end - self.lr_start) * progress
        # 从 SB3 的调度器读取（保持一致性）
        if hasattr(self.model, "lr_schedule"):
            lr_scheduled = float(self.model.lr_schedule(progress_remaining))
            self.lr_current = lr_scheduled

        # === 熵系数衰减 ===
        self.ent_current = self.ent_start + (self.ent_end - self.ent_start) * progress
        self.model.ent_coef = float(self.ent_current)

        # === 好奇心系数衰减 ===
        self.curiosity_current = self.curiosity_start + (self.curiosity_end - self.curiosity_start) * progress
        try:
            env0 = self.model.env.envs[0]
            if hasattr(env0, "env") and hasattr(env0.env, "icm"):
                env0.env.icm.coef = float(self.curiosity_current)
        except Exception:
            pass

        # === TensorBoard 记录 ===
        self.logger.record("train/learning_rate", self.lr_current)
        self.logger.record("train/entropy_coef", self.ent_current)
        self.logger.record("train/curiosity_coef", self.curiosity_current)
        self.logger.record("train/global_progress", progress)

        # === 控制台打印 ===
        if self.num_timesteps % 50_000 == 0:
            rospy.loginfo(
                f"[GlobalDecay] step={self.num_timesteps:,} "
                f"| progress={progress:.3f} "
                f"| LR={self.lr_current:.6f} | ENT={self.ent_current:.2e} | CUR={self.curiosity_current:.4f}"
            )

        return True

class RewardLoggingCallback(BaseCallback):
    def __init__(self):
        super().__init__()
    def _on_step(self) -> bool:
        if "rewards" in self.locals:
            mean_r = np.mean(self.locals["rewards"])
            self.logger.record("custom/reward_mean", mean_r)
        return True
        
class TrainingMetricsCallback(BaseCallback):
    """
    直接从环境内部读取课程状态 (DroneRLEnv.cx/cz/cl)，并记录到 TensorBoard。
    """
    def __init__(self, log_interval: int = 100):
        super().__init__()
        self.log_interval = log_interval

    def _unwrap_env(self, env):
        """一直向下取 .env，直到最底层（拿到 CuriosityEnvWrapper 或 DroneRLEnv）。"""
        base = env
        # 兼容 DummyVecEnv -> Monitor -> CuriosityEnvWrapper -> DroneRLEnv
        # 有些封装没有 .env 属性，用 while 安全剥壳
        max_depth = 10
        depth = 0
        while hasattr(base, "env") and depth < max_depth:
            base = base.env
            depth += 1
        return base

    def _on_step(self) -> bool:
        # （可选）降低记录频率，避免太密
        if self.num_timesteps % self.log_interval != 0:
            return True

        # --- ICM 内在奖励（若存在）---
        try:
            icm = self.model.env.envs[0].env.icm  # Monitor.env -> CuriosityEnvWrapper.icm
            if hasattr(icm, "icm_loss"):
                self.logger.record("train/curiosity_reward_mean", float(icm.icm_loss))
        except Exception:
            pass

        # --- 取最底层 env，然后拿 curriculum 状态 ---
        try:
            env0 = self.model.env.envs[0]      # DummyVecEnv slot
            base = self._unwrap_env(env0)      # 应该是 CuriosityEnvWrapper 或 DroneRLEnv

            if hasattr(base, "env"):
                base = base.env  # 拿到 DroneRLEnv

            if hasattr(base, "get_curriculum_status"):
                status = base.get_curriculum_status()
                global_sr = status.get("global", {})

                # 1. 记录【总体】平均奖励
                reward_overall = global_sr.get("episode_reward_mean")
                if reward_overall is not None and not np.isnan(reward_overall):
                    self.logger.record("custom/episode_reward_mean", reward_overall)

                # 2. 记录【滑动窗口】平均奖励
                reward_window = global_sr.get("episode_reward_mean_window")
                if reward_window is not None and not np.isnan(reward_window):
                    self.logger.record("custom/episode_reward_mean_window", reward_window)
                    
                # 记录总平均
                #self.logger.record("custom/episode_reward_mean", global_sr.get("episode_reward_mean", 0.0))
                self.logger.record("custom/window_success_rate", global_sr.get("window_success_rate", 0.0))
                self.logger.record("custom/total_success_rate",  global_sr.get("total_success_rate", 0.0))
                self.logger.record("curriculum/current_lesson_id",   status["x"]["lesson_id"])
                self.logger.record("curriculum/current_x",       status["x"]["current_value"])
                self.logger.record("curriculum/current_z",       status["z"]["current_value"])
                self.logger.record("curriculum/local_target_range",       status["local"]["current_value"])
                
        except Exception:
            # 不让训练因为日志失败而中断
            pass

        return True

# -------- Env 工厂 --------
def make_env(yaml_path: str, device="cpu"):
    base = DroneRLEnv(yaml_path=yaml_path)
    wrapped = CuriosityEnvWrapper(
        base_env=base,
        icm_coef_start=0.02, icm_coef_end=0.001,
        total_steps=8_000_000, device=device
    )
    return wrapped
    
def get_run_paths(base_checkpoint_dir="checkpoints", base_tb_dir="tb", run_name_prefix="PPO"):
    """
    为新的训练运行创建并返回唯一的路径。
    该函数会查找从 1 开始的第一个未被使用的整数编号。
    例如，如果 PPO_1 和 PPO_3 存在，则会创建 PPO_2。
    """
    # 确保基础目录存在
    os.makedirs(base_checkpoint_dir, exist_ok=True)
    os.makedirs(base_tb_dir, exist_ok=True)

    # --- 核心逻辑变更 ---
    # 旧逻辑：查找最大编号 + 1
    # 新逻辑：从 1 开始查找第一个未使用的编号
    new_run_num = 1
    while True:
        # 构造当前要检查的文件夹名
        potential_run_name = f"{run_name_prefix}_{new_run_num}"
        # 构造其完整路径
        potential_checkpoint_path = os.path.join(base_checkpoint_dir, potential_run_name)
        
        # 检查该路径是否已存在
        if not os.path.exists(potential_checkpoint_path):
            # 如果不存在，说明我们找到了可以使用的编号，跳出循环
            run_name = potential_run_name
            break
        
        # 如果存在，则尝试下一个编号
        new_run_num += 1
    
    # --- 逻辑变更结束 ---

    # 使用找到的 run_name 创建新的路径
    run_checkpoint_path = os.path.join(base_checkpoint_dir, run_name)
    run_tb_path = os.path.join(base_tb_dir, run_name)

    os.makedirs(run_checkpoint_path, exist_ok=True)
    os.makedirs(run_tb_path, exist_ok=True)

    print(f"本次训练将保存到: {run_checkpoint_path}")
    print(f"TensorBoard 日志将保存到: {run_tb_path}")

    return run_checkpoint_path, run_tb_path
    
# -------- 主函数 --------
def main():
    rospy.init_node("ppo_trainer")

    # 1. 动态生成本次训练的保存路径和日志路径
    save_path, tb_log_path = get_run_paths(run_name_prefix="PPO")

    yaml_path = "drone_config.yaml"
    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"Using device: {device}")

    # 2. 更新 Monitor 的日志文件路径
    env = DummyVecEnv([lambda: Monitor(make_env(yaml_path, device=device),
                                       filename=os.path.join(tb_log_path, "monitor.csv"))])
    env = VecNormalize(env, norm_obs=True, norm_reward=True, gamma=0.995,
                   clip_obs=10.0, clip_reward=10.0)

    total_steps = 1_500_000
    lr_sched = make_linear_schedule(3e-4, 1e-5)
    clip_sched = make_linear_schedule(0.10, 0.10)
    policy_kwargs = dict(net_arch=[512, 512, 512, 512], normalize_images=False)

    model = PPO(
        ActorCriticPolicy, env,
        n_steps=2048, batch_size=2048, n_epochs=3,
        gamma=0.995, gae_lambda=0.98,
        learning_rate=lr_sched, clip_range=clip_sched,
        ent_coef=1e-4, vf_coef=0.5, max_grad_norm=0.5,
        policy_kwargs=policy_kwargs, verbose=1,
        # 3. 更新 TensorBoard 的日志路径
        tensorboard_log=tb_log_path,
        device=device
    )

    # 4. 更新 CheckpointCallback 的保存路径
    ckpt_cb = CheckpointCallback(save_freq=50_000, save_path=save_path, name_prefix="drone_ppo")
    decay_cb = HyperDecayCallback(ent_start=1e-4, ent_end=1e-5,
                                  curiosity_start=0.02, curiosity_end=0.001,
                                  total_steps=total_steps)
    reward_cb = RewardLoggingCallback()
    metrics_cb = TrainingMetricsCallback()

    def handle_interrupt(sig, frame):
        print("\n KeyboardInterrupt detected! Gracefully stopping training...")
        # 5. 更新中断时模型的保存路径
        interrupt_save_file = os.path.join(save_path, "ppo_interrupt_last.zip")
        model.save(interrupt_save_file)
        print(f"Model saved to {interrupt_save_file}")
        rospy.signal_shutdown("User interrupt")
        sys.exit(0)

    signal.signal(signal.SIGINT, handle_interrupt)

    try:
        model.learn(total_timesteps=total_steps, callback=[ckpt_cb, decay_cb, reward_cb, metrics_cb])
    except KeyboardInterrupt:
        handle_interrupt(None, None)
    finally:
        print("Training finished. Saving final model...")
        # 6. 更新最终模型的保存路径
        final_model_file = os.path.join(save_path, "ppo_final.zip")
        model.save(final_model_file)
        print(f"Final model saved to {final_model_file}")
        rospy.signal_shutdown("Training complete")

    # 7. 更新 VecNormalize 统计数据的保存路径
    vecnorm_file = os.path.join(save_path, "vecnorm.pkl")
    env.save(vecnorm_file)
    print(f"VecNormalize stats saved to {vecnorm_file}")


if __name__ == "__main__":
    main()

