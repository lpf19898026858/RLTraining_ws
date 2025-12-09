#!/usr/bin/env python3
import torch
from stable_baselines3 import PPO

# 1) 加载策略（CPU）
model_path = "/home/lpf/docker_shared/rltraining_ws/src/rl_bridge/python/checkpoints/ppo_interrupt_last.zip"
model = PPO.load(model_path, device="cpu")
policy = model.policy

# 2) 自动推断策略期望的特征维度（通常等于观测维度）
#    对于 MLP 策略，policy.mlp_extractor.policy_net[0] 是第一层 Linear
in_features = policy.mlp_extractor.policy_net[0].in_features  # 这里应为 22
print(f"[INFO] policy expects feature dim = {in_features}")

# 3) 构造 dummy 输入：(batch, in_features)
dummy_obs = torch.randn(1, in_features, dtype=torch.float32)

# 4) 包装一个仅做前向推理的模块，复用 SB3 的特征抽取与头部
class PPOPolicyWrapper(torch.nn.Module):
    def __init__(self, policy):
        super().__init__()
        self.policy = policy

    def forward(self, obs):
        # 与你原来的实现一致：特征 -> actor 分支 -> 动作头
        features = self.policy.extract_features(obs)
        latent_pi = self.policy.mlp_extractor.forward_actor(features)
        actions = self.policy.action_net(latent_pi)
        return actions

wrapped_policy = PPOPolicyWrapper(policy).eval()

# 5) 导出 ONNX
onnx_path = "/home/lpf/docker_shared/rltraining_ws/src/rl_bridge/python/checkpoints/ppo_trainer.onnx"
torch.onnx.export(
    wrapped_policy,
    dummy_obs,
    onnx_path,
    input_names=["obs"],
    output_names=["action"],
    dynamic_axes={"obs": {0: "batch"}, "action": {0: "batch"}},
    opset_version=13
)
print(f"✅ Exported PPO policy to {onnx_path} (CPU mode)")

