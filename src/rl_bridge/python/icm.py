# icm.py
import torch
import torch.nn as nn
import torch.optim as optim

class MLP(nn.Module):
    def __init__(self, in_dim, out_dim, hidden=256, layers=2):
        super().__init__()
        mods = []
        d = in_dim
        for _ in range(layers):
            mods += [nn.Linear(d, hidden), nn.ReLU()]
            d = hidden
        mods += [nn.Linear(d, out_dim)]
        self.net = nn.Sequential(*mods)
    def forward(self, x): return self.net(x)

class ICM(nn.Module):
    """
    论文风格：phi(s), 逆模型 g(phi(s), phi(s'))→a_hat, 前向模型 f(phi(s), a)→phi(s')_hat。
    这里简化：直接在原 obs 空间上做（也可加 FeatureNet）。
    """
    def __init__(self, obs_dim: int, act_dim: int, feature_dim=128, lr=1e-3, device="cpu"):
        super().__init__()
        self.device = device
        self.obs_enc = MLP(obs_dim, feature_dim, hidden=256, layers=2)
        self.inv_model = MLP(feature_dim*2, act_dim, hidden=256, layers=2)
        self.fwd_model = MLP(feature_dim+act_dim, feature_dim, hidden=256, layers=2)
        self.to(self.device)
        self.optim = optim.Adam(self.parameters(), lr=lr)
        self.mse = nn.MSELoss()

    def forward(self, s, s_next, a_onehot):
        phi_s = self.obs_enc(s)
        phi_ns = self.obs_enc(s_next)
        inv_in = torch.cat([phi_s, phi_ns], dim=-1)
        a_hat = self.inv_model(inv_in)
        fwd_in = torch.cat([phi_s, a_onehot], dim=-1)
        phi_ns_hat = self.fwd_model(fwd_in)
        return a_hat, phi_ns, phi_ns_hat

    def update(self, s, s_next, a_idx, act_dim, beta=0.2):
        """
        s,s_next: (B,obs_dim) float; a_idx: (B,) long
        beta: 前向损失比重（ICM 论文记法）
        """
        B = s.shape[0]
        a_oh = torch.zeros(B, act_dim, device=self.device)
        a_oh[torch.arange(B), a_idx] = 1.0

        a_hat, phi_ns, phi_ns_hat = self.forward(s, s_next, a_oh)
        inv_loss = self.mse(a_hat, a_oh)
        fwd_loss = self.mse(phi_ns_hat, phi_ns)
        loss = (1 - beta) * inv_loss + beta * fwd_loss

        self.optim.zero_grad()
        loss.backward()
        self.optim.step()

        # 内在奖励 = 前向误差（未归一）
        with torch.no_grad():
            intrinsic = ((phi_ns_hat - phi_ns) ** 2).mean(dim=-1)  # (B,)
        return loss.item(), inv_loss.item(), fwd_loss.item(), intrinsic

