import rospy
import numpy as np
from rl_bridge.msg import CurriculumStatus


class CurriculumStage:
    """单个课程阶段定义"""
    def __init__(self, name: str, threshold: float, min_steps: int, value: float):
        self.name = name                # 阶段名称
        self.threshold = threshold      # 平均奖励达标阈值（基于 episode total reward）
        self.min_steps = min_steps      # 至少多少 episode 才能尝试晋级
        self.value = value              # 对应课程参数值
        self.rewards = []               # 存放每个 episode 的总奖励


class CurriculumManager:
    """
    管理一个课程维度（X / Z / Local）
    - 负责阶段晋级
    - 统计成功率（窗口 / 总体）
    - 发布 CurriculumStatus 消息
    """

    def __init__(self, topic_name: str, window_size: int = 1000):
        self.topic_name = topic_name
        self.pub = rospy.Publisher(topic_name, CurriculumStatus, queue_size=1)
        self.stages = []
        self.idx = 0                     # 当前阶段索引
        self.window_size = window_size

        # 成功率统计
        self.success_history = []
        self.total_episodes = 0
        self.total_successes = 0
        self._last_published = -1

    # ======================
    # --- 阶段添加与访问 ---
    # ======================
    def add_stage(self, name: str, threshold: float, min_steps: int, value: float):
        """添加课程阶段"""
        self.stages.append(CurriculumStage(name, threshold, min_steps, value))

    def current_value(self) -> float:
        """返回当前阶段课程参数值"""
        if not self.stages:
            return 5.0
        i = min(self.idx, len(self.stages) - 1)
        return self.stages[i].value

    def current_name(self) -> str:
        """返回当前阶段名称"""
        if not self.stages:
            return "None"
        i = min(self.idx, len(self.stages) - 1)
        return self.stages[i].name

    def record_reward(self, episode_total_reward: float):
        """记录一次完整 episode 的总奖励"""
        if not self.stages:
            return
        i = min(self.idx, len(self.stages) - 1)
        self.stages[i].rewards.append(episode_total_reward)

    def get_avg_reward(self) -> float:
        """
        ✨ 修改: 计算并返回当前阶段【所有】episode 的平均总奖励。
        这个指标用于在 TensorBoard 中进行宏观的性能监控。
        如果当前阶段没有任何数据，则返回 NaN。
        """
        if not self.stages or self.idx >= len(self.stages):
            return float('nan')

        st = self.stages[self.idx]
        # 检查当前阶段是否有任何奖励记录
        if not st.rewards:
            return float('nan')

        # ✨ 核心修改: 对 st.rewards 中的所有数据求平均
        avg_r = float(np.mean(st.rewards))
        return avg_r

    def get_sliding_window_avg_reward(self) -> float:
        """
        计算并返回用于晋级判断的【滑动窗口】平均奖励。
        如果数据不足，返回 NaN。
        """
        if not self.stages or self.idx >= len(self.stages):
            return float('nan')

        st = self.stages[self.idx]
        # 检查是否有足够的数据来计算滑动平均
        if len(st.rewards) < st.min_steps:
            return float('nan')

        # 计算并返回最近 min_steps 个 episode 的平均总奖励
        return float(np.mean(st.rewards[-st.min_steps:]))

    def maybe_advance(self) -> bool:
        """检查是否满足晋级条件"""
        # --- ✨ 2. 修改此处以调用新方法，使逻辑更清晰 ✨ ---
        avg_r_for_advance = self.get_sliding_window_avg_reward()

        # 如果返回的是 NaN (因为数据不足)，则直接返回 False
        if np.isnan(avg_r_for_advance):
            return False

        st = self.stages[self.idx]
        rospy.loginfo(
            f"[CurriculumCheck] {self.topic_name} | 当前阶段={st.name} | "
            f"avg_reward(window)={avg_r_for_advance:.3f} / threshold={st.threshold:.3f} | "
            f"episodes={len(st.rewards)}"
        )

        if avg_r_for_advance >= st.threshold:
            # (晋级逻辑保持不变)
            self.idx += 1
            self._publish()
            i_next = min(self.idx, len(self.stages) - 1)
            st_next = self.stages[i_next]
            rospy.loginfo(
                f"[CurriculumAdvance] {self.topic_name} 晋级 → {st_next.name} (value={st_next.value})"
            )
            return True

        return False

    def record_episode(self, success: bool):
        """记录一个 episode 的成功 / 失败"""
        self.total_episodes += 1
        self.success_history.append(1 if success else 0)
        if success:
            self.total_successes += 1

        # 限制滑动窗口长度
        if len(self.success_history) > self.window_size:
            self.success_history.pop(0)

    @property
    def window_success_rate(self) -> float:
        """最近 N 回合成功率"""
        if not self.success_history:
            return 0.0
        return float(np.mean(self.success_history[-self.window_size:]))

    @property
    def total_success_rate(self) -> float:
        """总体成功率"""
        if self.total_episodes == 0:
            return 0.0
        return float(self.total_successes / self.total_episodes)

    def _publish(self):
        """发布当前阶段信息"""
        i = min(self.idx, len(self.stages) - 1)
        st = self.stages[i]
        msg = CurriculumStatus()
        msg.lesson_id = i
        msg.lesson_name = st.name
        msg.changed = (i != self._last_published)
        self.pub.publish(msg)
        self._last_published = i

    def get_status_dict(self):
        """返回当前课程状态，用于 TensorBoard"""
        return {
            "lesson_id": self.idx,
            "lesson_name": self.current_name(),
            "window_success_rate": self.window_success_rate,
            "total_success_rate": self.total_success_rate,
            "current_value": self.current_value(),
        }

