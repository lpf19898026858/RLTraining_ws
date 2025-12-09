import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray, Bool
from rl_bridge.msg import RLEvent
from reward_module import compute_reward
from curriculum_manager import CurriculumManager
from yaml_curriculum_loader import load_three_axes_from_yaml
from reward_module import compute_reward, reset_reward_state

class DroneRLEnv:
    """
    封装 ROS-Unity 环境接口：
    - /uav/observation Float32MultiArray(22)
    - /uav/action      Float32MultiArray(4)
    - /uav/event       RLEvent{event_type, data[9], done}
    - /uav/reset       Bool(True)
    - /uav/curriculum_x|z|local  CurriculumStatus (发布给Unity)

    提供 Gym-like 接口 reset()/step()
    """

    def __init__(self, yaml_path="drone_config.yaml", spin_rate_hz=50):
        # === 状态与维度 ===
        self.obs_dim, self.action_dim = 22, 4
        self.obs = np.zeros(self.obs_dim, dtype=np.float32)
        self.ready = False
        self.done = False
        self.reward = 0.0
        self.total_reward = 0.0
        #self.episode_active = True # ✅ 新增一个状态锁
        #self.waiting_for_first_obs = True # <-- 新增状态标志
        #self.info={} # 用于存储额外信息,如是否被截断
        #self.new_obs_received = False # <-- ✅ 使用这个更清晰的标志
        rospy.loginfo("[DroneRLEnv] init ROS...")

        # === ROS 通信 ===
        self.pub_action = rospy.Publisher("/uav/action", Float32MultiArray, queue_size=1)
        self.pub_reset = rospy.Publisher("/uav/reset", Bool, queue_size=1)
        rospy.Subscriber("/uav/observation", Float32MultiArray, self._on_obs)
        rospy.Subscriber("/uav/event", RLEvent, self._on_event)

        # === 课程管理（X/Z/Local） ===
        self.cx = CurriculumManager("/uav/curriculum_x")
        self.cz = CurriculumManager("/uav/curriculum_z")
        self.cl = CurriculumManager("/uav/curriculum_local")
        
        x_list, z_list, l_list = load_three_axes_from_yaml(yaml_path)
        for n, t, m, v in x_list:
            self.cx.add_stage(n, t, m, v)
        for n, t, m, v in z_list:
            self.cz.add_stage(n, t, m, v)
        for n, t, m, v in l_list:
            self.cl.add_stage(n, t, m, v)
        print("Current Curriculum X thresholds:", [s.threshold for s in self.cx.stages])
        rospy.loginfo(
            f"[DroneRLEnv] curriculum loaded: X={len(self.cx.stages)}, Z={len(self.cz.stages)}, L={len(self.cl.stages)}"
        )

        # 初始化发布一次，确保 Unity 收到初始阶段
        self.cx._publish()
        self.cz._publish()
        self.cl._publish()

        # === 全局成功率统计 ===
        self.total_episodes = 0
        self.total_successes = 0
        self.success_history = []
        self.window_size = 1000
	
        self.spin_dt = 1.0 / spin_rate_hz

    # ---------- ROS callbacks ----------
    def _on_obs(self, msg):
        """接收 Unity 观测"""
        self.obs = np.asarray(msg.data, dtype=np.float32)     
        self.ready=True   
        #self.obs = np.clip(self.obs / 10.0, -1.0, 1.0)
        #if not self.episode_active:
        #    self.episode_active = True
        #self.new_obs_received = True # ✅ 收到新观测时，设置标志

    def _on_event(self, msg):
        """接收 Unity 事件（奖励 & done）"""
         # ✅ 核心守卫：如果当前回合已经结束，则忽略所有后续的事件消息。
        #if not self.episode_active:
        #    # 可以取消下面这行注释来调试，看看是否真的拦截了很多“死亡回声”
            # rospy.logwarn(f"Ignoring stale event '{msg.event_type}' because episode is already done.")
        #    return
        r_ext, done = compute_reward(msg.event_type, msg.data)
        self.reward = float(r_ext)
        self.done = bool(done)
        self.total_reward += self.reward

        #self.info = {}
	
        # ✅ 只在 episode 结束时统计
        if self.done:
            #if msg.event_type == "timeout":
            #    self.info["TimeLimit.truncated"] = True
            #    rospy.loginfo("Received timeout!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            #else:
            #    self.info["TimeLimit.truncated"] = False
            
        # 3.2 记录回合的最终信息到 info 字典，方便 Monitor 等回调使用
            #self.info["episode"] = {
            #    "r": self.total_reward,
            #    "l": 0, # 步长信息通常由 Wrapper (如Monitor) 统计，这里写0即可
            #    "t": rospy.Time.now().to_sec()
        #}
            #self.episode_active = False 
            
            is_success = (msg.event_type == "reached_target")
            # === 1. 记录全局成功率 ===
            self.total_episodes += 1
            if is_success:
                self.total_successes += 1
                self.success_history.append(1)
            else:
                self.success_history.append(0)
            if len(self.success_history) > self.window_size:
                self.success_history.pop(0)

            # === 2. 向课程系统记录 episode 总 reward ===
            self.cx.record_reward(self.total_reward)
            self.cz.record_reward(self.total_reward)
            self.cl.record_reward(self.total_reward)

            self.cx.record_episode(is_success)
            self.cz.record_episode(is_success)
            self.cl.record_episode(is_success)
            
            # === 3. 检查晋级 ===
            _adv = self.cx.maybe_advance() | self.cz.maybe_advance() | self.cl.maybe_advance()

            # === 4. 打印调试日志（可选）===
            rospy.loginfo(
                f"[Episode Done] reason={msg.event_type} | total_reward={self.total_reward:.2f} | "
                f"window_SR={self.window_success_rate:.3f} | total_SR={self.total_success_rate:.3f}"
            )

            # === 5. 通知 Unity 重置并清零累计 ===
            self.pub_reset.publish(Bool(True))
            self.total_reward = 0.0

    # ---------- 成功率属性 ----------
    @property
    def window_success_rate(self):
        """最近 N 回合的成功率"""
        if not self.success_history:
            return 0.0
        return np.mean(self.success_history[-self.window_size:])

    @property
    def total_success_rate(self):
        """总体成功率"""
        if self.total_episodes == 0:
            return 0.0
        return self.total_successes / self.total_episodes

    # ---------- Gym-like ----------
    def reset(self):
        """环境重置"""
        #rospy.loginfo("[DroneRLEnv.reset] Issuing reset request to Unity...")
        # 1. 重置内部状态
        self.total_reward = 0.0
        self.done = False
        self.ready = False
        
        #self.new_obs_received = False # <-- 清除观测接收标志
        reset_reward_state()
        # 3. 发布 reset 消息来触发 Unity 重置
        self.pub_reset.publish(Bool(True))
        #rospy.sleep(0.05)
        #self.episode_active = True
        
        # 3. 在这里阻塞，直到收到 Unity 重置后的第一个观测
        t0 = rospy.Time.now().to_sec()
        timeout = 5.0 # 设置一个5秒的超时，防止无限等待
        while not self.ready and (rospy.Time.now().to_sec() - t0) < 2.0:
            rospy.sleep(self.spin_dt)

        #if not self.new_obs_received:
        #    rospy.logerr(f"Reset timeout! Did not receive observation from Unity within {timeout}s.")
            # 在超时的情况下，返回一个零观测，但打印错误
        #    return np.zeros(self.obs_dim, dtype=np.float32)

        # 4. 返回真实、有效的初始观测
        return self.obs

    def step(self, action: np.ndarray):
        """单步交互 (修正版)"""
        # 1. 清除观测标志，准备接收下一步的观测
        #self.new_obs_received = False
        
        # 2. 发布动作
        self.pub_action.publish(Float32MultiArray(data=action.astype(np.float32)))
        
        # 3. 等待 Unity 响应 (收到新的观测)
        #    这个循环确保了我们是在收到这个action的结果后才返回
        t0 = rospy.Time.now().to_sec()
        #timeout = 0.5 # 超时可以设短一些，比如0.5秒
        while not self.ready and (rospy.Time.now().to_sec() - t0) < 0.5:
            rospy.sleep(self.spin_dt)

        #if not self.new_obs_received:
        #    rospy.logwarn(f"Step timeout! Using stale observation/reward after {timeout}s.")
            # 如果超时，我们只能返回上一步的数据，但self.done可能已经更新了
            # 这种情况会引入噪声，但比完全卡死要好。
            # 如果频繁出现此警告，说明Unity物理帧率或ROS通信有问题。
        obs, r, d = self.obs, self.reward, self.done
        self.ready = False
        # 4. 返回最新的状态
        return obs, r, d, {}

    # ---------- TensorBoard 用 ----------
    def get_curriculum_status(self):
        """返回课程与全局状态"""
        return {
            "x": self.cx.get_status_dict(),
            "z": self.cz.get_status_dict(),
            "local": self.cl.get_status_dict(),
            "global": {
                "window_success_rate": self.window_success_rate,
                "total_success_rate": self.total_success_rate,
                "episode_reward_mean":self.cx.get_avg_reward(),
                "episode_reward_mean_window":self.cx.get_sliding_window_avg_reward(),
            },
        }

