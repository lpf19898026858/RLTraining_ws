#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray, Bool
from rl_bridge.msg import RLEvent
from stable_baselines3 import PPO

OBS_DIM = 22
ACT_DIM = 4

class Evaluator:
    def __init__(
        self,
        model_path,
        deterministic=True,
        device="auto",
        no_reset=False,            # True => 模式3：不reset、不统计、不订阅事件
        suppress_logs=False,       # True => 尽量安静（不打印统计等信息）
        obs_topic="uav/observation",
        act_topic="uav/action",
        event_topic="uav/event",
        reset_topic="uav/reset",
    ):
        self.obs = np.zeros(OBS_DIM, dtype=np.float32)
        self.ready = False
        self.deterministic = deterministic
        self.no_reset = no_reset
        self.suppress_logs = suppress_logs

        # --- ROS pubs/subs（话题名完全由参数决定，保持与 Unity Inspector 一致） ---
        self.pub_action = rospy.Publisher(act_topic, Float32MultiArray, queue_size=1)
        if not self.no_reset:
            self.pub_reset = rospy.Publisher(reset_topic, Bool, queue_size=1)

        rospy.Subscriber(obs_topic, Float32MultiArray, self._on_obs)
        # 模式3：既不统计也不reset => 不订阅事件，避免任何 [EP-END] 打印
        if not self.no_reset:
            rospy.Subscriber(event_topic, RLEvent, self._on_event)

        if not self.suppress_logs:
            rospy.loginfo(f"[Eval] loading model: {model_path}")
        self.model = PPO.load(model_path, device=device)

        # 只有会 reset 的评测才做统计/日志
        if not self.no_reset:
            self.ep = 0
            self.win = 0

    # --- ROS callbacks ---
    def _on_obs(self, msg):
        self.obs = np.asarray(msg.data, dtype=np.float32)
        self.ready = True

    def _on_event(self, msg: RLEvent):
        # Only in mode2 (no_reset=False)
        if self.no_reset:
            return
        if msg.done:
            reason = msg.event_type
            self.ep += 1
            if reason == "reached_target":
                self.win += 1
            if not self.suppress_logs:
                sr_win = self.win / max(1, self.ep)
                rospy.loginfo(f"[EP-END][Eval] ep={self.ep} reason={reason} SR={sr_win:.3f}")
            # 通知 Unity 重置
            self.pub_reset.publish(Bool(True))

    # --- main loop ---
    def run(self):
        rate = rospy.Rate(50)  # 和 Unity FixedUpdate 对齐
        while not rospy.is_shutdown():
            if self.ready:
                self.ready = False
                action, _ = self.model.predict(self.obs, deterministic=self.deterministic)
                action = np.clip(np.asarray(action, dtype=np.float32), -1.0, 1.0)
                self.pub_action.publish(Float32MultiArray(data=action))
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("ppo_evaluator")

    # 路径/模式
    model_path    = rospy.get_param("~model_path", "/home/lpf/docker_shared/rltraining_ws/src/rl_bridge/python/checkpoints/ppo_final.zip")
    deterministic = rospy.get_param("~deterministic", True)
    device        = rospy.get_param("~device", "auto")
    no_reset      = rospy.get_param("~no_reset", False)         # True => 模式3
    suppress_logs = rospy.get_param("~suppress_logs", False)    # True => 尽量静默

    # 话题名（和 Unity Inspector 完全一致，不自动拼前缀）
    obs_topic   = rospy.get_param("~obs_topic",   "uav/observation")
    act_topic   = rospy.get_param("~act_topic",   "uav/action")
    event_topic = rospy.get_param("~event_topic", "uav/event")
    reset_topic = rospy.get_param("~reset_topic", "uav/reset")

    rospy.loginfo("--- ppo_evaluator node starting up ---")
    rospy.loginfo(f"Model Path: {model_path}")
    rospy.loginfo(f"Observation Topic: {obs_topic}")
    rospy.loginfo(f"Action Topic: {act_topic}")
    rospy.loginfo(f"No Reset Mode: {no_reset}")

    Evaluator(
        model_path=model_path,
        deterministic=deterministic,
        device=device,
        no_reset=no_reset,
        suppress_logs=suppress_logs,
        obs_topic=obs_topic,
        act_topic=act_topic,
        event_topic=event_topic,
        reset_topic=reset_topic,
    ).run()
    rospy.loginfo("Evaluator instance created. Starting run loop.")


