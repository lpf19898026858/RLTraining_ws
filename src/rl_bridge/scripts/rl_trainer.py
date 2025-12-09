#!/usr/bin/env python3
import rospy
from rl_bridge.msg import RLObservation, RLAction
import numpy as np

class RLTrainer:
    def __init__(self):
        rospy.init_node('rl_trainer')
        self.pub = rospy.Publisher('/rl/action', RLAction, queue_size=10)
        rospy.Subscriber('/rl/observation', RLObservation, self.obs_callback)
        self.rate = rospy.Rate(30)

    def obs_callback(self, msg):
        pos = np.array([msg.x, msg.y])
        vel = np.array([msg.vx, msg.vy])
        target = np.array([3.0, 3.0])

        direction = target - pos
        dist = np.linalg.norm(direction) + 1e-6
        direction /= dist  # 单位化方向

        # 根据距离动态调节力度（距离大 → 力度大）
        gain = min(1.0, dist / 3.0)   # 限制在 [0,1]
        force_mag = 3.0 * gain        # 最大不超过 5
        action_vec = force_mag * direction

        # 阻尼（防止振荡）
        damping = 0.8 * vel
        action_vec -= damping

        # 输出动作
        action = RLAction()
        action.ax = float(action_vec[0])
        action.ay = float(action_vec[1])
        self.pub.publish(action)

    def spin(self):
        rospy.loginfo("RL Trainer (unity_rl_bridge) running...")
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    RLTrainer().spin()

