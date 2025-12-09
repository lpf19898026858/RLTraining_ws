import numpy as np

class BaseRLAgent:
    def act(self, obs: np.ndarray) -> np.ndarray:
        raise NotImplementedError
    def learn(self, obs, action, reward, next_obs, done):
        pass

