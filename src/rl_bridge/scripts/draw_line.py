import matplotlib.pyplot as plt
from stable_baselines3.common.results_plotter import load_results, ts2xy
log_dir = "./ppo_ball/"
x, y = ts2xy(load_results(log_dir), 'timesteps')
plt.plot(x, y)
plt.xlabel('Timesteps')
plt.ylabel('Reward')
plt.show()

