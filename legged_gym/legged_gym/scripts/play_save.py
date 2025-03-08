from legged_gym import LEGGED_GYM_ROOT_DIR
import os

import isaacgym
from legged_gym.envs import *
from legged_gym.utils import  get_args, export_policy_as_jit, task_registry, Logger
from isaacgym.torch_utils import *
from legged_gym.utils.math import wrap_to_pi
from datetime import datetime 

import numpy as np
import torch

curr_time = datetime.now().strftime("%m%d%y%H%M%S")

import csv
import matplotlib.pyplot as plt
from datetime import datetime

# Update the logger to save to CSV and handle multiple joints/indices
class CSVLogger(Logger):
    def __init__(self, dt):
        super().__init__(dt)
        self.episode_logs = []
        self.episode_count = 0
        self.all_logs = []

    def log_states(self, state_dict):
        state_dict['episode_num'] = self.episode_count
        self.episode_logs.append(state_dict)
        self.all_logs.append(state_dict)

    def save_to_csv(self):
        if not self.all_logs:
            return
        keys = self.all_logs[0].keys()
        filename = 'log_all_episodes.csv'
        with open(filename, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=keys)
            writer.writeheader()
            writer.writerows(self.all_logs)
        print(f'Saved log to {filename}')
        # self.plot_states()
        self.episode_logs.clear()
        self.episode_count += 1

    def plot_states(self):
        fig, axs = plt.subplots(3, 2, figsize=(12, 10))
        time = [log['time'] for log in self.episode_logs if log['env_idx'] == 0]

        for j in range(3):
            axs[j, 0].plot(time, [log[f'dof_pos_{j}'] for log in self.episode_logs if log['env_idx'] == 0], label=f'dof_pos_{j}')
            axs[j, 0].set(xlabel='time [s]', ylabel='Position [rad]', title=f'DOF Position {j}')
            axs[j, 0].legend()

            axs[j, 1].plot(time, [log[f'dof_vel_{j}'] for log in self.episode_logs if log['env_idx'] == 0], label=f'dof_vel_{j}')
            axs[j, 1].set(xlabel='time [s]', ylabel='Velocity [rad/s]', title=f'DOF Velocity {j}')
            axs[j, 1].legend()

        plt.tight_layout()
        plt.show()

# Modify play function to log all joints/indices and save to CSV
def play(args):
    env_cfg, train_cfg = task_registry.get_cfgs(name=args.task)
    env_cfg.env.num_envs = min(env_cfg.env.num_envs, 50)
    env_cfg.terrain.num_rows = 5
    env_cfg.terrain.num_cols = 5
    env_cfg.terrain.curriculum = False
    env_cfg.noise.add_noise = False
    env_cfg.domain_rand.randomize_friction = False
    env_cfg.domain_rand.push_robots = False

    env, _ = task_registry.make_env(name=args.task, args=args, env_cfg=env_cfg)
    obs = env.get_observations()

    train_cfg.runner.resume = True
    ppo_runner, train_cfg = task_registry.make_alg_runner(env=env, name=args.task, args=args, train_cfg=train_cfg)
    policy = ppo_runner.get_inference_policy(device=env.device)
    obs = env.get_observations()

    logger = CSVLogger(env.dt)
    stop_state_log = 1000

    for episode in range(50):  # Run for 30 episodes
        for i in range(int(env.max_episode_length)):
            actions = policy(obs.detach())
            clipped_actions = torch.clip(actions, -env.cfg.normalization.clip_actions, env.cfg.normalization.clip_actions).to(env.device)
            # clipped_actions = torch.clamp(actions, -env.cfg.normalization.clip_actions, env.cfg.normalization.clip_actions)
            obs, privileged_obs, rews, dones, infos = env.step(actions.detach())
            for env_idx in range(env.num_envs):
                logger.log_states(
                    {
                        'time': i * env.dt,
                        'env_idx': env_idx,
                        **{f'dof_pos_{j}': env.dof_pos[env_idx, j].item() for j in range(3)},
                        **{f'dof_vel_{j}': env.dof_vel[env_idx, j].item() for j in range(3)},
                        **{f'box_poses_{j}': env.box_poses[env_idx, j].item() - env.env_origins[env_idx, j].item() for j in range(3)},
                        **{f'landed_{j}': env.landed_at[env_idx, j].item() for j in range(3)},
                        **{f'target_{j}': env.target[env_idx, j].item() for j in range(3)},
                        'reward': rews[env_idx].detach().cpu().item(),
                        'clipped_action': clipped_actions[env_idx].detach().cpu().numpy().tolist()
                    }
                )
        logger.save_to_csv()

        if infos['episode']:
            num_episodes = torch.sum(env.reset_buf).item()
            if num_episodes > 0:
                logger.log_rewards(infos['episode'], num_episodes)
        logger.print_rewards()

if __name__ == '__main__':
    EXPORT_POLICY = True
    args = get_args()
    play(args)
