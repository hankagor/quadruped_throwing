
import numpy as np
import csv
import matplotlib.pyplot as plt
from datetime import datetime
from collections import defaultdict
import numpy as np
import pandas as pd

curr_time = datetime.now().strftime("%m%d%y%H%M%S")

# Update the logger to save to CSV and handle multiple joints/indices
# class CSVLogger(Logger):
#     def __init__(self, dt):
#         super().__init__(dt)
#         self.episode_logs = []
#         self.episode_count = 0
#         self.all_logs = []

#     def log_states(self, state_dict):
#         state_dict['episode_num'] = self.episode_count
#         self.episode_logs.append(state_dict)
#         self.all_logs.append(state_dict)

#     def save_to_csv(self):
#         if not self.all_logs:
#             return
#         keys = self.all_logs[0].keys()
#         filename = 'log_all_episodes.csv'
#         with open(filename, 'w', newline='') as f:
#             writer = csv.DictWriter(f, fieldnames=keys)
#             writer.writeheader()
#             writer.writerows(self.all_logs)
#         print(f'Saved log to {filename}')
#         self.plot_states()
#         self.episode_logs.clear()
#         self.episode_count += 1

#     def plot_states(self):
#         fig, axs = plt.subplots(3, 2, figsize=(12, 10))
#         time = [log['time'] for log in self.episode_logs if log['env_idx'] == 0]

#         for j in range(3):
#             axs[j, 0].plot(time, [log[f'dof_pos_{j}'] for log in self.episode_logs if log['env_idx'] == 0], label=f'dof_pos_{j}')
#             axs[j, 0].set(xlabel='time [s]', ylabel='Position [rad]', title=f'DOF Position {j}')
#             axs[j, 0].legend()

#             axs[j, 1].plot(time, [log[f'dof_vel_{j}'] for log in self.episode_logs if log['env_idx'] == 0], label=f'dof_vel_{j}')
#             axs[j, 1].set(xlabel='time [s]', ylabel='Velocity [rad/s]', title=f'DOF Velocity {j}')
#             axs[j, 1].legend()

#         plt.tight_layout()
#         plt.show()

# Load data from CSV and plot DOF position separately from box poses, landed, and target states for the first three episodes
# Load data from CSV and plot DOF position separately from box poses, landed, and target states for the first three episodes
def load_and_plot(csv_file):
    data = []
    with open(csv_file, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            for key, value in row.items():
                try:
                    row[key] = float(value)
                except ValueError:
                    row[key] = value
                if key == 'episode_num':
                    row[key] = int(value)
            data.append(row)

    episodes = [20, 10, 15]  # First three episodes
    env_indices = [0, 1]  # First two environments
    
    for episode in episodes:
        for env_idx in env_indices:
            time = [row['time'] for row in data if row['env_idx'] == env_idx and row['episode_num'] == episode]
            
            # Plot DOF positions on the same plot
            plt.figure(figsize=(10, 6))
            for j in range(3):
                dof_pos = [row[f'dof_pos_{j}'] for row in data if row['env_idx'] == env_idx and row['episode_num'] == episode]
                plt.plot(time, dof_pos, label=f'DOF Pos {j}')

            plt.xlabel('Time [s]')
            plt.ylabel('Position [rad]')
            plt.title(f'Env {env_idx} Episode {episode} - DOF Positions')
            plt.legend()
            plt.grid(True)
            plt.show()

            # Plot box poses, landed, and target states only for x-axis (index 0)
            box_pos = [row['box_poses_0'] for row in data if row['env_idx'] == env_idx and row['episode_num'] == episode]
            landed = [row['landed_0'] for row in data if row['env_idx'] == env_idx and row['episode_num'] == episode]
            target = [row['target_0'] for row in data if row['env_idx'] == env_idx and row['episode_num'] == episode]

            plt.figure(figsize=(10, 6))
            plt.plot(time, box_pos, label='Box Pos X')
            plt.plot(time, landed, label='Landed X')
            plt.plot(time, target, label='Target X')

            plt.xlabel('Time [s]')
            plt.ylabel('Value')
            plt.title(f'Env {env_idx} Episode {episode} - Box Pos, Landed, Target X')
            plt.legend()
            plt.grid(True)
            plt.show()

# # Modify play function to log all joints/indices and save to CSV
# def play(args):
#     env_cfg, train_cfg = task_registry.get_cfgs(name=args.task)
#     env_cfg.env.num_envs = min(env_cfg.env.num_envs, 50)
#     env_cfg.terrain.num_rows = 5
#     env_cfg.terrain.num_cols = 5
#     env_cfg.terrain.curriculum = False
#     env_cfg.noise.add_noise = False
#     env_cfg.domain_rand.randomize_friction = False
#     env_cfg.domain_rand.push_robots = False

#     env, _ = task_registry.make_env(name=args.task, args=args, env_cfg=env_cfg)
#     obs = env.get_observations()

#     train_cfg.runner.resume = True
#     ppo_runner, train_cfg = task_registry.make_alg_runner(env=env, name=args.task, args=args, train_cfg=train_cfg)
#     policy = ppo_runner.get_inference_policy(device=env.device)
#     obs = env.get_observations()

#     logger = CSVLogger(env.dt)
#     stop_state_log = 1000

#     for episode in range(30):  # Run for 30 episodes
#         for i in range(int(env.max_episode_length)):
#             actions = policy(obs.detach())
#             clipped_actions = torch.clamp(actions, -env.cfg.normalization.clip_actions, env.cfg.normalization.clip_actions)
#             obs, privileged_obs, rews, dones, infos = env.step(actions.detach())
#             for env_idx in range(env.num_envs):
#                 logger.log_states(
#                     {
#                         'time': i * env.dt,
#                         'env_idx': env_idx,
#                         **{f'dof_pos_{j}': env.dof_pos[env_idx, j].detach().cpu().item() for j in range(3)},
#                         **{f'dof_vel_{j}': env.dof_vel[env_idx, j].detach().cpu().item() for j in range(3)},
#                         **{f'box_poses_{j}': env.box_poses[env_idx, j].detach().cpu().item() for j in range(3)},
#                         **{f'landed_{j}': env.landed_at[env_idx, j].detach().cpu().item() for j in range(3)},
#                         **{f'target_{j}': env.target[env_idx, j].detach().cpu().item() for j in range(3)},
#                         'reward': rews[env_idx].detach().cpu().item(),
#                         'clipped_action': clipped_actions[env_idx].detach().cpu().numpy().tolist()
#                     }
#                 )
#         logger.save_to_csv()

#         if infos['episode']:
#             num_episodes = torch.sum(env.reset_buf).item()
#             if num_episodes > 0:
#                 logger.log_rewards(infos['episode'], num_episodes)
#         logger.print_rewards()

def calculate_average_deviation(data):
    deviations = []

    for key, values in data.items():
        # Convert to numpy array to enable vectorized operations
        values = np.array(values)
        if key < 2.0:
            continue
        # Calculate the absolute deviations from the key (target)
        deviations.append(np.abs(np.mean(values) - key))
        
        # Append deviations to the list
       # deviations.extend(absolute_deviation)#
    
    # Calculate overall average deviation
    average_deviation = np.mean(deviations)
    
    return average_deviation

def aggregate_landed_by_target(csv_file):
    aggregation_map = defaultdict(list)

    with open(csv_file, 'r') as f:
        reader = csv.DictReader(f)
        last_entries = {}

        for row in reader:
            episode = int(row['episode_num'])
            env_idx = int(row['env_idx'])
            key = (episode, env_idx)
            last_entries[key] = row

        for entry in last_entries.values():
            target_x = float(entry['target_0'])
            landed_x = float(entry['landed_0'])
            aggregation_map[target_x].append(landed_x)

    target_x_values = sorted(aggregation_map.keys())
    mean_values = [np.mean(aggregation_map[target_x]) for target_x in target_x_values]
    std_values = [np.std(aggregation_map[target_x]) for target_x in target_x_values]
    goal_positions = []

    measured_positions = []


    data = aggregation_map
    for goal, measurements in data.items():

        goal_positions.extend([goal] * len(measurements))

        measured_positions.extend(measurements)



    # Calculate mean of goal positions

    goal_mean = np.mean(goal_positions)



    # Calculate SSR and SST

    SSR = np.sum((np.array(goal_positions) - np.array(measured_positions))**2)

    SST = np.sum((np.array(goal_positions) - goal_mean)**2)



    # Calculate R²

    R_squared = 1 - (SSR / SST)

    # print(R_squared)
    # print(target_x_values)
    # print(mean_values)
    # print(std_values)
    # Plot results
    plt.figure(figsize=(10, 6))

    plt.plot(target_x_values, mean_values, '-o', label='Mean Position')
    plt.plot(target_x_values, target_x_values, '--r', label = 'Goal')
    plt.fill_between(target_x_values, 
                     np.array(mean_values) - np.array(std_values), 
                     np.array(mean_values) + np.array(std_values), 
                     alpha=0.3, label='Std Dev')
    plt.xlabel('Goal Position (m)')
    plt.ylabel('Measured Position (m)')
    plt.title('RL control')
    plt.legend()
    plt.grid(True)
    plt.show()
    # Calculate error as the difference between target_x and landed_x for the aggregation map
    errors = {key: np.abs(np.array(values) - key) for key, values in aggregation_map.items()}
    ranges = [(0.5, 1.0), (1.0, 2.0), (2.0, 3.0)]
    # Grouping and calculating mean/std for errors
    table_data = []

    for r in ranges:
        # Filter error points within the specified range
        filtered_errors = [err for key, values in errors.items() if r[0] <= key < r[1] for err in values]
        mean_error = np.mean(filtered_errors)
        std_error = np.std(filtered_errors)
        
        table_data.append([f"{r[0]} - {r[1]} m", f"{mean_error:.3f} ± {std_error:.3f}"])
        # print(mean_error, std_error)

    df = pd.DataFrame(table_data, columns=["Category", "Quadruped (RL Policy)"])

    

# Calculate for data1 (Quadratic Interpolation)
    average_deviation_quadratic = calculate_average_deviation(aggregation_map)
    print(average_deviation_quadratic)
    # import ace_tools as tools; tools.display_dataframe_to_user(name="Error Comparison Table (RL Policy)", dataframe=df)

    return aggregation_map

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

# def calculate_average_deviation(data):
#     deviations = []

#     for key, values in data.items():
#         # Convert to numpy array to enable vectorized operations
#         values = np.array(values)
        
#         # Calculate the absolute deviations from the key (target)
#         absolute_deviation = np.abs(values - key)
        
#         # Append deviations to the list
#         deviations.extend(absolute_deviation)
    
#     # Calculate overall average deviation
#     average_deviation = np.mean(deviations)
    
    # return average_deviation
if __name__ == '__main__':
    # load_and_plot('log_all_episodes.csv')
    data = aggregate_landed_by_target('log_RL.csv')
    
    # Prepare goal and measured positions

    