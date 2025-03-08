# SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2021 ETH Zurich, Nikita Rudin

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

def play(args):
    env_cfg, train_cfg = task_registry.get_cfgs(name=args.task)
    # override some parameters for testing
    env_cfg.env.num_envs = min(env_cfg.env.num_envs, 50)
    env_cfg.terrain.num_rows = 5
    env_cfg.terrain.num_cols = 5
    env_cfg.terrain.curriculum = False
    env_cfg.noise.add_noise = False
    env_cfg.domain_rand.randomize_friction = False
    env_cfg.domain_rand.push_robots = False

    DO_LOG = True

    # prepare environment
    env, _ = task_registry.make_env(name=args.task, args=args, env_cfg=env_cfg)
    obs = env.get_observations()
    print('obs',obs)

    # load policy
    train_cfg.runner.resume = True
    ppo_runner, train_cfg = task_registry.make_alg_runner(env=env, name=args.task, args=args, train_cfg=train_cfg)
    policy = ppo_runner.get_inference_policy(device=env.device)
    obs = env.get_observations()
    
    # export policy as a jit module (used to run it from C++)
    if EXPORT_POLICY:
        path = os.path.join(LEGGED_GYM_ROOT_DIR, 'logs', train_cfg.runner.experiment_name, 'exported', 'policies')
        export_policy_as_jit(ppo_runner.alg.actor_critic, path)
        print('Exported policy as jit script to: ', path)

    logger = Logger(env.dt)
    robot_index = 0 # which robot is used for logging
    joint_index = 2 # which joint is used for logging
    stop_state_log = 1000#500 # number of steps before plotting states
    stop_rew_log = env.max_episode_length + 1 # number of steps before print average episode rewards
    camera_position = np.array(env_cfg.viewer.pos, dtype=np.float64)
    camera_vel = np.array([1., 1., 0.])
    camera_direction = np.array(env_cfg.viewer.lookat) - np.array(env_cfg.viewer.pos)
    img_idx = 0
    add_cpg_states = ("CPG" in env_cfg.control.control_type)

    for i in range(10*int(env.max_episode_length)):
        actions = policy(obs.detach())
        # print('actions', actions)
        obs, privileged_obs, rews, dones, infos = env.step(actions.detach())
        # print('obs 2', obs)
        # if i == 2:
        #     sys.exit()
        if RECORD_FRAMES:
            if i % 2:
                # filename = os.path.join(LEGGED_GYM_ROOT_DIR, 'logs', train_cfg.runner.experiment_name, 'exported', 'frames', f"{img_idx}.png")
                root_path = os.path.join('/data/isaacgym_logs',curr_time, 'frames')
                filename = os.path.join(root_path, f"{img_idx:04d}.png")
                if not os.path.exists( root_path ):
                    os.makedirs( root_path )
                print(filename)
                env.gym.write_viewer_image_to_file(env.viewer, filename)
                img_idx += 1 
        if MOVE_CAMERA:
            # camera_position += camera_vel * env.dt
            # env.set_camera(camera_position, camera_position + camera_direction)

            robot_pos = np.array(env.root_states[robot_index,0:3].cpu())
            # camera_position = robot_pos + np.array([0.7,-1.2,0.2]) # diagonal front from FR side 
            camera_position = robot_pos + np.array([0,-1.,0.1])
            env.set_camera(camera_position, robot_pos)

        if DO_LOG:
            if i < stop_state_log:
                if add_cpg_states:
                    rpy = wrap_to_pi(get_rpy_tensor(env.base_quat))
                    logger.log_states(
                        {
                            'dof_pos_target': env.actions[robot_index, joint_index].item() * env.cfg.control.action_scale,
                            'dof_pos': env.dof_pos[robot_index, joint_index].item(),
                            'dof_vel': env.dof_vel[robot_index, joint_index].item(),
                            'dof_torque': env.torques[robot_index, joint_index].item(),
                            'command_x': env.commands[robot_index, 0].item(),
                            'command_y': env.commands[robot_index, 1].item(),
                            'command_yaw': env.commands[robot_index, 2].item(),
                            'base_vel_x': env.base_lin_vel[robot_index, 0].item(),
                            'base_vel_y': env.base_lin_vel[robot_index, 1].item(),
                            'base_vel_z': env.base_lin_vel[robot_index, 2].item(),
                            'base_z': env.root_states[robot_index,2].item(),
                            'base_vel_yaw': env.base_ang_vel[robot_index, 2].item(),
                            'contact_forces_z': env.contact_forces[robot_index, env.feet_indices, 2].cpu().numpy(),
                            'cpg_r0': env._cpg.X[robot_index, 0, 0].item(),
                            'cpg_th0': env._cpg.X[robot_index, 1, 0].item(),
                            'cpg_dr0': env._cpg.X_dot[robot_index, 0, 0].item(),
                            'cpg_dth0': env._cpg.X_dot[robot_index, 1, 0].item(),
                            'roll':  rpy[robot_index,0].item(),
                            'pitch': rpy[robot_index,1].item(),
                            'yaw':   rpy[robot_index,2].item()
                        })
                else:
                    logger.log_states(
                        {
                            'dof_pos_target': env.actions[robot_index, joint_index].item() * env.cfg.control.action_scale,
                            'dof_pos': env.dof_pos[robot_index, joint_index].item(),
                            'dof_vel': env.dof_vel[robot_index, joint_index].item(),
                            'box_poses': env.box_poses[robot_index, 0].item(),
                            'landed': env.landed_at[robot_index, 0].item(),
                            'target' : env.target[robot_index, 0].item(),
                            'box_vel': env.box_poses[robot_index, 0].item(),
                            'dof_torque': env.torques[robot_index, joint_index].item(),
                            'command_x': env.commands[robot_index, 0].item(),
                            'command_y': env.commands[robot_index, 1].item(),
                            'command_yaw': env.commands[robot_index, 2].item(),
                            'base_vel_x': env.base_lin_vel[robot_index, 0].item(),
                            'base_vel_y': env.base_lin_vel[robot_index, 1].item(),
                            'base_vel_z': env.base_lin_vel[robot_index, 2].item(),
                            'base_z': env.root_states[robot_index,2].item(),
                            'base_vel_yaw': env.base_ang_vel[robot_index, 2].item(),
                            'contact_forces_z': env.contact_forces[robot_index, env.feet_indices, 2].cpu().numpy()
                        }
                    )
            elif i==stop_state_log:
                logger.plot_states()
            if  0 < i < stop_rew_log:
                if infos["episode"]:
                    num_episodes = torch.sum(env.reset_buf).item()
                    if num_episodes>0:
                        logger.log_rewards(infos["episode"], num_episodes)
            elif i==stop_rew_log:
                logger.print_rewards()

if __name__ == '__main__':
    EXPORT_POLICY = True
    RECORD_FRAMES = False
    MOVE_CAMERA = False
    args = get_args()
    play(args)