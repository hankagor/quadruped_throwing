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

from .base_config import BaseConfig

# Default task - forward locomotion
ROUGH_TERRAIN = False; PERCEPTIVE_FLAG = False
JOINT_PD = True

ADD_NOISE  = False # for sim2real / avoiding overfitting 
EXTRA_HARD = False # for sim2real / avoiding overfitting 
TEST_TIME = False 

OBS_HISTORY = 3

class LeggedRobotCfg(BaseConfig):
    class env:
        num_envs = 4096
        num_privileged_obs = None # if not None a priviledge_obs_buf will be returned by step() (critic obs for assymetric training). None is returned otherwise 
        num_actions = 3 # switch based on desired environment.... 
        env_spacing = 4. # not used with heightfields/trimeshes 
        send_timeouts = True # send time out information to the algorithm
        episode_length_s = 1.5 # episode length in seconds

        boxes = False
        targets = False
        if TEST_TIME:
            episode_length_s = 200

        num_observations = 56  
        if not JOINT_PD:
            num_observations = 56 + 4*4   # -4 due to actions if not omni, 4 states each for r, dr, theta, dtheta, 2 for BH/GC
            num_observations -= 8 # remove contacts and BH/GC , quaternion
            num_actions = 8
        if ROUGH_TERRAIN and PERCEPTIVE_FLAG:
            num_observations += 17*11 # if use measured height points
        if boxes:
            num_observations += 3
        num_observations = 18
        if targets:
            num_observations += 3
        if OBS_HISTORY > 0:
            num_observations *= OBS_HISTORY
            num_stacked_observations = OBS_HISTORY
        
    

    class terrain:
        # mesh_type = 'trimesh' # "heightfield" # none, plane, heightfield or trimesh
        horizontal_scale = 0.1 # [m]
        vertical_scale = 0.005 # [m]
        border_size = 25 # [m]
        curriculum = True
        static_friction = 5.0
        dynamic_friction = 5.0
        restitution = 0.
        # rough terrain only:
        # measure_heights = True
        measured_points_x = [-0.8, -0.7, -0.6, -0.5, -0.4, -0.3, -0.2, -0.1, 0., 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8] # 1mx1.6m rectangle (without center line)
        measured_points_y = [-0.5, -0.4, -0.3, -0.2, -0.1, 0., 0.1, 0.2, 0.3, 0.4, 0.5]
        selected = False # select a unique terrain type and pass all arguments
        terrain_kwargs = None # Dict of arguments for selected terrain
        max_init_terrain_level = 5 # starting curriculum state
        terrain_length = 8.
        terrain_width = 8.
        num_rows= 10 # number of terrain rows (levels)
        num_cols = 20 # number of terrain cols (types)
        # terrain types: [smooth slope, rough slope, stairs up, stairs down, discrete]
        terrain_proportions = [0.1, 0.1, 0.35, 0.25, 0.2]
        # trimesh only:
        slope_treshold = 0.75 # slopes above this threshold will be corrected to vertical surfaces

        mesh_type = 'plane'
        measure_heights = False

        rough_terrain = ROUGH_TERRAIN
        test_time = TEST_TIME

        if ROUGH_TERRAIN:
            if PERCEPTIVE_FLAG:
                measure_heights = True

            mesh_type = 'trimesh'
            # terrain_proportions = [0., 1., 0., 0., 0., 0., 0., 0.] # rough slope
            # if TERRAIN_TYPE == "TREES":
            #     terrain_proportions = [0., 0., 0., 0., 0., 1, 0., 0.] # stepping stones, trees, or horizontal gap (check terrain.py)
            #     for i,x in enumerate(measured_points_x):
            #         measured_points_x[i] += 0.3 # only slightly farther look ahead (1.1m), these transferred decently 
            #     curriculum = False # turn curriculum off

            # elif TERRAIN_TYPE == "GAP":
            #     terrain_proportions = [0., 0., 0., 0., 0., 0, 1., 0.] # gap crossing 
            #     horizontal_scale = 0.05
            #     vertical_scale = 0.02
            #     for i,x in enumerate(measured_points_x):
            #         measured_points_x[i] += 0.3 # only slightly farther look ahead (1.1m), 
            # else:
            #     # random rough terrain
            #     terrain_proportions = [0., 1., 0., 0., 0., 0., 0., 0.] # rough slope
            #     measure_heights = False



    class commands:
        curriculum = False
        max_curriculum = 1.
        num_commands = 4 # default: lin_vel_x, lin_vel_y, ang_vel_yaw, heading (in heading mode ang_vel_yaw is recomputed from heading error)
        resampling_time = 5 # time before command are changed[s] (was 5)

        heading_command = False # if true: compute ang vel command from heading error
        class ranges:
            # original
            # lin_vel_x = [-1.0, 1.0]   # min max [m/s]
            # lin_vel_y = [-1.0, 1.0]   # min max [m/s]
            # ang_vel_yaw = [-1, 1]     # min max [rad/s]
            # heading = [-3.14, 3.14]

            # lin_vel_x = [0.2, 1]      # 
            # lin_vel_y = [-1e-7, 1e-7]   # min max [m/s]
            # ang_vel_yaw = [-1e-7, 1e-7] 
            # heading = [-1e-7, 1e-7]

            # stand still
            # min max [m/s]
            lin_vel_x = [0.0, 0.0]   # min max [m/s]
            lin_vel_y = [0.0, 0.0]   # min max [m/s]
            ang_vel_yaw = [0.0, 0.0]     # min max [rad/s]
            heading = [0.0, 0.0]

    class init_state:
        rot = [0.0, -0.7, 0.0, 0.7] # x,y,z,w [quat]
        lin_vel = [0.0, 0.0, 0.0]  # x,y,z [m/s]
        ang_vel = [0.0, 0.0, 0.0]  # x,y,z [rad/s]

        pos = [0.0, 0.0, 0.415] # x,y,z [m]
        if EXTRA_HARD:
            pos = [0.0, 0.0, 0.35] # x,y,z [m]


        # open grip - 0.204007 1.67837 -0.956219 -0.203971 1.67837 -0.956216 0.00260456 2.3222 -2.29922 -0.00282896 2.32316 -2.30011 
        # closed grip - 0.489253 1.71581 -0.986607 -0.489205 1.71581 -0.986614 0.00307185 2.32016 -2.3016 -0.00337036 2.32121 -2.30257 

        # default_joint_angles = { # = target angles [rad] when action = 0.0
        #     'BFL_hip_joint': 0.,   # [rad]
        #     'DRL_hip_joint': 0,   # [rad]
        #     'AFR_hip_joint': 0 ,  # [rad]
        #     'CRR_hip_joint': 0.,   # [rad]

        #     'BFL_thigh_joint': 1.2566,   # [rad]
        #     'DRL_thigh_joint': 1.2566,   # [rad]
        #     'AFR_thigh_joint': 2.3181,   # [rad]
        #     'CRR_thigh_joint': 2.3181,   # [rad]

        #     'BFL_calf_joint': -2.5562,   # [rad]
        #     'DRL_calf_joint': -2.5562,   # [rad]
        #     'AFR_calf_joint': -2.2331,   # [rad]
        #     'CRR_calf_joint': -2.2331,   # [rad]
        # }
        # self.default_foot_pos = to_torch([0., 0.0838, -0.3, 0., -0.0838, -0.3, 0., 0.0838, -0.3, 0., -0.0838, -0.3], device=self.device)

        default_joint_angles = { # = target angles [rad] when action = 0.0
            'BFL_hip_joint': -0.203971,   # [rad]
            'DRL_hip_joint': 0,   # [rad]
            'AFR_hip_joint': 0.204007,  # [rad]
            'CRR_hip_joint': 0.,   # [rad]

            'BFL_thigh_joint': 1.67837,   # [rad]
            'DRL_thigh_joint': 2.32016,   # [rad]
            'AFR_thigh_joint': 1.67837,   # [rad]
            'CRR_thigh_joint': 2.32016,   # [rad]

            'BFL_calf_joint': -0.956216,   # [rad]
            'DRL_calf_joint': -2.30162,   # [rad]
            'AFR_calf_joint': -0.956219,   # [rad]
            'CRR_calf_joint': -2.3016,   # [rad]
        }

        RL_default = [
            default_joint_angles['DRL_hip_joint'],
            default_joint_angles['DRL_thigh_joint'],
            default_joint_angles['DRL_calf_joint']
        ]

        RR_default = [
            default_joint_angles['CRR_hip_joint'],
            default_joint_angles['CRR_thigh_joint'],
            default_joint_angles['CRR_calf_joint']
        ]
        grip_joint_angles = {
            'BFL_hip_joint': -0.489205,   # [rad]
            'DRL_hip_joint': 0,   # [rad]
            'AFR_hip_joint': 0.489253,  # [rad]
            'CRR_hip_joint': 0.,   # [rad]

            'BFL_thigh_joint': 1.71581,   # [rad]
            'DRL_thigh_joint': 2.32016,   # [rad]
            'AFR_thigh_joint': 1.71581,   # [rad]
            'CRR_thigh_joint': 2.32016,   # [rad]

            'BFL_calf_joint': -0.986614,   # [rad]
            'DRL_calf_joint': -2.30162,   # [rad]
            'AFR_calf_joint': -0.986607,   # [rad]
            'CRR_calf_joint': -2.3016,   # [rad]
        }
        # -0.01201 -0.083 -0.1526 -0.01201 0.083 -0.1526 -0.16369 -0.083 -0.063343 -0.16369 0.083 -0.063343
        default_foot_pos = { # = target foot pos [m] when action = 0.0
            'FL_x': 0.16369,   # [m]
            'RL_x': -0.01201,   # [m]
            'FR_x': 0.16369 ,  # [m]
            'RR_x': -0.01201,   # [m]

            'FL_y':  0.0838,   # [m]
            'RL_y':  0.0838,   # [m]
            'FR_y': -0.0838,   # [m]
            'RR_y': -0.0838,   # [m]

            'FL_z': -0.063343,   # [m]
            'RL_z': -0.1526,   # [m]
            'FR_z': -0.063343,   # [m]
            'RR_z': -0.1526,   # [m]
        }

    class control:
        control_type =  'P' #'P' # P: position, V: velocity, T: torques 'CPG'
        if JOINT_PD:
            control_type = 'P'

        CPG_TEST = False
        cpg_body_height = 0.3
        cpg_ground_clearance = 0.05
        cpg_save_dir = '/data/tmp'
        
        # PD Drive parameters:
        stiffness = {'joint_a': 10.0, 'joint_b': 15.}  # [N*m/rad]
        damping = {'joint_a': 1.0, 'joint_b': 1.5}     # [N*m*s/rad]
        # action scale: target angle = actionScale * action + defaultAngle
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = 1 #10 #4
        action_scale = 0.25

        # whether to sample variable PD gains during learning, from the following ranges
        variable_gains = True 
        if not ADD_NOISE:
            variable_gains = False
        variable_stiffness = [55, 100]
        variable_damping = [1, 2.5]

        if EXTRA_HARD:
            variable_stiffness = [25, 100]
            variable_damping = [0.3, 2.]
        
        stiffness = {'joint': 100.}  # [N*m/rad]
        damping = {'joint': 2.}     # [N*m*s/rad]
        # stiffness = {'joint': 55.}  # [N*m/rad]
        # damping = {'joint': 0.8}     # [N*m*s/rad]

        if JOINT_PD:
            stiffness = {'joint': 55.}   # [N*m/rad]
            damping = {'joint': 0.8}     # [N*m*s/rad]

    class asset:
        disable_gravity = False
        collapse_fixed_joints = True # merge bodies connected by fixed joints. Specific fixed joints can be kept by adding " <... dont_collapse="true">
        fix_base_link = True # fixe the base of the robot
        default_dof_drive_mode = 3 # see GymDofDriveModeFlags (0 is none, 1 is pos tgt, 2 is vel tgt, 3 effort)
        #self_collisions = 0 # 1 to disable, 0 to enable...bitwise filter
        replace_cylinder_with_capsule = True # replace collision cylinders with capsules, leads to faster/more stable simulation
        flip_visual_attachments = True # Some .obj meshes must be flipped from y-up to z-up
        
        density = 0.001
        angular_damping = 0.
        linear_damping = 0.
        max_angular_velocity = 1000.
        max_linear_velocity = 1000.
        armature = 0.01  # SUPER IMPORTANT
        thickness = 0.01


        robot_name = "GO1"
        hip_y = 0.
        file = ""

        if robot_name == "A1":
            hip_y = 0.0838
            file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/a1/urdf/a1_updated_v2.urdf'  
        elif robot_name == "GO1":
            hip_y = 0.08 
            file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/go1/urdf/go1_newLimits_FIXORDER.urdf' # fix thigh limits AND link order 
        else:
            raise ValueError("******* incorrect robot_name *******")



        foot_name = "foot"
        # penalize_contacts_on = ["thigh", "calf"]
        penalize_contacts_on = ["base", "hip", "thigh", "calf"]
        terminate_after_contacts_on = []
        self_collisions = 1 # [TODO] test - should this be 1 to disable, 0 to enable...bitwise filter (opposite?)

    class domain_rand:
        randomize_friction = True 
        # on ground planes the friction combination mode is averaging, i.e total friction = (foot_friction + 1.)/2.
        friction_range = [-0.4, 1.0]

        randomize_base_mass = True
        added_mass_hip_range = [0., 10.] # not being used
        added_mass_range = [0., 10.] 

        push_robots = True 
        push_interval_s = 15
        max_push_vel_xy = 0.5

        motor_latency = True # True
        motor_latency_factor_min = 0.4 # test a bit less bad

        if EXTRA_HARD:
            motor_latency_factor_min = 0.3

        if not ADD_NOISE:
            randomize_friction = False
            randomize_base_mass = False
            motor_latency = False

        friction_range = [-0.7, 1.5]
        added_mass_range = [0., 8.] 
        

    class rewards:
        class scales:
            displacement = 0.1
            reached_target = 1.0

 
        only_positive_rewards = True # if true negative total rewards are clipped at zero (avoids early termination problems)
        tracking_sigma = 0.25 # tracking reward = exp(-error^2/sigma)
        tracking_sigma_vx = 2
        soft_dof_vel_limit = 0.
        soft_torque_limit = 0.
        max_contact_force = 1000. # forces above this value are penalized

        soft_dof_pos_limit = 0.9
        # base_height_target = 0.3
            

    class normalization:
        class obs_scales:
            lin_vel = 2.0
            ang_vel = 1 
            dof_pos = 1.0
            dof_vel = 0.05
            # height_measurements = 5.0
            height_measurements = 2.0   
            box_pos = 1.0
        clip_observations = 100.
        clip_actions = 4.

    class noise:
        add_noise = True
        if not ADD_NOISE:
            add_noise = False
        noise_level = 1.0 # scales other values
        class noise_scales:
            height_measurements = 0.1

            dof_pos = 0.01
            dof_vel = 0.05
            lin_vel = 0.05
            ang_vel = 0.1
            gravity = 0.05
            box_poses = 0.001


    # viewer camera:
    class viewer:
        ref_env = 0
        pos = [10, 3, 6]  # [m]
        lookat = [8, 4, 0.]  # [m]


    class sim:
        dt = 0.005 # 0.001 #0.005
        substeps = 3
        gravity = [0., 0. ,-9.81]  # [m/s^2]
        up_axis = 1  # 0 is y, 1 is z

        class physx:
            num_threads = 10
            solver_type = 1  # 0: pgs, 1: tgs
            num_position_iterations = 10
            num_velocity_iterations = 8
            contact_offset = 0.01  # [m]
            rest_offset = 0.0   # [m]
            bounce_threshold_velocity = 0.00001 #0.5 [m/s] 
            max_depenetration_velocity = 0.00001 # 
            max_gpu_contact_pairs = 2**23 #2**24 -> needed for 8000 envs and more
            default_buffer_size_multiplier = 5
            contact_collection = 2 # 0: never, 1: last sub-step, 2: all sub-steps (default=2)


class LeggedRobotCfgPPO(BaseConfig):
    seed = -1
    runner_class_name = 'OnPolicyRunner'
    class policy:
        init_noise_std = 1.0
        actor_hidden_dims = [512, 256, 128]
        critic_hidden_dims = [512, 256, 128]
        activation = 'elu' # can be elu, relu, selu, crelu, lrelu, tanh, sigmoid


    class algorithm:
        # training params
        value_loss_coef = 1.0
        use_clipped_value_loss = True
        clip_param = 0.2
        entropy_coef = 0.01
        num_learning_epochs = 5
        num_mini_batches = 4 # mini batch size = num_envs*nsteps / nminibatches
        learning_rate = 1.e-3 #5.e-4
        schedule = 'adaptive' # could be adaptive, fixed
        gamma = 0.99
        lam = 0.95
        desired_kl = 0.01
        max_grad_norm = 1.

    class runner:
        policy_class_name = 'ActorCritic' # 'ActorCriticRecurrent'
        algorithm_class_name = 'PPO'
        num_steps_per_env = 24 # per iteration
        max_iterations = 1500 # number of policy updates


        # logging
        save_interval = 100 # check for potential saves every this many iterations
        experiment_name = 'quadruped' # 'quadruped' 
        run_name = ''
        # load and resume
        resume = False
        load_run = -1 # -1 = last run
        checkpoint = -1 # -1 = last saved model
        resume_path = None # updated from load_run and chkpt