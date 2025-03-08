import rospy
from unitree_legged_msgs.msg import MotorCmd
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
import numpy as np

class SimpleRobotMover:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('simple_robot_mover')

        # Parameters
        self.robot_name = rospy.get_param('/robot_name')
        self.loop_rate = rospy.Rate(1000)  # 1000 Hz
        rospy.loginfo(f"Initialized {self.robot_name}")

        # Publishers for motor commands
        self.servo_pub = [
            rospy.Publisher(f'/{self.robot_name}_gazebo/FR_hip_controller/command', MotorCmd, queue_size=1),
            rospy.Publisher(f'/{self.robot_name}_gazebo/FR_thigh_controller/command', MotorCmd, queue_size=1),
            rospy.Publisher(f'/{self.robot_name}_gazebo/FR_calf_controller/command', MotorCmd, queue_size=1),
            rospy.Publisher(f'/{self.robot_name}_gazebo/FL_hip_controller/command', MotorCmd, queue_size=1),
            rospy.Publisher(f'/{self.robot_name}_gazebo/FL_thigh_controller/command', MotorCmd, queue_size=1),
            rospy.Publisher(f'/{self.robot_name}_gazebo/FL_calf_controller/command', MotorCmd, queue_size=1),
            rospy.Publisher(f'/{self.robot_name}_gazebo/RR_hip_controller/command', MotorCmd, queue_size=1),
            rospy.Publisher(f'/{self.robot_name}_gazebo/RR_thigh_controller/command', MotorCmd, queue_size=1),
            rospy.Publisher(f'/{self.robot_name}_gazebo/RR_calf_controller/command', MotorCmd, queue_size=1),
            rospy.Publisher(f'/{self.robot_name}_gazebo/RL_hip_controller/command', MotorCmd, queue_size=1),
            rospy.Publisher(f'/{self.robot_name}_gazebo/RL_thigh_controller/command', MotorCmd, queue_size=1),
            rospy.Publisher(f'/{self.robot_name}_gazebo/RL_calf_controller/command', MotorCmd, queue_size=1),
        ]

        # Service client for resetting the robot state
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_model_state_client = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        # Initialize robot state
        self.initialize_robot_state()

    def initialize_robot_state(self):
        # Create an instance of ModelState and set the model state (position and orientation)
        model_state = ModelState()
        model_state.model_name = f"{self.robot_name}_gazebo"
        model_state.pose.position.z = 0.5  # Reset height

        # Call the service
        self.set_model_state_client(model_state)
        rospy.sleep(1)  # Wait for state to settle

    def move_robot(self):
        counter = 0

        # Define two sets of motor commands (state A and state B)
        motor_commands_state_a = [
            (0.0, 0.0, 0.0, 70.0, 3.0),    # Motor 0
            (0.783037, 0.0, 0.0, 180.0, 8.0), # Motor 1
            (-1.566075, 0.0, 0.0, 300.0, 15.0), # Motor 2
            (0.0, 0.0, 0.0, 70.0, 3.0),    # Motor 3
            (0.783037, 0.0, 0.0, 180.0, 8.0), # Motor 4
            (-1.566075, 0.0, 0.0, 300.0, 15.0), # Motor 5
            (0.0, 0.0, 0.0, 70.0, 3.0),    # Motor 6
            (0.783037, 0.0, 0.0, 180.0, 8.0), # Motor 7
            (-1.566075, 0.0, 0.0, 300.0, 15.0), # Motor 8
            (0.0, 0.0, 0.0, 70.0, 3.0),    # Motor 9
            (0.783037, 0.0, 0.0, 180.0, 8.0), # Motor 10
            (-1.566075, 0.0, 0.0, 300.0, 15.0) # Motor 11
        ]

        motor_commands_state_b = [
            (0.5, 0.0, 0.0, 70.0, 3.0),    # Motor 0
            (0.783037, 0.0, 0.0, 180.0, 8.0), # Motor 1
            (-1.0, 0.0, 0.0, 300.0, 15.0), # Motor 2
            (0.5, 0.0, 0.0, 70.0, 3.0),    # Motor 3
            (0.783037, 0.0, 0.0, 180.0, 8.0), # Motor 4
            (-1.0, 0.0, 0.0, 300.0, 15.0), # Motor 5
            (0.5, 0.0, 0.0, 70.0, 3.0),    # Motor 6
            (0.783037, 0.0, 0.0, 180.0, 8.0), # Motor 7
            (-1.0, 0.0, 0.0, 300.0, 15.0), # Motor 8
            (0.5, 0.0, 0.0, 70.0, 3.0),    # Motor 9
            (0.783037, 0.0, 0.0, 180.0, 8.0), # Motor 10
            (-1.0, 0.0, 0.0, 300.0, 15.0)  # Motor 11
        ]

        while not rospy.is_shutdown() and counter < 10000:
            # Create a new MotorCmd message for each iteration
            cmd_msg = MotorCmd()
            cmd_msg.mode = 0x0A  # Control mode

            # Alternate between state A and state B every 1000 iterations
            if counter % 20 < 10:
                motor_commands = motor_commands_state_a
            else:
                motor_commands = motor_commands_state_b

            # Publish commands for each motor
            for i, (q, dq, tau, Kp, Kd) in enumerate(motor_commands):
                cmd_msg.q = float(q)
                cmd_msg.dq = float(dq)
                cmd_msg.tau = float(tau)
                cmd_msg.Kp = float(Kp)
                cmd_msg.Kd = float(Kd)

                # Publish the command for the specific motor
                rospy.loginfo(f"Publishing command to motor {i}: q={cmd_msg.q}, dq={cmd_msg.dq}, tau={cmd_msg.tau}, Kp={cmd_msg.Kp}, Kd={cmd_msg.Kd}")
                self.servo_pub[i].publish(cmd_msg)

            # self.loop_rate.sleep()
            rospy.sleep(0.5)
            counter += 1

        rospy.loginfo("Finished moving the robot")


if __name__ == '__main__':
    mover = SimpleRobotMover()
    mover.move_robot()
