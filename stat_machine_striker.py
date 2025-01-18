### INITIAL CODE### (works very poorly)
# import numpy as np
# from controller import Supervisor, Motion
# import math
# import time

# class SoccerNaoForward(Supervisor):
#     def __init__(self):
#         super(SoccerNaoForward, self).__init__()

#         # Time step
#         self.timeStep = int(self.getBasicTimeStep())
#         self.fsm = ForwardFSM(self)  # Attach FSM

#         # Load motions
#         try:
#             # Load the forward motion file
#             self.forwards_50 = Motion("/home/shah/Webots/NAO6/motions/Forwards50.motion")
#             self.turn_left_40 = Motion("/home/shah/Webots/NAO6/motions/TurnLeft40.motion")
#             self.turn_right_40 = Motion("/home/shah/Webots/NAO6/motions/TurnRight40.motion")
#             self.kick_motion = Motion("/home/shah/Webots/NAO6/motions/Shoot.motion")
#         except Exception as e:
#             print(f"Error loading motion files: {e}")
#             self.forwards_50 = None
#             self.turn_left_40 = None
#             self.turn_right_40 = None
#             self.kick_motion = None

#         # Supervisor-specific: Get ball object and robot (striker) object
#         self.striker = self.getFromDef("PLAYER_1_0")  # Get striker node
#         self.ball = self.getFromDef("BALL")  # Ball object in the world (Supervisor node)

#         # Get translation and rotation fields for striker (robot)
#         self.trans_field_striker = self.striker.getField("translation")
#         self.rot_field_striker = self.striker.getField("rotation")
#         self.trans_field_ball = self.ball.getField("translation")

#         # Cooldown settings
#         self.cooldown_time = 1.0  # Cooldown time in seconds (1 second)
#         self.last_transition_time = self.getTime()  # Track the last transition time

#     def move_forward(self):
#         """Moves the robot forward by playing the forward motion."""
#         if self.forwards_50:
#             self.forwards_50.play()

#     def stop(self):
#         """Stop all ongoing motions."""
#         if self.forwards_50:
#             self.forwards_50.stop()

#     def get_robot_position(self):
#         """Returns the current position of the robot (x, y)."""
#         return np.array(self.trans_field_striker.getSFVec3f())[:2]  # Use only x, y position

#     def get_ball_position(self):
#         """Returns the current position of the ball (x, y, z)."""
#         return np.array(self.trans_field_ball.getSFVec3f())[:2]  # Use only x, y position

#     def turn_towards(self, angle_difference):
#         """Turn robot left or right based on angle difference."""
#         if angle_difference > 0:
#             self.turn_left_40.play()
#         else:
#             self.turn_right_40.play()

#     def reset_temporary_variables(self):
#         """Reset temporary variables."""
#         self.temporary_vars = {
#             "current_distance": 0,
#             "striker_heading": 0,
#             "vector_to_target": 0,
#             "angle_to_target": 0,
#             "angle_difference": 0
#         }

#     def print_debug_info(self, pos_striker, pos_ball, striker_heading, angle_to_target, current_distance, angle_difference):
#         """Print debugging information."""
#         print(f"Striker Position: {pos_striker}")
#         print(f"Striker Heading: {angle_to_target} rad")
#         print(f"Ball Position: {pos_ball}")
#         print(f"Distance to Ball: {current_distance}")
#         print(f"Angle to Ball: {angle_to_target} rad")
#         print(f"Angle difference: {angle_difference} rad")


# class ForwardFSM:
#     def __init__(self, robot):
#         self.robot = robot
#         self.state = "S0"  # Initial state

#     def update(self):
#         """Update the FSM based on conditions."""
#         current_time = self.robot.getTime()

#         # Ensure cooldown has passed before transitioning
#         if current_time - self.robot.last_transition_time < self.robot.cooldown_time:
#             return  # Wait for cooldown

#         # Get current positions and heading of robot and ball
#         pos_striker = np.array(self.robot.get_robot_position())
#         pos_ball = np.array(self.robot.get_ball_position())

#         # Calculate distance and angle to the ball
#         current_distance = np.linalg.norm(pos_ball[:2] - pos_striker[:2])  # 2D distance
#         vector_to_target = pos_ball[:2] - pos_striker[:2]
#         angle_to_target = math.atan2(vector_to_target[1], vector_to_target[0])  # Angle to ball

#         # Get the current heading of the robot (based on its rotation)
#         rot_striker = self.robot.rot_field_striker.getSFRotation()  # Get rotation (quaternion)
#         striker_heading = rot_striker[3]  # Yaw (rotation around z-axis)

#         # Calculate the angle difference (the amount the robot needs to turn)
#         angle_difference = (angle_to_target - striker_heading + math.pi) % (2 * math.pi) - math.pi  # Normalize to [-π, π]

#         # Transition based on current state
#         if self.state == "S0":  # Starting state
#             print("Starting forward movement.")
#             self.robot.last_transition_time = current_time  # Reset cooldown
#             self.state = "S1"  # Transition to moving forward

#         elif self.state == "S1":  # Move forward state
#             print("Moving forward...")
#             self.robot.move_forward()  # Start moving forward
#             if abs(angle_difference) > 0.25:  # Turn if not aligned with ball
#                 self.robot.turn_towards(angle_difference)
#                 self.state = "S4"  # Transition to turn towards ball
#             elif current_distance <= 0.2:  # If close to the ball, stop and kick
#                 self.state = "S2"  # Transition to stop state

#         elif self.state == "S2":  # Stop state
#             print("Stopping.")
#             self.robot.stop()  # Stop moving
#             self.robot.last_transition_time = current_time  # Reset cooldown
#             self.state = "S3"  # Transition to kick state

#         elif self.state == "S3":  # Kick state
#             print("Preparing to kick.")
#             self.robot.kick_motion.play()  # Perform the kick
#             self.robot.last_transition_time = current_time  # Reset cooldown
#             self.state = "S0"  # Transition back to start state

#         elif self.state == "S4":  # Turn towards ball state
#             if abs(angle_difference) <= 0.25:  # If robot is aligned with the ball, stop turning
#                 self.robot.stop()
#                 self.state = "S1"  # Transition to moving forward state
#             else:
#                 self.robot.turn_towards(angle_difference)  # Keep turning towards ball

#     def transition(self, condition):
#         """Handle state transitions based on condition."""
#         if condition == "game_start":
#             self.state = "S2"  # Start in state S0 (Stop state)


###STATE MACHINE BASED ON CURRENT STATE### (works poorly)
# import numpy as np
# from controller import Supervisor, Motion
# import math
# import time

# class SoccerNaoForward(Supervisor):
#     def __init__(self):
#         super(SoccerNaoForward, self).__init__()

#         # Time step
#         self.timeStep = int(self.getBasicTimeStep())
#         self.fsm = ForwardFSM(self)  # Attach FSM

#         # Load motions
#         try:
#             self.forwards_50 = Motion("/home/shah/Webots/NAO6/motions/Forwards50.motion")
#             self.turn_left_40 = Motion("/home/shah/Webots/NAO6/motions/TurnLeft40.motion")
#             self.turn_right_40 = Motion("/home/shah/Webots/NAO6/motions/TurnRight40.motion")
#             self.kick_motion = Motion("/home/shah/Webots/NAO6/motions/Shoot.motion")
#         except Exception as e:
#             print(f"Error loading motion files: {e}")
#             self.forwards_50 = None
#             self.turn_left_40 = None
#             self.turn_right_40 = None
#             self.kick_motion = None

#         # Supervisor-specific: Get ball object and robot (striker) object
#         self.striker = self.getFromDef("PLAYER_1_0")  # Get striker node
#         self.ball = self.getFromDef("BALL")  # Ball object in the world (Supervisor node)

#         # Get translation and rotation fields for striker (robot)
#         self.trans_field_striker = self.striker.getField("translation")
#         self.rot_field_striker = self.striker.getField("rotation")
#         self.trans_field_ball = self.ball.getField("translation")

#         # Cooldown settings
#         self.cooldown_time = 1.0  # Cooldown time in seconds (1 second)
#         self.last_transition_time = self.getTime()  # Track the last transition time

#     def move_forward(self):
#         """Moves the robot forward by playing the forward motion."""
#         if self.forwards_50:
#             self.forwards_50.play()

#     def stop(self):
#         """Stop all ongoing motions."""
#         if self.forwards_50:
#             self.forwards_50.stop()

#     def get_robot_position(self):
#         """Returns the current position of the robot (x, y)."""
#         return np.array(self.trans_field_striker.getSFVec3f())[:2]  # Use only x, y position

#     def get_ball_position(self):
#         """Returns the current position of the ball (x, y, z)."""
#         return np.array(self.trans_field_ball.getSFVec3f())[:2]  # Use only x, y position

#     def turn_towards(self, angle_difference):
#         """Turn robot left or right based on angle difference."""
#         if angle_difference > 0:
#             self.turn_left_40.play()
#         else:
#             self.turn_right_40.play()

#     def reset_temporary_variables(self):
#         """Reset temporary variables."""
#         self.temporary_vars = {
#             "current_distance": 0,
#             "striker_heading": 0,
#             "vector_to_target": 0,
#             "angle_to_target": 0,
#             "angle_difference": 0
#         }

#     def print_debug_info(self, pos_striker, pos_ball, striker_heading, angle_to_target, current_distance, angle_difference):
#         """Print debugging information."""
#         print(f"Striker Position: {pos_striker}")
#         print(f"Striker Heading: {angle_to_target} rad")
#         print(f"Ball Position: {pos_ball}")
#         print(f"Distance to Ball: {current_distance}")
#         print(f"Angle to Ball: {angle_to_target} rad")
#         print(f"Angle difference: {angle_difference} rad")


# class ForwardFSM:
#     def __init__(self, robot):
#         self.robot = robot
#         self.state = "S0"  # Initial state

#     def update(self):
#         """Update the FSM based on conditions."""
#         current_time = self.robot.getTime()

#         # Ensure cooldown has passed before transitioning
#         if current_time - self.robot.last_transition_time < self.robot.cooldown_time:
#             return  # Wait for cooldown

#         # Get current positions and heading of robot and ball
#         pos_striker = np.array(self.robot.get_robot_position())
#         pos_ball = np.array(self.robot.get_ball_position())

#         # Calculate distance and angle to the ball
#         current_distance = np.linalg.norm(pos_ball[:2] - pos_striker[:2])  # 2D distance
#         vector_to_target = pos_ball[:2] - pos_striker[:2]
#         angle_to_target = math.atan2(vector_to_target[1], vector_to_target[0])  # Angle to ball

#         # Get the current heading of the robot (based on its rotation)
#         rot_striker = self.robot.rot_field_striker.getSFRotation()  # Get rotation (quaternion)
#         striker_heading = rot_striker[3]  # Yaw (rotation around z-axis)

#         # Calculate the angle difference (the amount the robot needs to turn)
#         angle_difference = (angle_to_target - striker_heading + math.pi) % (2 * math.pi) - math.pi  # Normalize to [-π, π]

#         # Transition based on current state
#         if self.state == "S0":  # Stop state (Initial state)
#             print("Starting forward movement.")
#             self.robot.last_transition_time = current_time  # Reset cooldown
#             self.state = "S1"  # Transition to turn towards the ball

#         elif self.state == "S1":  # Turn towards ball
#             print("Turning towards the ball...")
#             if abs(angle_difference) > 0.25:  # Turn if not aligned with ball
#                 self.robot.turn_towards(angle_difference)
#             else:
#                 self.state = "S2"  # Transition to moving forward once aligned

#         elif self.state == "S2":  # Move forward
#             print("Moving forward...")
#             self.robot.move_forward()  # Move towards the ball
#             if current_distance <= 0.2:  # If close to the ball, stop and proceed to small turn
#                 self.state = "S3"  # Transition to small turn around the ball

#         elif self.state == "S3":  # Small turn to face the goal
#             print("Turning towards the goal...")
#             # Calculate the angle difference to face the opponent's goal
#             goal_position = np.array([4.5, 0])  # Opponent's goal position (x, y)
#             goal_vector = goal_position - pos_striker[:2]
#             angle_to_goal = math.atan2(goal_vector[1], goal_vector[0])
#             goal_angle_diff = (angle_to_goal - striker_heading + math.pi) % (2 * math.pi) - math.pi  # Normalize

#             if abs(goal_angle_diff) > 0.25:
#                 self.robot.turn_towards(goal_angle_diff)  # Turn towards goal
#             else:
#                 self.state = "S4"  # Transition to kick state once aligned with the goal

#         elif self.state == "S4":  # Kicking state
#             print("Kicking the ball...")
#             self.robot.kick_motion.play()  # Perform the kick
#             self.robot.last_transition_time = current_time  # Reset cooldown
#             self.state = "S0"  # Transition back to stop state after kicking

#     def transition(self, condition):
#         """Handle state transitions based on condition."""
#         if condition == "game_start":
#             self.state = "S0"  # Start in state S0 (Stop state)



###STATE MACHINE BASED ON CONDITIONS###
import numpy as np
from controller import Supervisor, Motion
import math
import time

class SoccerNaoForward(Supervisor):
    def __init__(self):
        super(SoccerNaoForward, self).__init__()

        # Time step
        self.timeStep = int(self.getBasicTimeStep())
        self.fsm = ForwardFSM(self)  # Attach FSM

        # Load motions
        try:
            self.forwards_50 = Motion("/home/shah/Webots/NAO6/motions/Forwards50.motion")
            self.turn_left_40 = Motion("/home/shah/Webots/NAO6/motions/TurnLeft40.motion")
            self.turn_right_40 = Motion("/home/shah/Webots/NAO6/motions/TurnRight40.motion")
            self.kick_motion = Motion("/home/shah/Webots/NAO6/motions/Shoot.motion")
        except Exception as e:
            print(f"Error loading motion files: {e}")
            self.forwards_50 = None
            self.turn_left_40 = None
            self.turn_right_40 = None
            self.kick_motion = None

        # Supervisor-specific: Get ball object and robot (striker) object
        self.striker = self.getFromDef("PLAYER_1_0")  # Get striker node
        self.ball = self.getFromDef("BALL")  # Ball object in the world (Supervisor node)

        # Get translation and rotation fields for striker (robot)
        self.trans_field_striker = self.striker.getField("translation")
        self.rot_field_striker = self.striker.getField("rotation")
        self.trans_field_ball = self.ball.getField("translation")

        # Cooldown settings
        self.cooldown_time = 1  # Cooldown time in seconds (1 second)
        self.last_transition_time = self.getTime()  # Track the last transition time

    def move_forward(self):
        """Moves the robot forward by playing the forward motion."""
        if self.forwards_50:
            print("Robot is moving forward.")
            self.forwards_50.play()

    def stop(self):
        """Stop all ongoing motions."""
        if self.forwards_50:
            print("Robot stopped.")
            self.forwards_50.stop()
            self.turn_left_40.stop()
            self.turn_right_40.stop()
            self.kick_motion.stop()

    def get_robot_position(self):
        """Returns the current position of the robot (x, y)."""
        return np.array(self.trans_field_striker.getSFVec3f())[:2]  # Use only x, y position

    def get_ball_position(self):
        """Returns the current position of the ball (x, y, z)."""
        return np.array(self.trans_field_ball.getSFVec3f())[:2]  # Use only x, y position

    def turn_towards(self, angle_difference):
        """Turn robot left or right based on angle difference."""
        if angle_difference > 0:
            print(f"Turning left. Angle difference: {angle_difference} rad")
            self.turn_left_40.play()
        else:
            print(f"Turning right. Angle difference: {angle_difference} rad")
            self.turn_right_40.play()

class ForwardFSM:
    def __init__(self, robot):
        self.robot = robot
        self.state = "S0"  # Initial state

    def check_conditions(self, alignment_threshold=0.3, distance_threshold=0.25):
        """
        Check the following conditions:
        - Is the robot aligned with the ball (based on heading)?
        - How far is the robot from the ball?
        - How far is the robot from the goal?
        - Is the robot facing the opponent's goal?

        Returns:
            - aligned_with_ball: Boolean indicating if robot is aligned with the ball.
            - close_to_ball: Boolean indicating if robot is close enough to the ball.
            - close_to_goal: Boolean indicating if robot is close enough to the goal.
            - facing_goal: Boolean indicating if robot is facing the goal.
        """

        # Extract positions (x, y) for easier manipulation
        robot_x, robot_y = np.array(self.robot.get_robot_position())
        ball_x, ball_y = np.array(self.robot.get_ball_position())
        goal_x = 4.5
        goal_y = 0
        robot_rotation = self.robot.rot_field_striker.getSFRotation()
        robot_heading = robot_rotation[3]

        # Calculate the vector to the ball
        vector_to_ball = (ball_x - robot_x, ball_y - robot_y)
        # Angle to the ball
        angle_to_ball = math.atan2(vector_to_ball[1], vector_to_ball[0])

        # Calculate the angle difference between robot's heading and the angle to the ball
        self.angle_difference = (angle_to_ball - robot_heading + math.pi) % (2 * math.pi) - math.pi  # Normalize to [-π, π]

        # Check if robot is aligned with the ball (within a threshold angle)
        aligned_with_ball = abs(self.angle_difference) <= alignment_threshold

        # Calculate distance to the ball
        distance_to_ball = math.sqrt((ball_x - robot_x)**2 + (ball_y - robot_y)**2)

        # Check if robot is close enough to the ball
        close_to_ball = distance_to_ball <= distance_threshold

        # Calculate distance to the goal
        distance_to_goal = math.sqrt((goal_x - robot_x)**2 + (goal_y - robot_y)**2)

        # Check if robot is close enough to the goal (threshold can be adjusted)
        close_to_goal = distance_to_goal <= distance_threshold

        # Calculate angle to the goal
        vector_to_goal = (goal_x - robot_x, goal_y - robot_y)
        angle_to_goal = math.atan2(vector_to_goal[1], vector_to_goal[0])

        # Check if robot is facing the goal (within a threshold angle)
        self.goal_angle_difference = (angle_to_goal - robot_heading + math.pi) % (2 * math.pi) - math.pi  # Normalize to [-π, π]
        facing_goal = abs(self.goal_angle_difference) <= alignment_threshold

        print(f"aligned_with_ball: {aligned_with_ball}, close_to_ball: {close_to_ball}, close_to_goal: {close_to_goal}, facing_goal: {facing_goal}")

        return aligned_with_ball, close_to_ball, close_to_goal, facing_goal

    def determine_action(self,aligned_with_ball, close_to_ball, close_to_goal, facing_goal):
        """
        Determines the robot's action based on the combination of conditions.
        
        Parameters:
        - aligned_with_ball (bool): Whether the robot is aligned with the ball.
        - close_to_ball (bool): Whether the robot is close to the ball.
        - close_to_goal (bool): Whether the robot is close to the goal.
        - facing_goal (bool): Whether the robot is facing the goal.
        
        Returns:
            - action (str): The action the robot should take (e.g., 'move_forward', 'turn_to_ball', 'kick_ball', etc.)
        """

        if aligned_with_ball and facing_goal and close_to_ball and close_to_goal:
            return "kick"
        elif not aligned_with_ball:
            return "turn to ball"
        elif not close_to_ball:
            return "move forward"
        elif not facing_goal:
            return "turn to goal"
        elif not close_to_goal:
            return "move forward"

    def update(self):
        """Update the FSM based on conditions."""
        # current_time = self.robot.getTime()

        # # Ensure cooldown has passed before transitioning
        # if current_time - self.robot.last_transition_time < self.robot.cooldown_time:
        #     return  # Wait for cooldown

        # Transition based on conditions
        if self.state == "S0":  # Stop state (Initial state)
            print("Transitioning in S0: Stopping the robot.")
            self.robot.stop()  # Always stop before transitioning
            # self.robot.last_transition_time = current_time

        elif self.state == "S1":  # Turn towards ball
            print("Transitioning in S1: Turning towards the ball.")
            self.robot.stop()
            self.robot.turn_towards(self.angle_difference)  # Turn towards ball
            # self.robot.last_transition_time = current_time

        elif self.state == "S2":  # Move forward
            print("Transitioning in S2: Moving forward towards the ball.")
            self.robot.stop()
            self.robot.move_forward()  # Move towards the ball
            # self.robot.last_transition_time = current_time

        elif self.state == "S3":  # Small turn around the ball
            print("Transitioning in S3: Turning towards the goal.")  
            self.robot.stop()
            self.robot.turn_towards(self.goal_angle_difference)
            # self.robot.last_transition_time = current_time

        elif self.state == "S4":  # Kicking state
            print("Transitioning in S4: Kicking the ball.")
            self.robot.stop()
            self.robot.kick_motion.play()  # Perform the kick
            # self.robot.last_transition_time = current_time

    
    def transition(self, action):
        """Handle state transitions based on action required."""
        if action == "kick":
            self.state = "S4"  # Start in state S0 (Stop state)
        elif action == "turn to ball":
            self.state = "S1"
        elif action == "move forward":
            self.state = "S2"
        elif action == "turn to goal":
            self.state = "S3"
        else:
            print("Something went wrong... No action found!")
# ---------------------------------------------
# Main Loop
# ---------------------------------------------
# Instantiate the SoccerNaoForward object
robot = SoccerNaoForward()

# Main loop for the robot
while robot.step(robot.timeStep) != -1:
    # Update FSM (state machine)
    a, b, c, d = robot.fsm.check_conditions()
    action = robot.fsm.determine_action(aligned_with_ball=a,
                                        close_to_ball=b,
                                        close_to_goal=c,
                                        facing_goal=d)
    robot.fsm.transition(action)
    robot.fsm.update()