import numpy as np
from controller import Supervisor, Motion
import math
import time

# Initialize Supervisor
supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())  # Convert timestep to seconds

# Get robot and ball nodes by name
striker = supervisor.getFromDef("PLAYER_1_0")
ball = supervisor.getFromDef("BALL")

# Get translation and rotation fields
trans_field_striker = striker.getField("translation")
rot_field_striker = striker.getField("rotation")
trans_field_ball = ball.getField("translation")

# Initialize motion files for walking, turning, and kicking
forward_motion = Motion('/home/shah/Webots/NAO6/motions/Forwards50.motion')
kick_motion = Motion('/home/shah/Webots/NAO6/motions/Shoot.motion')
turn_left_motion = Motion('/home/shah/Webots/NAO6/motions/TurnLeft40.motion')
turn_right_motion = Motion('/home/shah/Webots/NAO6/motions/TurnRight40.motion')

# Define thresholds and initialize tracking variables
distance_threshold = 0.2  # Distance to trigger kick action
angle_tolerance = 0.3       # Tolerance for alignment (radians)

prev_pos = np.array(trans_field_striker.getSFVec3f())  # Track previous position

# Flag to track whether kick motion was triggered
kick_triggered = False

# Function to convert angle to fraction of pi and limit to 3 decimals
def angle_to_pi_fraction(angle):
    return round(angle / math.pi, 3)

# Function to limit distances to 4 decimal places
def limit_to_4_decimal_places(value):
    return round(value, 4)

# Main loop
while supervisor.step(int(timestep)) != -1:
    # Get current positions and orientation
    pos_striker = np.array(trans_field_striker.getSFVec3f())
    pos_ball = np.array(trans_field_ball.getSFVec3f())
    rot_striker = rot_field_striker.getSFRotation()

    # Calculate distance to ball
    current_distance = np.sqrt((pos_striker[0] - pos_ball[0]) ** 2 + (pos_striker[1] - pos_ball[1]) ** 2)
    
    striker_heading = (rot_striker[3] + math.pi) % (2 * math.pi) - math.pi  # Assuming rotation around Z-axis
    
    # Calculate the vector from Striker to Ball in the X-Y plane
    vector_to_target = [pos_ball[0] - pos_striker[0], pos_ball[1] - pos_striker[1]]
    
    # Calculate the angle to the target in the X-Y plane
    angle_to_target = math.atan2(vector_to_target[1], vector_to_target[0])
    
    # Calculate the angle difference for Robot to face the Ball
    angle_difference = angle_to_target - striker_heading
    angle_difference = (angle_difference + math.pi) % (2 * math.pi) - math.pi  # Normalize to [-π, π]
    
    # Decide on the direction to turn based on the angle difference
    if abs(angle_difference) > angle_tolerance:
        if angle_difference > 0:
            # Turn left
            turn_left_motion.play()
            if abs(angle_difference) <= angle_tolerance:
                turn_left_motion.stop()
                time.sleep(0.001)
        else:
            # Turn right
            turn_right_motion.play()
            if abs(angle_difference) <= angle_tolerance:
                turn_right_motion.stop()
                time.sleep(0.001)
    else:
        # Stop turning when facing ball within tolerance
        turn_left_motion.stop()
        time.sleep(0.001)
        turn_right_motion.stop()
        time.sleep(0.001)
        forward_motion.play()
        
        if current_distance <= distance_threshold and not kick_triggered:
            # Play kick motion only if it hasn't been triggered before
            forward_motion.stop()
            kick_motion.play()
            kick_triggered = True  # Set the flag to true to prevent further kicks

        # Reset condition: when ball is farther away than threshold, allow kick again
        if current_distance > distance_threshold:
            kick_triggered = False  # Allow kicking again when ball is far enough

    # Print debugging information in terms of pi with 3 decimal places for angles
    # and 4 decimal places for distances
    print(f"Striker Position: {pos_striker}")
    print(f"Striker Heading: {angle_to_pi_fraction(striker_heading)}π")
    print(f"Ball Position: {pos_ball}")
    print(f"Distance to Ball: {limit_to_4_decimal_places(current_distance)}")
    print(f"Angle to Ball: {angle_to_pi_fraction(angle_to_target)}π")
    print(f"Angle difference: {angle_to_pi_fraction(angle_difference)}π")

    # Check if the position changes
    if np.allclose(pos_striker, prev_pos):
        print("Striker position is not changing. Motion might not be playing.")
    prev_pos = pos_striker
    
    # Reset temporary variables
    current_distance = 0
    striker_heading = 0
    vector_to_target = 0
    angle_to_target = 0
    angle_difference = 0
