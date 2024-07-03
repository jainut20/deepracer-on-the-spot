import math
import numpy as np
def reward_function(params):
    MAX_REWARD = 1e2
    MIN_REWARD = 1e-3
    DIRECTION_THRESHOLD = 10.0
    ABS_STEERING_THRESHOLD = 30





    ########################
    ### Input parameters ###
    ########################
    on_track = params['all_wheels_on_track']
    distance_from_center = params['distance_from_center']
    track_width = params['track_width']
    steering = abs(params['steering_angle']) # Only need the absolute steering angle for calculations
    speed = params['speed']
    waypoints = params['waypoints']
    closest_waypoints = params['closest_waypoints']
    heading = params['heading']
    steps = params['steps']
    progress = params['progress']
    x = params['x']
    y = params['y']

    # negative exponential penalty
    reward = math.exp(-6 * distance_from_center)


    ########################
    ### Helper functions ###
    ########################

    ########################
    ### Reward functions ###
    ########################

    def distance_from_center_reward(current_reward, track_width, distance_from_center):
        # Calculate 3 marks that are farther and father away from the center line
        marker_1 = 0.1 * track_width
        marker_2 = 0.25 * track_width
        marker_3 = 0.4 * track_width

        # Give higher reward if the car is closer to center line and vice versa
        if distance_from_center <= marker_1:
            current_reward += 1.0
        elif distance_from_center <= marker_2:
            current_reward += 0.5
        elif distance_from_center <= marker_3:
            current_reward += 0.1
        else:
            current_reward = MIN_REWARD  # likely crashed/ close to off track

        return current_reward

    def direction_reward(current_reward, waypoints, closest_waypoints, heading):

        '''
        Calculate the direction of the center line based on the closest waypoints
        '''

        next_point = waypoints[closest_waypoints[1]]
        prev_point = waypoints[closest_waypoints[0]]

        # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
        direction = math.atan2(next_point[1] - prev_point[1], next_point[0] - prev_point[0])
        # Convert to degrees
        direction = math.degrees(direction)

        # Cacluate difference between track direction and car heading angle
        direction_diff = abs(direction - heading)

        # Penalize if the difference is too large
        if direction_diff > DIRECTION_THRESHOLD:
            current_reward *= 0.5

        return current_reward

    def step_reward(current_reward, steps, progress):
        if on_track and steps > 0 :
            current_reward = ((progress/steps)*100)+speed*2
        else :
            current_reward = MIN_REWARD
        return current_reward
    
    def orientation_towards_next_waypoint_reward(current_reward, waypoints, closest_waypoints, heading, x, y):
        rabbit = [0,0]
        pointing = [0,0]
        
        rabbit = waypoints[closest_waypoints[1]]
        radius = math.hypot(x - rabbit[0], y - rabbit[1])

        pointing[0] = x + (radius * math.cos(heading))
        pointing[1] = y + (radius * math.sin(heading))

        vector_delta = math.hypot(pointing[0] - rabbit[0], pointing[1] - rabbit[1])

        if vector_delta == 0:
            current_reward+= 1
        else:
            current_reward += (1- (vector_delta / (radius * 2)))
        
        return current_reward


    ########################
    ### Execute Rewards  ###
    ########################
    reward = distance_from_center_reward(reward, track_width, distance_from_center)
    reward = direction_reward(reward, waypoints, closest_waypoints, heading)
    reward = step_reward(reward, steps, progress)
    reward = orientation_towards_next_waypoint_reward(reward, waypoints, closest_waypoints, heading, x, y)
    
    return float(reward)
