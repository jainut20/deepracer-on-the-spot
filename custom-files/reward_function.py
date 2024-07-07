import math

MAX_REWARD = 1e2
MIN_REWARD = 1e-3
DIRECTION_THRESHOLD = 10.0
ABS_STEERING_THRESHOLD = 15
    
def reward_function(params):
    '''
    Optimized reward function for AWS DeepRacer
    '''
    
    # Read input parameters
    all_wheels_on_track = params['all_wheels_on_track']
    distance_from_center = params['distance_from_center']
    track_width = params['track_width']
    steering = abs(params['steering_angle'])  # Only need the absolute steering angle
    speed = params['speed']
    waypoints = params['waypoints']
    closest_waypoints = params['closest_waypoints']
    heading = params['heading']
    steps = params['steps']
    progress = params['progress']
    x = params['x']
    y = params['y']

    # Initial reward
    reward = 1.0

    ########################
    ### Execute Rewards  ###
    ########################
    reward = distance_from_center_reward(reward, track_width, distance_from_center)
    reward = direction_reward(reward, waypoints, closest_waypoints, heading)
    reward = step_reward(reward, steps, progress, all_wheels_on_track)
    reward = speed_reward(reward, speed, steering)
    
    # Ensure reward is within the range
    # reward = max(MIN_REWARD, min(MAX_REWARD, reward))

    return float(reward)

def distance_from_center_reward(current_reward, track_width, distance_from_center):
    # Calculate 3 marks that are farther and farther away from the center line
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

    # Calculate the direction in radians, arctan2(dy, dx), the result is (-pi, pi) in radians
    track_direction = math.atan2(next_point[1] - prev_point[1], next_point[0] - prev_point[0])
    # Convert to degrees
    track_direction = math.degrees(track_direction)

    # Calculate the difference between track direction and car heading angle
    direction_diff = abs(track_direction - heading)

    # Penalize if the difference is too large
    if direction_diff > DIRECTION_THRESHOLD:
        current_reward *= 0.5

    return current_reward

def step_reward(current_reward, steps, progress, all_wheels_on_track):
    if all_wheels_on_track and steps > 0:
        current_reward += (progress / steps) * 100
    else:
        current_reward = MIN_REWARD
    return current_reward

def speed_reward(current_reward, speed, steering):
    # Encourage higher speeds but penalize for too high steering angles
    if speed > 1.0 and steering < ABS_STEERING_THRESHOLD:
        current_reward += speed
    else:
        current_reward *= 0.8
    return current_reward