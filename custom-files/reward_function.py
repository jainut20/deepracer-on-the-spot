def reward_function(params):
    '''
    Example of reward function using centre line and progress
    '''

    # Read input parameters
    track_width = params['track_width']
    distance_from_center = params['distance_from_center']
    progress = params['progress']
    steps = params['steps']
    speed = params['speed']
    all_wheels_on_track = params['all_wheels_on_track']
    is_offtrack = params['is_offtrack']
    waypoints = params['waypoints']
    closest_waypoints = params['closest_waypoints']
    heading = params['heading']

    # Reward for staying in the center of the track
    reward = 1.0
    marker_1 = 0.1 * track_width
    marker_2 = 0.25 * track_width
    marker_3 = 0.5 * track_width

    if distance_from_center <= marker_1:
        reward = 1.0
    elif distance_from_center <= marker_2:
        reward = 0.5
    elif distance_from_center <= marker_3:
        reward = 0.1
    else:
        reward = 1e-3  # likely crashed/ close to off track

    # Higher reward if the car completes a lap faster
    if progress == 100:
        reward += 10.0

    # Penalise if the car goes off-track
    if not all_wheels_on_track:
        reward = 1e-3

    # Reward for speed
    speed_threshold = 1.0
    if speed > speed_threshold:
        reward += 1.0

    # Penalise if the car is off-track
    if is_offtrack:
        reward = 1e-3

    return float(reward)
