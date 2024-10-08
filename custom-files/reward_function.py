# Reward function with waypoints hardcoded above the generic model

import math
import numpy as np
def reward_function(params):
    MAX_REWARD = 1e2
    MIN_REWARD = 1e-3
    DIRECTION_THRESHOLD = 5.0
    ABS_STEERING_THRESHOLD = 30
    FUTURE_STEP = 5





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
    # x = params['x']
    # y = params['y']
    is_offtrack = params['is_offtrack']

    # negative exponential penalty
    reward = math.exp(-4)


    arr =np.array([[-0.19537496, -5.38186735],
       [-0.45890971, -5.38681829],
       [-0.72390234, -5.3906617 ],
       [-0.99095257, -5.39363344],
       [-1.26022691, -5.39598897],
       [-1.53218563, -5.39785969],
       [-1.80632559, -5.39946928],
       [-2.08222086, -5.40090676],
       [-2.36103503, -5.40211324],
       [-2.64333861, -5.40310637],
       [-2.9300637 , -5.40388146],
       [-3.22218907, -5.40444136],
       [-3.52370858, -5.40475035],
       [-3.82522857, -5.40512204],
       [-4.12405136, -5.39989991],
       [-4.41670017, -5.38337332],
       [-4.69945046, -5.35080927],
       [-4.96832833, -5.29857627],
       [-5.21913081, -5.22414554],
       [-5.4474827 , -5.12614047],
       [-5.64893788, -5.00441844],
       [-5.81915974, -4.86013931],
       [-5.95431988, -4.6958711 ],
       [-6.03140996, -4.51083142],
       [-6.06170913, -4.31864766],
       [-6.05139489, -4.12577762],
       [-6.00451043, -3.93643262],
       [-5.91556301, -3.75647035],
       [-5.79061455, -3.58936519],
       [-5.63353812, -3.43753664],
       [-5.44754231, -3.30289838],
       [-5.235754  , -3.18686429],
       [-5.00161824, -3.09009059],
       [-4.74926866, -3.01182769],
       [-4.48257839, -2.95049869],
       [-4.20489021, -2.90406506],
       [-3.9190514 , -2.87014878],
       [-3.62735128, -2.84634161],
       [-3.3316564 , -2.83018232],
       [-3.03316851, -2.81990923],
       [-2.73294215, -2.81365241],
       [-2.43178308, -2.80972993],
       [-2.13027906, -2.80671251],
       [-1.82877499, -2.80369198],
       [-1.52727103, -2.80067146],
       [-1.22592176, -2.7973264 ],
       [-0.92528293, -2.79248691],
       [-0.62599979, -2.78486299],
       [-0.3288813 , -2.77294237],
       [-0.03477949, -2.75526469],
       [ 0.25528767, -2.73021268],
       [ 0.54017175, -2.69612499],
       [ 0.81857645, -2.65130621],
       [ 1.08896267, -2.59392472],
       [ 1.34938041, -2.52185498],
       [ 1.59752991, -2.4329151 ],
       [ 1.83057446, -2.32477165],
       [ 2.04406462, -2.19403267],
       [ 2.23196217, -2.03725096],
       [ 2.38304713, -1.84995796],
       [ 2.50138837, -1.64165772],
       [ 2.58956997, -1.41832077],
       [ 2.65039025, -1.18468549],
       [ 2.68589191, -0.94424828],
       [ 2.69766307, -0.69976604],
       [ 2.68753705, -0.45352437],
       [ 2.65758253, -0.20731687],
       [ 2.60769706,  0.03723345],
       [ 2.5369961 ,  0.27813429],
       [ 2.44366499,  0.51249066],
       [ 2.32493877,  0.73531461],
       [ 2.18816562,  0.94662396],
       [ 2.0356956 ,  1.14533331],
       [ 1.86880127,  1.32969999],
       [ 1.68848343,  1.49748348],
       [ 1.49600022,  1.64646613],
       [ 1.29178868,  1.77210166],
       [ 1.07720306,  1.86942058],
       [ 0.85415799,  1.93072911],
       [ 0.62633818,  1.94678494],
       [ 0.40002109,  1.90434135],
       [ 0.17858495,  1.82052476],
       [-0.03804416,  1.70538236],
       [-0.25104188,  1.56799484],
       [-0.46210573,  1.41881252],
       [-0.69399506,  1.26100344],
       [-0.92933846,  1.10747075],
       [-1.16779357,  0.95854454],
       [-1.40925263,  0.81476007],
       [-1.65363502,  0.67669927],
       [-1.90090825,  0.54509006],
       [-2.15096075,  0.42055478],
       [-2.40372179,  0.30393637],
       [-2.65905271,  0.19606077],
       [-2.91668619,  0.09746635],
       [-3.17626147,  0.00836784],
       [-3.43736136, -0.07135554],
       [-3.69963668, -0.14131845],
       [-3.96270543, -0.20083009],
       [-4.22609251, -0.24883592],
       [-4.48918005, -0.28423557],
       [-4.75103449, -0.30460124],
       [-5.01021642, -0.30733848],
       [-5.26452102, -0.2892688 ],
       [-5.51058645, -0.24662561],
       [-5.74299782, -0.17469892],
       [-5.95419546, -0.06973428],
       [-6.13413249,  0.070502  ],
       [-6.26968049,  0.24540168],
       [-6.34319054,  0.44782054],
       [-6.36620049,  0.65952812],
       [-6.34874508,  0.87246718],
       [-6.29713982,  1.0826921 ],
       [-6.21566644,  1.28785487],
       [-6.10845384,  1.48670609],
       [-5.9760392 ,  1.67758713],
       [-5.81721407,  1.85795987],
       [-5.63602512,  2.02757829],
       [-5.43635683,  2.18704876],
       [-5.22185925,  2.33757497],
       [-4.99627486,  2.48095791],
       [-4.76347274,  2.6194868 ],
       [-4.52703441,  2.75551224],
       [-4.27691518,  2.90271723],
       [-4.02789852,  3.05187591],
       [-3.77967397,  3.20243796],
       [-3.53195596,  3.35389737],
       [-3.2844912 ,  3.50580541],
       [-3.03874804,  3.65534533],
       [-2.79347271,  3.80258104],
       [-2.54849742,  3.94662141],
       [-2.30352041,  4.08669364],
       [-2.05817971,  4.22200108],
       [-1.81206612,  4.35163219],
       [-1.56471828,  4.47449542],
       [-1.31562008,  4.58927517],
       [-1.06418992,  4.69433637],
       [-0.80962004,  4.78703994],
       [-0.55094829,  4.86395934],
       [-0.28704258,  4.9207698 ],
       [-0.01972384,  4.96462343],
       [ 0.2504935 ,  4.99782231],
       [ 0.523213  ,  5.02230917],
       [ 0.79807351,  5.03990667],
       [ 1.07485238,  5.05196177],
       [ 1.35338394,  5.05964137],
       [ 1.63363141,  5.06383029],
       [ 1.91557777,  5.06542962],
       [ 2.19739985,  5.06488166],
       [ 2.47558381,  5.06181371],
       [ 2.74945486,  5.05425898],
       [ 3.01870372,  5.04039202],
       [ 3.28295619,  5.01844304],
       [ 3.54166197,  4.98662624],
       [ 3.79403379,  4.94306499],
       [ 4.03891606,  4.88556987],
       [ 4.2746805 ,  4.81153819],
       [ 4.49916121,  4.71804912],
       [ 4.7092993 ,  4.60158504],
       [ 4.89962214,  4.45652341],
       [ 5.06109461,  4.27614517],
       [ 5.19795299,  4.07042154],
       [ 5.31217383,  3.84411026],
       [ 5.40600569,  3.60105433],
       [ 5.48235969,  3.34482698],
       [ 5.54378351,  3.07813616],
       [ 5.59343357,  2.80366966],
       [ 5.63464855,  2.52386967],
       [ 5.67004914,  2.24050476],
       [ 5.70311045,  1.9497769 ],
       [ 5.7391356 ,  1.65778684],
       [ 5.77769227,  1.36547089],
       [ 5.8184089 ,  1.0732107 ],
       [ 5.86105769,  0.7811752 ],
       [ 5.90550484,  0.48945702],
       [ 5.95165466,  0.19811167],
       [ 5.99942267, -0.09283091],
       [ 6.04890041, -0.38331034],
       [ 6.10019725, -0.67326664],
       [ 6.15348267, -0.96262589],
       [ 6.20905746, -1.25127073],
       [ 6.26368276, -1.52057315],
       [ 6.31347198, -1.78841978],
       [ 6.35350173, -2.05371008],
       [ 6.3792324 , -2.31591121],
       [ 6.38590576, -2.57441671],
       [ 6.36787007, -2.82788778],
       [ 6.31910002, -3.07411035],
       [ 6.22501412, -3.30596957],
       [ 6.09532497, -3.5219633 ],
       [ 5.93711125, -3.72149982],
       [ 5.75605299, -3.90482891],
       [ 5.55643519, -4.07257566],
       [ 5.34162507, -4.22563254],
       [ 5.11461554, -4.36538668],
       [ 4.87736012, -4.4927089 ],
       [ 4.63145787, -4.60839686],
       [ 4.3782295 , -4.71313484],
       [ 4.11882297, -4.80753511],
       [ 3.85428952, -4.89217681],
       [ 3.58562733, -4.96762744],
       [ 3.31380312, -5.03445422],
       [ 3.03975819, -5.09323089],
       [ 2.76440199, -5.14454155],
       [ 2.4885946 , -5.18898103],
       [ 2.21312068, -5.22715151],
       [ 1.93865799, -5.25965596],
       [ 1.66574442, -5.28708954],
       [ 1.39474758, -5.31003044],
       [ 1.12584086, -5.32903187],
       [ 0.85898986, -5.3446159 ],
       [ 0.5939527 , -5.3572692 ],
       [ 0.33029662, -5.3674397 ],
       [ 0.06743198, -5.37547853],
       [-0.19537496, -5.38186735]])
       
       
    ########################
    ### Helper functions ###
    ########################

    ########################
    ### Reward functions ###
    ########################

    # def distance_from_center_reward(current_reward, track_width, distance_from_center):
    #     # Calculate 3 marks that are farther and father away from the center line
    #     marker_1 = 0.1 * track_width
    #     marker_2 = 0.25 * track_width
    #     marker_3 = 0.4 * track_width

    #     # Give higher reward if the car is closer to center line and vice versa
    #     if distance_from_center <= marker_1:
    #         current_reward += 1.0
    #     elif distance_from_center <= marker_2:
    #         current_reward += 0.5
    #     elif distance_from_center <= marker_3:
    #         current_reward += 0.1
    #     else:
    #         current_reward = MIN_REWARD  # likely crashed/ close to off track

    #     return current_reward

    def follow_waypoint_reward(current_reward, arr, waypoints, closest_waypoints):
        next_waypoint = waypoints[closest_waypoints[1]]
        if (next_waypoint in arr) :
           current_reward += MAX_REWARD
        else :
           current_reward += MIN_REWARD
        return current_reward
    
    def direction_reward(current_reward, waypoints, closest_waypoints, heading):

        '''
        Calculate the direction of the center line based on the closest waypoints
        
        '''

        next_point = waypoints[closest_waypoints[1]]
        prev_point = waypoints[closest_waypoints[0]]

        # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
       
        direction = calculate_direction(prev_point[0],prev_point[1],next_point[0],next_point[1])

        # Cacluate difference between track direction and car heading angle
        direction_diff = abs(direction - heading)

        if direction_diff > 180:
            direction_diff = 360 - direction_diff

        # Penalize if the difference is too large and if less, then reward if steering is less else penalize.
        if direction_diff > DIRECTION_THRESHOLD:
            current_reward *= 0.5
        else:
            if steering < 5:
                current_reward += 1.0
            else:
                current_reward *=0.5

        return current_reward
    
    def calculate_direction(x1, y1, x2, y2):

        direction = math.atan2(y2 - y1, x2 - x1)
        direction_degrees = math.degrees(direction)

        return direction_degrees
    
    # def future_direction_reward(current_reward, waypoints, closest_waypoints, x, y, heading, speed):
    #     # Get current and next waypoints
    #     next_wp_index = closest_waypoints[1]

    #     # calculate indices for three waypoints ahead
    #     waypoint_1_index = (next_wp_index + 1) % len(waypoints)
    #     waypoint_2_index = (next_wp_index + 2) % len(waypoints)
    #     waypoint_3_index = (next_wp_index + 3) % len(waypoints)

    #     # Get coordinates of three waypoints ahead
    #     waypoint_1 = waypoints[waypoint_1_index]
    #     waypoint_2 = waypoints[waypoint_2_index]
    #     waypoint_3 = waypoints[waypoint_3_index]

    #     # calculate directions from car's current position to three waypoints
    #     direction_to_waypoint_1 = calculate_direction(x, y, waypoint_1[0], waypoint_1[1])
    #     direction_to_waypoint_2 = calculate_direction(x, y, waypoint_2[0], waypoint_2[1])
    #     direction_to_waypoint_3 = calculate_direction(x, y, waypoint_3[0], waypoint_3[1])

    #     # calculate the average direction of three waypoints ahead
    #     average_direction = (direction_to_waypoint_1 + direction_to_waypoint_2 + direction_to_waypoint_3) / 3.0

    #     # calculate difference between car's heading and the average direction
    #     direction_diff = abs(average_direction - heading)

    #     if direction_diff > 180:
    #         direction_diff = 360 - direction_diff
        
    #     # reward or penalise based on alignment with the average direction
    #     if direction_diff < DIRECTION_THRESHOLD:
    #         current_reward+=speed
    #     elif direction_diff < 20:
    #         current_reward*=0.5
    #     else:
    #         current_reward = MIN_REWARD
    #     return current_reward



    def step_reward(current_reward, steps, progress, on_track):
        if is_offtrack == False and steps > 0 :
            current_reward = ((progress/steps)*100)+speed*3
        else :
            current_reward = MIN_REWARD
        return current_reward
    
    # def orientation_towards_next_waypoint_reward(current_reward, waypoints, closest_waypoints, heading, x, y):
    #     rabbit = [0,0]
    #     pointing = [0,0]
        
    #     rabbit = waypoints[closest_waypoints[1]]
    #     radius = math.hypot(x - rabbit[0], y - rabbit[1])

    #     pointing[0] = x + (radius * math.cos(heading))
    #     pointing[1] = y + (radius * math.sin(heading))

    #     vector_delta = math.hypot(pointing[0] - rabbit[0], pointing[1] - rabbit[1])

    #     if vector_delta == 0:
    #         current_reward+= 1
    #     else:
    #         current_reward += (1- (vector_delta / (radius * 2)))
        
    #     return current_reward  
    
    def penalize_if_offtrack(current_reward, is_offtrack):
        if is_offtrack:
            current_reward = MIN_REWARD
        return current_reward
    
    # def reward_corner_approach_from_center_line(current_reward, waypoints, closest_waypoints, distance_from_center, track_width):
    #     next_point = waypoints[closest_waypoints[1]]
    #     prev_point = waypoints[closest_waypoints[0]]

    #     # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
       
    #     track_direction_current = calculate_direction(prev_point[0],prev_point[1],next_point[0],next_point[1])

    #     future_point_index = min(closest_waypoints[1] + FUTURE_STEP, len(waypoints) - 1)

    #     future_point = waypoints[future_point_index]

    #     track_direction_future = calculate_direction[next_point[0], next_point[1], future_point[0], future_point[1]]

    #     direction_change = abs(track_direction_future - track_direction_current)

    #     is_corner_approaching = direction_change > 15.0

    #     if is_corner_approaching:
    #         if distance_from_center <= 0.1 * track_width:
    #             current_reward = 2.0
    #         elif distance_from_center <= 0.25 * track_width:
    #             current_reward = 1.0
    #         else:
    #             current_reward = 0.5
    
    #     return current_reward
     



    ########################
    ### Execute Rewards  ###
    ########################
    # reward = distance_from_center_reward(reward, track_width, distance_from_center)
    reward = direction_reward(reward, waypoints, closest_waypoints, heading)
    reward = follow_waypoint_reward(reward, arr, waypoints, closest_waypoints)
    # reward = future_direction_reward(reward,waypoints,closest_waypoints,x,y,heading,speed)
    reward = step_reward(reward, steps, progress, on_track)
    # reward = orientation_towards_next_waypoint_reward(reward, waypoints, closest_waypoints, heading, x, y)
    reward = penalize_if_offtrack(reward, is_offtrack)
    # reward = reward_corner_approach_from_center_line(reward, waypoints, closest_waypoints, distance_from_center, track_width)
    return float(reward)
