#!/usr/bin/python3

# file to import in the other files

###############################################################################
###############################################################################
###############################################################################
###############################################################################

SIMULATOR_FLAG = True

# BRAIN
# ========================= STATES ==========================
START_STATE = 'start_state'
END_STATE = 'end_state'
DOING_NOTHING = 'doing_nothing'
LANE_FOLLOWING = 'lane_following'
APPROACHING_STOP_LINE = 'approaching_stop_line'
INTERSECTION_NAVIGATION = 'intersection_navigation'
TURNING_RIGHT = 'turning_right'
TURNING_LEFT = 'turning_left'
GOING_STRAIGHT = 'going_straight'
TRACKING_LOCAL_PATH = 'tracking_local_path'
ROUNDABOUT_NAVIGATION = 'roundabout_navigation'
WAITING_FOR_PEDESTRIAN = 'waiting_for_pedestrian'
WAITING_FOR_GREEN = 'waiting_for_green'
WAITING_AT_STOPLINE = 'waiting_at_stopline'
OVERTAKING_STATIC_CAR = 'overtaking_static_car'
OVERTAKING_MOVING_CAR = 'overtaking_moving_car'
TAILING_CAR = 'tailing_car'
AVOIDING_ROADBLOCK = 'avoiding_roadblock'
PARKING = 'parking'
CROSSWALK_NAVIGATION = 'crosswalk_navigation'
CLASSIFYING_OBSTACLE = 'classifying_obstacle'
BRAINLESS = 'brainless'

# ======================== ROUTINES ==========================
FOLLOW_LANE = 'follow_lane'
DETECT_STOP_LINE = 'detect_stop_line'
SLOW_DOWN = 'slow_down'
ACCELERATE = 'accelerate'
CONTROL_FOR_SIGNS = 'control_for_signs'
CONTROL_FOR_OBSTACLES = 'control_for_obstacles'
UPDATE_STATE = 'update_state'
DRIVE_DESIRED_SPEED = 'drive_desired_speed'

# ========================== EVENTS ==========================
INTERSECTION_STOP_EVENT = 'intersection_stop_event'
INTERSECTION_TRAFFIC_LIGHT_EVENT = 'intersection_traffic_light_event'
INTERSECTION_PRIORITY_EVENT = 'intersection_priority_event'
JUNCTION_EVENT = 'junction_event'
ROUNDABOUT_EVENT = 'roundabout_event'
CROSSWALK_EVENT = 'crosswalk_event'
PARKING_EVENT = 'parking_event'
END_EVENT = 'end_event'
HIGHWAY_EXIT_EVENT = 'highway_exit_event'

EVENT_TYPES = [INTERSECTION_STOP_EVENT,
               INTERSECTION_TRAFFIC_LIGHT_EVENT,
               INTERSECTION_PRIORITY_EVENT,
               JUNCTION_EVENT,
               ROUNDABOUT_EVENT,
               CROSSWALK_EVENT,
               PARKING_EVENT,
               HIGHWAY_EXIT_EVENT]


# ======================== ACHIEVEMENTS ========================
# consider adding all the tasks, may be too cumbersome
PARK_ACHIEVED = 'park_achieved'

# ======================== CONDITIONS ==========================
CAN_OVERTAKE = 'can_overtake'
HIGHWAY = 'highway'
TRUST_GPS = 'trust_gps'
CAR_ON_PATH = 'car_on_path'
REROUTING = 'rerouting'
BUMPY_ROAD = 'bumpy_road'


###############################################################################
###############################################################################
###############################################################################
###############################################################################


# DETECTION
# PARKING SIGNS
PARK = 'park'
CLOSED_ROAD = 'closed_road'
HW_EXIT = 'hw_exit'
HW_ENTER = 'hw_enter'
STOP = 'stop'
ROUNDABOUT = 'roundabout'
PRIORITY = 'priority'
CROSSWALK = 'cross_walk'
ONE_WAY = 'one_way'
NO_SIGN = 'NO_sign'
TRAFFIC_LIGHT = 'traffic_light'
SIGN_NAMES = [PARK,
              CLOSED_ROAD,
              HW_EXIT,
              HW_ENTER,
              STOP,
              ROUNDABOUT,
              PRIORITY,
              CROSSWALK,
              ONE_WAY,
              NO_SIGN]

# obstacles
CAR = 'car'
PEDESTRIAN = 'pedestrian'
ROADBLOCK = 'roadblock'

# ENVIROMENTAL SERVER
STATIC_CAR_ON_ROAD = 'static_car_on_road'
STATIC_CAR_PARKING = 'static_car_parking'
PEDESTRIAN_ON_CROSSWALK = 'pedestrian_on_crosswalk'
PEDESTRIAN_ON_ROAD = 'pedestrian_on_road'
ROADBLOCK = 'roadblock'
BUMPY_ROAD = 'bumpy_road'

# sempahores
MASTER = 'master'
SLAVE = 'slave'
ANTIMASTER = 'antimaster'
START = 'start'
# semaphore states
GREEN = 2
YELLOW = 1
RED = 0

###############################################################################
###############################################################################
###############################################################################
###############################################################################

# AUTOMOBILE DATA

# PARKING SUBSTATES
LOCALIZING_PARKING_SPOT = 1
CHECKING_FOR_PARKED_CARS = 2
STEP0 = 69
T_STEP2 = 4
T_STEP3 = 5
T_STEP4 = 6
T_STEP5 = 7
S_STEP2 = 9
S_STEP3 = 10
S_STEP4 = 11
S_STEP5 = 12
S_STEP6 = 13
S_STEP7 = 14
PARK_END = 16
# park types
T_PARK = 't'
S_PARK = 's'
