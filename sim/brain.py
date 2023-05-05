#!/usr/bin/python3
from names_and_constants import SIMULATOR_FLAG, SPEED_CHALLENGE

import numpy as np
import cv2 as cv
from time import time, sleep
from numpy.linalg import norm
from collections import deque
import names_and_constants as nac

if not SIMULATOR_FLAG:
    from control.automobile_data_interface import Automobile_Data
else:
    from automobile_data_interface import Automobile_Data
from path_planning4 import PathPlanning
from controller3 import Controller
from controllerSP import ControllerSpeed
from controllerAG import ControllerSpeed as ControllerBL
from detection import Detection
from environmental_data_simulator import EnvironmentalData
from obstacle2 import Obstacle
import helper_functions as hf

from parkman import Maneuvers

SHOW_IMGS = False

# To start from the start node (86) use [-42, -42]
STARTING_COORDS = [-42, -42]

END_NODE = 85

CHECKPOINTS = [86, 430, 193, 141, 349, 85]  # complete track - old

CHECKPOINTS = [86, 430, 197, 112, 349, 113, 134, 145, END_NODE]  # La Tecnique
CHECKPOINTS = [90, 430, 197, 112, 349, 113, 134, 145, END_NODE]  # La Tecnique
CHECKPOINTS = [86, 430, 197, 123]  # Speed Challenge - easy
# CHECKPOINTS = [86, 430, 229, 265, 146, 121]  # Speed Challenge - hard

# CHECKPOINTS = [263, 184]
# CHECKPOINTS = [90, 430, 104]

ALWAYS_USE_VISION_FOR_STOPLINES = True

# GPS
ALWAYS_TRUST_GPS = False  # if true the car will always trust the gps (bypass)
# if true, the car will always distrust the gps (bypass)
ALWAYS_DISTRUST_GPS = True
assert not (ALWAYS_TRUST_GPS and ALWAYS_DISTRUST_GPS), \
        'ALWAYS_TRUST_GPS and ALWAYS_DISTRUST_GPS cannot be both True'

# Templates for obstacle detection
num_tem = 5
tem = []
tem.append(cv.imread("templates/car1.png"))
tem.append(cv.imread("templates/car2.png"))
tem.append(cv.imread("templates/car3.png"))
tem.append(cv.imread("templates/car4.png"))
tem.append(cv.imread("templates/car5.png"))
# tem.append(cv.imread("templates/rb1.png"))
# tem.append(cv.imread("templates/rb2.png"))
# tem.append(cv.imread("templates/rb3.png"))
# tem.append(cv.imread("templates/rb4.png"))
# tem.append(cv.imread("templates/rb5.png"))
obs = Obstacle(tem, num_tem)

park = Maneuvers()


class State():
    def __init__(self, name=None, method=None, activated=False):
        self.name = name
        self.method = method
        self.active = activated
        self.start_time = None
        self.start_position = None
        self.start_distance = None
        self.just_switched = False
        self.interrupted = False
        # variables specific to state, can be freely assigned
        self.var1 = None
        self.var2 = None
        self.var3 = None
        self.var4 = None

    def __str__(self):
        return self.name.upper() if self.name is not None else 'None'

    def run(self):
        self.method()


ALWAYS_ON_ROUTINES = [nac.UPDATE_STATE, nac.CONTROL_FOR_SIGNS]


class Routine():
    def __init__(self, name, method, activated=False):
        self.name = name
        self.method = method
        self.active = activated
        self.start_time = None
        self.start_position = None
        self.start_distance = None
        self.var1 = None
        self.var2 = None
        self.var3 = None

    def __str__(self):
        return self.name

    def run(self):
        self.method()


EVENT_TYPES = [nac.INTERSECTION_STOP_EVENT,
               nac.INTERSECTION_TRAFFIC_LIGHT_EVENT,
               nac.INTERSECTION_PRIORITY_EVENT,
               nac.JUNCTION_EVENT,
               nac.ROUNDABOUT_EVENT,
               nac.CROSSWALK_EVENT,
               nac.PARKING_EVENT,
               nac.HIGHWAY_EXIT_EVENT]


class Event:
    def __init__(self, name=None, dist=None, point=None, yaw_stopline=None,
                 path_ahead=None, length_path_ahead=None, curvature=None):
        self.name = name  # name/type of the event
        self.dist = dist  # distance of event from start of path
        # [x,y] position on the map of the event
        self.point = point
        self.yaw_stopline = yaw_stopline  # yaw of the stop line at the event
        # <++>
        # if self.yaw_stopline is None:
        #     self.yaw_stopline = 0.0
        # sequence of points after the event,
        # only for intersections or roundabouts
        self.path_ahead = path_ahead
        # length of the path after the event,
        # only for intersections or roundabouts
        self.length_path_ahead = length_path_ahead
        self.curvature = curvature  # curvature of the path ahead of the event

    def __str__(self):
        return self.name.upper() if self.name is not None else 'None'


CONDITIONS = {
        # if true, the car is in presence of a
        # dotted line and is allowed to overtake it
        nac.CAN_OVERTAKE: False,
        # if true, the car is in a highway, the speed
        # should be higher on highway,
        nac.HIGHWAY:      False,
        # if true the car will trust the gps, for example
        # in expecting a sign or traffic light
        # can be set as false if there is a lot of
        # package loss or the car has not received signal in a while
        nac.TRUST_GPS:    True,
        # if true, the car is on the path, if the gps is trusted
        # and the position is too far from the path it will be set to false
        nac.CAR_ON_PATH:  True,
        # if true, the car is rerouting, for example at the
        # beginning or after a roadblock
        nac.REROUTING:    True,
        # if true, the car is on a bumpy road
        nac.BUMPY_ROAD:   False,
}

ACHIEVEMENTS = {
        nac.PARK_ACHIEVED: False
}

# ==============================================================
# ========================= PARAMTERS ==========================
# ==============================================================
# signs
SIGN_DIST_THRESHOLD = 0.5
# sempahores
SEMAPHORE_IS_ALWAYS_GREEN = False if not SIMULATOR_FLAG else True
# SEMAPHORE_IS_ALWAYS_GREEN = True

DEQUE_OF_PAST_FRAMES_LENGTH = 50
DISTANCES_BETWEEN_FRAMES = 0.03

# Yaw
APPLY_YAW_CORRECTION = True
GPS_DELAY = 0.45  # [s] delay for gps message to arrive
ENCODER_POS_FREQ = 100.0  # [Hz] frequency of encoder position messages
GPS_FREQ = 10.0  # [Hz] frequency of gps messages
BUFFER_PAST_MEASUREMENTS_LENGTH = int(round(GPS_DELAY * ENCODER_POS_FREQ))

# Vehicle driving parameters
MIN_SPEED = -0.3                    # [m/s]     minimum speed
MAX_SPEED = 2.5                     # [m/s]     maximum speed
MAX_ACCEL = 5.5                     # [m/ss]    maximum accel
MAX_STEER = 28.0                    # [deg]     maximum steering angle

# Vehicle parameters
LENGTH = 0.45  	    # [m]       car body length
WIDTH = 0.18   	    # [m]       car body width
BACKTOWHEEL = 0.10  # [m]       distance of the wheel and the car body
WHEEL_LEN = 0.03

# STOPLINES
STOPLINE_APPROACH_DISTANCE = 0.4
STOPLINE_STOP_DISTANCE = 0.15  # 0.1 #in the true map
GPS_STOPLINE_APPROACH_DISTANCE = 0.8
GPS_STOPLINE_STOP_DISTANCE = 0.5
assert STOPLINE_STOP_DISTANCE <= STOPLINE_APPROACH_DISTANCE
assert GPS_STOPLINE_STOP_DISTANCE <= GPS_STOPLINE_APPROACH_DISTANCE

STOP_WAIT_TIME = 0.001*3.0 if not SPEED_CHALLENGE else 0.0  # 3.0
# local tracking
OPEN_LOOP_PERCENTAGE_OF_PATH_AHEAD = 0.6  # 0.6
# distance from previous stopline from which is possible to
# start detecting a stop line again
STOPLINE_DISTANCE_THRESHOLD = 0.2
POINT_AHEAD_DISTANCE_LOCAL_TRACKING = 0.3  # 0.3

# speed control
# multiplier for desired speed, used to regulate highway speed
ACCELERATION_CONST = 1.2
SLOW_DOWN_CONST = 0.3

# highway exit
# [m] go straight for this distance in orther to exit the hihgway
STRAIGHT_DIST_TO_EXIT_HIGHWAY = 0.8

# Rerouting
# distance between 2 consecutive measure of the gps for
# the kalmann filter to be considered converged
GPS_DISTANCE_THRESHOLD_FOR_CONVERGENCE = 0.2
GPS_SAMPLE_TIME = 0.25  # [s] time between 2 consecutive gps measurements
GPS_CONVERGENCE_PATIANCE = 0  # 2 #iterations to consider the gps converged
GPS_TIMEOUT = 5.0  # [s] time to wait to have gps signal

# end state
# [m] distance from the end of the path for the car
# to be considered at the end of the path
END_STATE_DISTANCE_THRESHOLD = 0.3

# PARKING
PARKING_DISTANCE_SLOW_DOWN_THRESHOLD = 0.7  # 1.0
PARKING_DISTANCE_STOP_THRESHOLD = 0.1  # 0.1
# length in samples of the path to consider around the parking position, max
SUBPATH_LENGTH_FOR_PARKING = 300
ALWAYS_USE_GPS_FOR_PARKING = False  # debug
ALWAYS_USE_SIGN_FOR_PARKING = False  # debug
DEFAULT_PARKING_METHOD = 'gps'  # 'gps' or 'sign'
assert not (ALWAYS_USE_GPS_FOR_PARKING and ALWAYS_USE_SIGN_FOR_PARKING)
# [s] max seconds to wait for gps to be available
PARK_MAX_SECONDS_W8_GPS = 10.0
MAX_PARK_SEARCH_DIST = 2.0  # [m] max distance to search for parking
# index offset from the saved parking position
IDX_OFFSET_FROM_SAVED_PARK_POSITION = 56 + 13
# [s] max seconds to wait for a sign to be available
PARK_SIGN_DETETCTION_PATIENCE = 8.0
PARK_SEARCH_SPEED = 0.1  # [m/s] speed to search for parking
PARK_MANOUVER_SPEED = 0.15  # [m/s] speed to perform the parking manouver
# [m] distance from the sign to the first parking spot
DIST_SIGN_FIRST_T_SPOT = 0.75
DIST_T_SPOTS = 0.45  # [m] distance from the sign to the second parking spot
# [m] distance from the sign to the first parking spot
DIST_SIGN_FIRST_S_SPOT = 0.9
DIST_S_SPOTS = 0.7  # [m] distance from the sign to the second parking spot
# [m] distance to proceed further in order to perform the s manouver
FURTHER_DIST_S = 0.69+0.0
# [m] distance to proceed further in order to perform the t manouver
FURTHER_DIST_T = 0.64+0.0
T_ANGLE = 27.0  # [deg] angle to perform the t manouver
S_ANGLE = 29.0  # [deg] angle to perform the s manouver
DIST_2T = 0.8  # [m] distance to perform the 2nd part of t manouver
DIST_3T = 0.1  # [m] distance to perform the 3rd part of t manouver
DIST_2S = 0.40  # 0.38 #[m] distance to perform the 2nd part of s manouver
DIST_4S = 0.05  # [m] dsemaphoreistance to perform the 4th part of s manouver
STEER_ACTUATION_DELAY_PARK = 0.5  # [s] delay to perform the steering manouver
# [s] WARNING: this stops the state machine. So be careful increasing it
SLEEP_AFTER_STOPPING = 0.3
STEER_ACTUATION_DELAY = 0.3  # [s] delay to perform the steering manouver

# OBSTACLES
OBSTACLE_IS_ALWAYS_PEDESTRIAN = False
OBSTACLE_IS_ALWAYS_CAR = False
OBSTACLE_IS_ALWAYS_ROADBLOCK = False

# obstacle classification
# dont detect obstacle for this distance after detecting one of them
MIN_DIST_BETWEEN_OBSTACLES = 0.5
# [m] distance from the obstacle to consider it as an obstacle
OBSTACLE_DISTANCE_THRESHOLD = 0.5
# distance to where to stop wrt the obstacle
OBSTACLE_CONTROL_DISTANCE = 0.3
# dist from where we capture imgs
OBSTACLE_IMGS_CAPTURE_START_DISTANCE = 0.48
# dist up to we capture imgs
OBSTACLE_IMGS_CAPTURE_STOP_DISTANCE = 0.31
assert OBSTACLE_IMGS_CAPTURE_STOP_DISTANCE > OBSTACLE_CONTROL_DISTANCE
# pedestrian
# [m] distance to keep from the pedestrian
PEDESTRIAN_CONTROL_DISTANCE = 0.35
# [s] time to w8 after the pedestrian cleared the road
PEDESTRIAN_TIMEOUT = 2.0
# [m] distance to keep from the vehicle while tailing
TAILING_DISTANCE = 0.25
# overtake static car
OVERTAKE_STEER_ANGLE = 27.0  # [deg]
OVERTAKE_STATIC_CAR_SPEED = 0.2  # [m/s]
OT_STATIC_SWITCH_1 = 0.3
OT_STATIC_SWITCH_2 = 0.55
OT_STATIC_LANE_FOLLOW = 0.3
# overtake moving car
OVERTAKE_MOVING_CAR_SPEED = 0.5  # [m/s]
OT_MOVING_SWITCH_1 = 0.3  # [m]
OT_MOVING_LANE_FOLLOW = 1.420  # [m]
OT_MOVING_SWITCH_2 = 0.3  # [m]
# roadblock
RB_NODES_LEFT_LANE = ['16', '138', '137', '136', '135', '134', '7']
RB_NODES_RIGHT_LANE = ['15', '143', '142', '141', '140', '139', '8']
AVOID_ROADBLOCK_ANGLE = 27.0  # [deg]
AVOID_ROADBLOCK_SPEED = 0.2  # [m/s]
AVOID_ROADBLOCK_DISTANCE = 0.7  # [m]

# CHECKS
# [m] max distance from the lane to trip the state checker
MAX_DIST_AWAY_FROM_LANE = 0.8
MAX_ERROR_ON_LOCAL_DIST = 0.05  # [m] max error on the local distance

BRAINLESS_MAXD = 18


# ==============================================================
# =========================== BRAIN ============================
# ==============================================================
class Brain:
    def __init__(self,
                 car: Automobile_Data,
                 controller: Controller,
                 controller_sp: ControllerSpeed,
                 controller_ag: ControllerBL,
                 env: EnvironmentalData,
                 detection: Detection,
                 path_planner: PathPlanning,
                 checkpoints=None,
                 desired_speed=0.3,
                 debug=True):
        print("Initialize brain")
        self.car = car
        self.controller = controller
        self.controller_sp = controller_sp
        self.controller_ag = controller_ag
        self.detect = detection
        self.path_planner = path_planner
        self.env = env

        self.car.drive(speed=0.0, angle=0.0)

        # navigation instruction is a list of tuples:
        self.navigation_instructions = []
        # events are an ordered list of tuples:
        # (type , distance from start, x y position)
        self.events = []
        if checkpoints is not None:
            self.checkpoints = checkpoints
        else:
            self.checkpoints = CHECKPOINTS
        self.checkpoint_idx = 0
        self.desired_speed = desired_speed
        self.parking_method = DEFAULT_PARKING_METHOD
        if ALWAYS_USE_GPS_FOR_PARKING:
            self.parking_method = 'gps'
        if ALWAYS_USE_SIGN_FOR_PARKING:
            self.parking_method = 'sign'

        # current and previous states (class State)
        self.curr_state = State()
        self.prev_state = State()
        # previous and next event (class Event)
        self.prev_event = Event()
        self.next_event = Event()
        self.event_idx = 0

        # stop line with higher precision
        self.stopline_distance_median = 1.0
        self.car_dist_on_path = 0  # init

        # debug
        self.debug = debug
        if self.debug and SHOW_IMGS:
            cv.namedWindow('brain_debug', cv.WINDOW_NORMAL)
            self.debug_frame = None

        self.conditions = CONDITIONS
        self.achievements = ACHIEVEMENTS

        # INITIALIZE STATES

        self.states = {
            nac.START_STATE:   State(nac.START_STATE, self.start_state),
            nac.END_STATE:     State(nac.END_STATE, self.end_state),
            nac.DOING_NOTHING: State(nac.DOING_NOTHING, self.doing_nothing),
            # lane following, between intersections or roundabouts
            nac.LANE_FOLLOWING: State(nac.LANE_FOLLOWING, self.lane_following),
            # intersection navigation, further divided into the possible
            # directions [left, right, straight]
            nac.APPROACHING_STOPLINE:
            State(nac.APPROACHING_STOPLINE, self.approaching_stopline),
            nac.INTERSECTION_NAVIGATION:
            State(nac.INTERSECTION_NAVIGATION, self.intersection_navigation),
            nac.GOING_STRAIGHT:
            State(nac.GOING_STRAIGHT, self.going_straight),
            nac.TRACKING_LOCAL_PATH:
            State(nac.TRACKING_LOCAL_PATH, self.tracking_local_path),
            # roundabout, for roundabouts the car will track the local path
            nac.ROUNDABOUT_NAVIGATION:
            State(nac.ROUNDABOUT_NAVIGATION, self.roundabout_navigation),
            # waiting states
            nac.WAITING_FOR_PEDESTRIAN:
            State(nac.WAITING_FOR_PEDESTRIAN, self.waiting_for_pedestrian),
            nac.WAITING_FOR_GREEN:
            State(nac.WAITING_FOR_GREEN, self.waiting_for_green),
            nac.WAITING_AT_STOPLINE:
            State(nac.WAITING_AT_STOPLINE, self.waiting_at_stopline),
            # overtaking manouver
            nac.OVERTAKING_STATIC_CAR:
            State(nac.OVERTAKING_STATIC_CAR, self.overtaking_static_car),
            nac.OVERTAKING_MOVING_CAR:
            State(nac.OVERTAKING_MOVING_CAR, self.overtaking_moving_car),
            nac.TAILING_CAR:
            State(nac.TAILING_CAR, self.tailing_car),
            nac.AVOIDING_ROADBLOCK:
            State(nac.AVOIDING_ROADBLOCK, self.avoiding_roadblock),
            # parking
            nac.PARKING: State(nac.PARKING, self.parking),
            # crosswalk navigation
            nac.CROSSWALK_NAVIGATION:
            State(nac.CROSSWALK_NAVIGATION, self.crosswalk_navigation),
            nac.CLASSIFYING_OBSTACLE:
            State(nac.CLASSIFYING_OBSTACLE, self.classifying_obstacle),
            nac.BRAINLESS: State(nac.BRAINLESS, self.brainless),
        }

        # INITIALIZE ROUTINES
        self.routines = {
            nac.FOLLOW_LANE:
            Routine(nac.FOLLOW_LANE,  self.follow_lane),
            nac.DETECT_STOPLINE:
            Routine(nac.DETECT_STOPLINE,  self.detect_stopline),
            nac.SLOW_DOWN:
            Routine(nac.SLOW_DOWN,  self.slow_down),
            nac.ACCELERATE:
            Routine(nac.ACCELERATE,  self.accelerate),
            nac.CONTROL_FOR_SIGNS:
            Routine(nac.CONTROL_FOR_SIGNS,  self.control_for_signs),
            nac.CONTROL_FOR_OBSTACLES:
            Routine(nac.CONTROL_FOR_OBSTACLES,  self.control_for_obstacles),
            nac.UPDATE_STATE:
            Routine(nac.UPDATE_STATE, self.update_state),
            nac.DRIVE_DESIRED_SPEED:
            Routine(nac.DRIVE_DESIRED_SPEED, self.drive_desired_speed)
        }
        self.active_routines_names = []

        self.sign_points = np.load('data/sign_points.npy')
        self.sign_types = np.load('data/sign_types.npy').astype(int)
        assert len(self.sign_points) == len(self.sign_types)
        self.sign_seen = np.zeros(len(self.sign_types))
        self.curr_sign = nac.NO_SIGN
        self.past_frames = deque(maxlen=DEQUE_OF_PAST_FRAMES_LENGTH)

        self.frame_for_stopline_angle = None
        self.last_run_call = time()

        self.start_node_validated = False

        print('Brain initialized')
        print('Waiting for start semaphore...')
        sleep(3.0)
        while True:
            semaphore_start_state = self.env.get_semaphore_state(nac.START)
            if SEMAPHORE_IS_ALWAYS_GREEN:
                semaphore_start_state = nac.GREEN
            if semaphore_start_state == nac.GREEN:
                break
            sleep(0.1)

        # <++>
        # <++>
        # <++>
        # <++>
        start_time = time()
        while True:
            # get closest node
            curr_time = time()
            curr_pos = np.array([self.car.x_est, self.car.y_est])
            closest_node, distance = self.path_planner.get_closest_node(
                    curr_pos)
            # sleep(3.0)
            if len(self.car.x_buffer) >= 5:
                print(f'Waiting for gps: \
{(curr_time-start_time):.1f}/{GPS_TIMEOUT}')
                self.checkpoints[self.checkpoint_idx] = closest_node
                if distance > 0.8:
                    self.error('ERROR: REROUTING: GPS converged, but distance \
is too large, we are too far from the lane')
                break
            if curr_time - start_time > GPS_TIMEOUT:
                print('WARNING: ROUTE_GENERATION: No gps signal, \
Starting from the first checkpoint')
                sleep(3.0)
                break
            if ALWAYS_DISTRUST_GPS:
                if STARTING_COORDS != [-42, -42]:
                    curr_pos = np.array(STARTING_COORDS)
                    closest_node, distance = self.path_planner.\
                        get_closest_node(curr_pos)
                    self.checkpoints[self.checkpoint_idx] = closest_node
                    self.car.x_est = curr_pos[0]
                    self.car.y_est = curr_pos[1]
                elif len(self.car.x_buffer) < 5:
                    node_coords = self.path_planner.get_coord(
                        str(self.checkpoints[0]))
                    self.car.x_est = node_coords[0]
                    self.car.y_est = node_coords[1]
                break
            # self.car.update_estimated_state()  # <++>

        # if bool(self.checkpoints[self.checkpoint_idx] in
        #         self.path_planner.intersection_in or
        #         self.checkpoints[self.checkpoint_idx] in
        #         self.path_planner.ra_enter) ^ \
        #    bool(self.detect.detect_stopline(
        #         self.car.frame, show_ROI=SHOW_IMGS)[0] < 0.05):
        #     # self.checkpoints[self.checkpoint_idx] = hf.switch_lane_check(
        #     #                                            closest_node, angle)
        #     print("<++>")

        self.switch_to_state(nac.START_STATE)

    # =============== STATES =============== #
    def start_state(self):
        if self.curr_state.just_switched:
            self.conditions[nac.REROUTING] = True
            self.car.drive_distance(0.0)  # stop
            self.curr_state.var2 = time()
            self.curr_state.just_switched = False

        # localize the car and go to the first checkpoint

        print('Generating route...')
        # get start and end nodes from the chekpoint list
        assert len(self.checkpoints) >= 2, \
            'List of checkpoints needs 2 or more nodes'
        start_node = self.checkpoints[self.checkpoint_idx]
        # already checked in end_state
        end_node = self.checkpoints[self.checkpoint_idx+1]
        print(f'Start node: {start_node}, End node: {end_node}')
        # calculate path
        self.path_planner.compute_shortest_path(start_node, end_node)
        # initialize the list of events on the path
        print('Augmenting path...')
        events = self.path_planner.augment_path(draw=SHOW_IMGS)
        print('Path augmented')
        # add the events to the list of events, increasing it
        self.events = self.create_sequence_of_events(events)
        self.event_idx = 1
        self.next_event = self.events[0]
        self.prev_event.dist = 0.0
        self.car.reset_rel_pose()
        print(f'EVENTS: {self.next_event}, idx: {self.event_idx}')
        for e in self.events:
            print(e)
        # draw the path
        self.path_planner.draw_path()
        print('Starting...')
        if self.next_event.name == nac.PARKING_EVENT:
            print('Skipping parking if its the first event')
            self.go_to_next_event()
        self.conditions[nac.REROUTING] = False
        # reset the signs seen
        self.sign_seen = np.zeros_like(self.sign_seen)

        self.car_dist_on_path = 0

        self.switch_to_state(nac.LANE_FOLLOWING)

    def end_state(self):
        self.activate_routines([nac.SLOW_DOWN])
        self.go_to_next_event()
        # start routing for next checkpoint
        self.next_checkpoint()
        self.switch_to_state(nac.START_STATE)

    # Not used <++>
    def doing_nothing(self):
        self.activate_routines([])

    def lane_following(self):  # LANE FOLLOWING ##############################
        # highway conditions
        if self.prev_event.name == nac.JUNCTION_EVENT and SPEED_CHALLENGE:
            self.switch_to_state(nac.BRAINLESS)

        if self.conditions[nac.HIGHWAY]:
            self.activate_routines([nac.FOLLOW_LANE,
                                    nac.DETECT_STOPLINE,
                                    nac.CONTROL_FOR_OBSTACLES,
                                    nac.ACCELERATE])
        elif self.conditions[nac.BUMPY_ROAD] and SPEED_CHALLENGE:
            self.activate_routines([nac.FOLLOW_LANE,
                                    nac.DETECT_STOPLINE,
                                    nac.CONTROL_FOR_OBSTACLES,
                                    nac.ACCELERATE])
        else:
            self.activate_routines([nac.FOLLOW_LANE,
                                    nac.DETECT_STOPLINE,
                                    nac.CONTROL_FOR_OBSTACLES,
                                    nac.DRIVE_DESIRED_SPEED])

        # check parking
        if self.next_event.name == nac.PARKING_EVENT:
            # we dont need stoplines in parking
            self.activate_routines([nac.FOLLOW_LANE,
                                    nac.CONTROL_FOR_OBSTACLES,
                                    nac.DRIVE_DESIRED_SPEED])
            self.lane_following_to_parking()

        # check highway exit case
        elif self.next_event.name == nac.HIGHWAY_EXIT_EVENT:
            self.lane_following_to_highway_exit()

        # end of current route, go to end state
        elif self.next_event.name == nac.END_EVENT:
            self.lane_following_to_end()

        # we are approaching a stopline, check only if we are far enough
        # from the previous stopline
        else:
            if not self.start_node_validated:
                if self.curr_state.just_switched:
                    # initial position
                    self.curr_state.var1 = [self.car.x_est, self.car.y_est]
                    self.curr_state.just_switched = False
                ladistancedeldiocane = np.\
                    sqrt((self.curr_state.var1[0] - self.car.x_est)**2 +
                         (self.curr_state.var1[1] - self.car.y_est)**2)
                print("LA DISTANCE DEL DIO CANE: ", ladistancedeldiocane)
                closest_node, _ = self.path_planner.get_closest_node(
                        np.array([self.car.x, self.car.y]))
                print('CLOSEST NODE: ', closest_node)
                if closest_node == self.checkpoints[0]:
                    pass
                elif closest_node in self.path_planner.route_graph.nodes:
                    self.start_node_validated = True
                else:
                    langledeldiocane = np.round(np.rad2deg(
                        np.arctan2(self.car.y_est - self.curr_state.var1[1],
                                   self.car.x_est - self.curr_state.var1[0])))
                    self.checkpoints[self.checkpoint_idx] = hf.\
                        switch_lane_check(closest_node, langledeldiocane)
                    print("<++>")

            if self.conditions[nac.TRUST_GPS]:
                dist_to_stopline = self.next_event.dist - self.car_dist_on_path
                if dist_to_stopline >= GPS_STOPLINE_APPROACH_DISTANCE:
                    print(f'Stopline is far: \
{dist_to_stopline-GPS_STOPLINE_APPROACH_DISTANCE:.2f} [m]')
                elif dist_to_stopline > 0.0:
                    print('Switching to approaching stopline')
                    self.switch_to_state(nac.APPROACHING_STOPLINE)
                else:
                    print('It seems we passed the stopline, or path \
self intersected.')
            else:
                far_enough_from_prev_stopline = (self.event_idx == 1) or \
                        (self.car.dist_loc > STOPLINE_DISTANCE_THRESHOLD)
                if self.prev_event.name is not None:
                    print(f'stop enough: {self.car.dist_loc}')
                if self.detect.est_dist_to_stopline < \
                        STOPLINE_APPROACH_DISTANCE and \
                        far_enough_from_prev_stopline and \
                        self.routines[nac.DETECT_STOPLINE].active:
                    self.switch_to_state(nac.APPROACHING_STOPLINE)

    # Event: Parking <++>
    def lane_following_to_parking(self):
        dist_between_events = self.next_event.dist - self.prev_event.dist
        # Relative positioning is reset at every stopline, so we
        # can use that to approximately calculate the distance
        # to the parking spot
        approx_dist_from_parking = dist_between_events - self.car.dist_loc
        print(f'Approx dist from parking: {approx_dist_from_parking}')
        # we are reasonably close to the parking spot
        if SPEED_CHALLENGE:
            self.switch_to_state(nac.PARKING)
        elif approx_dist_from_parking < PARKING_DISTANCE_SLOW_DOWN_THRESHOLD:
            self.car.drive_speed(0.0)
            sleep(SLEEP_AFTER_STOPPING)
            self.switch_to_state(nac.PARKING)

    # Event: Highway exit <++>
    def lane_following_to_highway_exit(self):
        if self.curr_state.just_switched:
            self.curr_state.var1 = self.car.encoder_distance
            self.curr_state.just_switched = False

        if self.conditions[nac.TRUST_GPS]:
            diff = self.next_event.dist - self.car_dist_on_path
            if diff > 0.1:
                print(f'Driving toward highway exit: exiting in \
{diff:.2f} [m]')
            elif diff > -0.05:
                print('Arrived at highway exit, switching to going \
straight for exiting')
                self.switch_to_state(nac.GOING_STRAIGHT)
            else:
                self.error('ERROR: LANE FOLLOWING: Missed Highway exit')

        else:
            diff = self.car.encoder_distance - self.curr_state.var1
            print("##########################################")
            print("diff = ", diff)
            print("##########################################")
            if diff < 2.0:
                print(f'Driving toward highway exit: dist so far \
{diff:.2f} [m]')
            elif diff < 3.2:
                self.activate_routines([])
                print(f'Driving toward highway exit: dist so far \
{diff:.2f} [m]')

                self.car.drive_angle(angle=00)
            else:
                print('Arrived at highway exit, switching to going \
straight for exiting')
                self.switch_to_state(nac.END_STATE)

    # Event: End
    def lane_following_to_end(self):
        if self.curr_state.just_switched:
            self.curr_state.var1 = self.car.encoder_distance
            self.curr_state.just_switched = False

        self.activate_routines([nac.FOLLOW_LANE,
                                nac.CONTROL_FOR_OBSTACLES,
                                nac.DRIVE_DESIRED_SPEED])
        # NOTE End is implemented only with gps now, much more robust,
        # but cannot do it without it
        if self.conditions[nac.TRUST_GPS]:
            dist_to_end = len(self.path_planner.path)*0.01 - \
                    self.car_dist_on_path
            if dist_to_end > END_STATE_DISTANCE_THRESHOLD:
                print(f'Driving toward end: exiting in \
{dist_to_end:.2f} [m]')
            elif dist_to_end > -END_STATE_DISTANCE_THRESHOLD:
                print('Arrived at end, switching to end state')
                self.switch_to_state(nac.END_STATE)
            else:
                self.error('ERROR: LANE FOLLOWING: Missed end')
        else:
            diff = self.car.encoder_distance - self.curr_state.var1
            dist_to_end = self.next_event.dist - diff
            # dist_to_end = len(self.path_planner.path)*0.01 - diff
            print('DIST TO END: ', dist_to_end)
            if dist_to_end > END_STATE_DISTANCE_THRESHOLD:
                print(f'Driving toward end: exiting in \
{dist_to_end:.2f} [m]')
            elif dist_to_end > -END_STATE_DISTANCE_THRESHOLD:
                print('Arrived at end, switching to end state')
                self.switch_to_state(nac.END_STATE)
            else:
                self.error('ERROR: LANE FOLLOWING: Missed end')

    def approaching_stopline(self):
        # FOLLOW_LANE, SLOW_DOWN, DETECT_STOPLINE, CONTROL_FOR_OBSTACLES
        if not SPEED_CHALLENGE:
            self.activate_routines([nac.FOLLOW_LANE,
                                    nac.SLOW_DOWN,
                                    nac.DETECT_STOPLINE,
                                    nac.CONTROL_FOR_OBSTACLES])

        if self.curr_state.just_switched:
            cv.imwrite(f'asl/asl_{int(time() * 1000)}.png', self.car.frame)
            self.curr_state.just_switched = False
        if self.conditions[nac.TRUST_GPS] and not \
                ALWAYS_USE_VISION_FOR_STOPLINES:
            decide_next_state = self.approaching_stopline_gps()
        else:
            decide_next_state = self.approaching_stopline_vision()

        if decide_next_state:
            print('Deciding next state, based on next event...')
            next_event_name = self.next_event.name
            # Events with stopline
            if next_event_name == nac.INTERSECTION_STOP_EVENT:
                self.switch_to_state(nac.WAITING_AT_STOPLINE)
            elif next_event_name == nac.INTERSECTION_TRAFFIC_LIGHT_EVENT:
                self.switch_to_state(nac.WAITING_FOR_GREEN)
            elif next_event_name == nac.INTERSECTION_PRIORITY_EVENT:
                self.switch_to_state(nac.INTERSECTION_NAVIGATION)
            elif next_event_name == nac.JUNCTION_EVENT:
                # TODO: careful with this
                self.switch_to_state(nac.INTERSECTION_NAVIGATION)
            elif next_event_name == nac.ROUNDABOUT_EVENT:
                self.switch_to_state(nac.ROUNDABOUT_NAVIGATION)
            elif next_event_name == nac.CROSSWALK_EVENT:
                # directly go to lane keeping, the pedestrian will
                # be managed in that state
                self.switch_to_state(nac.CROSSWALK_NAVIGATION)
            # Events without stopline = LOGIC ERROR
            elif next_event_name == nac.PARKING_EVENT:
                self.error('WARNING: UNEXPECTED STOP LINE FOUND WITH \
PARKING AS NEXT EVENT')
            elif next_event_name == nac.HIGHWAY_EXIT_EVENT:
                self.error('WARNING: UNEXPECTED STOP LINE FOUND WITH \
HIGHWAY EXIT AS NEXT EVENT')
            else:
                self.error('ERROR: UNEXPECTED STOP LINE FOUND WITH \
UNKNOWN EVENT AS NEXT EVENT')
            self.activate_routines([])  # deactivate all routines

    # Substate
    def approaching_stopline_gps(self):
        dist_to_stopline = self.next_event.dist - self.car_dist_on_path
        if dist_to_stopline >= GPS_STOPLINE_APPROACH_DISTANCE:
            print('Switching to lane following')
            self.switch_to_state(nac.LANE_FOLLOWING)
            return
        elif dist_to_stopline >= GPS_STOPLINE_STOP_DISTANCE:
            print(f'Approaching stop line: \
{dist_to_stopline-GPS_STOPLINE_STOP_DISTANCE:.2f} [m]')
            decide_next_state = False
        elif dist_to_stopline >= 0:
            print('Arrived at stop line')
            decide_next_state = True
        else:
            self.error(f'ERROR: APPROACHING STOP LINE: Missed stop line, \
dist: {dist_to_stopline}')
        return decide_next_state

    # Substate
    def approaching_stopline_vision(self):
        dist = self.detect.est_dist_to_stopline
        # #check if we are here by mistake
        if dist > STOPLINE_APPROACH_DISTANCE:
            self.switch_to_state(nac.LANE_FOLLOWING)
            return False
        # we have a median => we have an accurate position for the stopline
        if self.stopline_distance_median is not None:
            print('Driving towards stop line... at distance: ',
                  self.stopline_distance_median)
            self.activate_routines([nac.FOLLOW_LANE,
                                    nac.SLOW_DOWN])  # SLOW_DOWN
            dist_to_drive = self.stopline_distance_median - \
                self.car.encoder_distance
            self.car.drive_distance(dist_to_drive)
            if dist_to_drive < STOPLINE_STOP_DISTANCE:
                print(f'Arrievd at stop line. Using median distance: \
{self.stopline_distance_median}')
                print(f'                           encoder distance: \
{self.car.encoder_distance:.2f}')
                # sleep(1.0)
                decide_next_state = True
            else:
                decide_next_state = False
        # alternative, if we don't have a median, we just use the
        # (possibly inaccurate) network estimaiton
        else:
            print('WARNING: APPROACHING_STOPLINE: stop distance may \
be imprecise')
            if dist < STOPLINE_STOP_DISTANCE:
                print('Stopped at stop line. Using network distance: ',
                      self.detect.est_dist_to_stopline)
                decide_next_state = True
                dist_from_line = dist
                assert dist_from_line < 0.5, \
                    f'dist_from_line is too large, {dist_from_line:.2f}'
            else:
                decide_next_state = False
        return decide_next_state

    def intersection_navigation(self):
        self.activate_routines([])
        self.switch_to_state(nac.TRACKING_LOCAL_PATH)

    def going_straight(self):
        if self.curr_state.just_switched:
            self.activate_routines([])
            if self.next_event.name == nac.HIGHWAY_EXIT_EVENT:
                distance_to_stop = STRAIGHT_DIST_TO_EXIT_HIGHWAY
            else:
                distance_to_stop = self.curr_state.start_distance + \
                        OPEN_LOOP_PERCENTAGE_OF_PATH_AHEAD * \
                        self.next_event.length_path_ahead
            self.curr_state.var1 = distance_to_stop + self.car.encoder_distance
            self.curr_state.just_switched = False

        distance_to_stop = self.curr_state.var1
        if self.car.encoder_distance < distance_to_stop:
            self.car.drive(speed=self.desired_speed, angle=3.0)
        # end of the maneuver
        else:
            self.switch_to_state(nac.LANE_FOLLOWING)
            self.go_to_next_event()

    def tracking_local_path(self):
        # var1=initial distance from stopline, #var2=path to follow
        # var3=local path, #var4=intersection direction / ra pred avg
        print('State: tracking_local_path')
        # self.activate_routines([nac.DRIVE_DESIRED_SPEED])
        self.activate_routines([])
        if self.curr_state.just_switched:
            stopline_position = self.next_event.point
            stopline_yaw = self.next_event.yaw_stopline
            # local path in the stop line frame
            local_path_slf_rot = self.next_event.path_ahead

            if self.conditions[nac.TRUST_GPS] and not \
                    ALWAYS_USE_VISION_FOR_STOPLINES:
                USE_PRECISE_LOCATION_AND_YAW = False
                point_car_est = np.array([self.car.x_est, self.car.y_est])

                if USE_PRECISE_LOCATION_AND_YAW:
                    angle = self.car.yaw
                    car_position_slf = point_car_est - stopline_position
                    car_position_slf = car_position_slf @ hf.rot_matrix(angle)
                else:
                    x_dist = self.next_event.dist - self.car_dist_on_path
                    y_dist = 0.0
                    car_position_slf = -np.array([x_dist, y_dist])
                print('Car position in stop line frame: ', car_position_slf)
            else:
                _, stopline_y, _ = \
                        self.detect.detect_stopline(self.car.frame,
                                                    show_ROI=SHOW_IMGS)
                e2 = stopline_y
                if self.stopline_distance_median is not None:
                    print('We HAVE the median, using median estimation')
                    print(len(self.routines[nac.DETECT_STOPLINE].var2))
                    d = self.stopline_distance_median - \
                        self.car.encoder_distance
                # we do not have an accurate position for the stopline
                else:
                    print('We DONT have the median, using \
simple net estimation')
                    print(len(self.routines[nac.DETECT_STOPLINE].var2))
                    if self.detect.est_dist_to_stopline < \
                            STOPLINE_APPROACH_DISTANCE:
                        d = self.detect.est_dist_to_stopline
                    else:
                        d = 0.0

                car_position_slf = -np.array([+d+0.33, +e2])

            # get orientation of the car in the stop line frame
            yaw_car = self.car.yaw
            yaw_mult_90 = hf.get_yaw_closest_axis(yaw_car)
            # get the difference from the closest multiple of 90deg
            alpha = hf.diff_angle(yaw_car, yaw_mult_90)
            # alpha_true = alpha
            print(f'alpha true: {np.rad2deg(alpha):.1f}')
            alpha = self.detect.detect_yaw_stopline(self.car.frame,
                                                    SHOW_IMGS and False) * 0.8
            print(f'alpha est: {np.rad2deg(alpha):.1f}')
            if APPLY_YAW_CORRECTION:
                closest_node, _ = self.path_planner.get_closest_node(
                        np.array([self.car.x, self.car.y]))
                if closest_node not in \
                        self.path_planner.no_yaw_calibration_nodes:
                    print(f'yaw = {np.rad2deg(self.car.yaw):.2f}')
                    print(f'est yaw = \
{np.rad2deg(self.next_event.yaw_stopline + alpha):.2f}')
                    diff = hf.diff_angle(self.next_event.yaw_stopline + alpha,
                                         self.car.yaw)
                    self.car.yaw_offset += diff
                    self.car.yaw += diff
            assert abs(alpha) < np.pi/6, \
                f'Car orientation wrt stopline is too big, it needs to be \
better aligned, alpha = {alpha}'

            # get position of the car in the stop line frame
            # NOTE: rotation first if we ignore the lateral error and \
            #       consider only the euclidean distance from the line
            # cf = car frame
            local_path_cf = (local_path_slf_rot @ hf.rot_matrix(alpha)) - \
                car_position_slf
            # rotate from slf to cf
            self.curr_state.var1 = local_path_cf
            # Every time we stop for a stopline, we reset the local
            # frame of reference
            self.car.reset_rel_pose()
            self.curr_state.just_switched = False

            if self.next_event.name.startswith('intersection') or \
               self.next_event.name.startswith('junction'):
                hf.determine_intersection_direction(self, local_path_cf)
            else:
                self.curr_state.var4 = 0

            hf.show_local_path_just_switched(self, alpha, stopline_yaw,
                                             car_position_slf, local_path_cf,
                                             stopline_position, SHOW_IMGS)

        D = POINT_AHEAD_DISTANCE_LOCAL_TRACKING
        # track the local path using simple pure pursuit
        local_path = self.curr_state.var1
        car_pos_loc = np.array([self.car.x_loc, self.car.y_loc])
        local_path_cf = local_path - car_pos_loc
        dist_path = norm(local_path_cf, axis=1)
        # get idx of car position on the path
        idx_car_on_path = np.argmin(dist_path)
        dist_path = dist_path[idx_car_on_path:]
        dist_path = np.abs(dist_path - D)
        # get idx of point ahead
        idx_point_ahead = np.argmin(dist_path) + idx_car_on_path
        print(f'idx_point_ahead: {idx_point_ahead} / {len(local_path_cf)}')

        local_path_cf = local_path_cf @ hf.rot_matrix(self.car.yaw_loc)

        hf.show_local_path(self, car_pos_loc, SHOW_IMGS)

        # the local path is straight
        if (np.abs(hf.get_curvature(local_path_cf)) < 0.1 and
           not self.next_event.name.startswith("roundabout")):
            print('straight')
            # max_idx = len(local_path_cf)-60  # dont follow until the end
            max_idx = len(local_path_cf)-1  # dont follow until the end
        else:  # curvy path
            max_idx = len(local_path_cf)-1  # follow until the end
            print('curvy')

        # State exit conditions
        if self.next_event.name.startswith("junction"):
            if self.curr_state.var4 == "right":
                max_idx = max_idx * 0.80
            else:
                max_idx = max_idx * 0.60
        if idx_point_ahead >= max_idx:  # we reached the end of the path
            self.switch_to_state(nac.LANE_FOLLOWING)
            self.go_to_next_event()
        elif self.next_event.name.startswith("intersection"):
            hf.navigate_intersection(self, SHOW_IMGS)
        elif self.next_event.name.startswith("junction"):
            hf.navigate_junction(self, idx_point_ahead, SHOW_IMGS)
        elif self.next_event.name.startswith("roundabout"):
            hf.navigate_roundabout(self, idx_point_ahead, max_idx, SHOW_IMGS)
        else:  # we are still on the path
            hf.navigate_open_loop(self,
                                  local_path_cf,
                                  idx_point_ahead,
                                  idx_car_on_path,
                                  SHOW_IMGS)

    # Consider removing this <++>
    def roundabout_navigation(self):
        self.switch_to_state(nac.TRACKING_LOCAL_PATH)

    def waiting_for_pedestrian(self):
        self.activate_routines([nac.FOLLOW_LANE])
        dist_ahead = self.car.filtered_sonar_distance
        if self.curr_state.just_switched:
            self.car.drive_speed(0.0)
            sleep(SLEEP_AFTER_STOPPING)
            self.activate_routines([])
            dist_to_keep = dist_ahead - PEDESTRIAN_CONTROL_DISTANCE
            print('dist_ahead: ', dist_ahead)
            print(f'dist_to_keep: {dist_to_keep}')
            # last time I saw the pedestrian, initiliaze it
            self.curr_state.var1 = time()
            self.curr_state.just_switched = False
        if dist_ahead < OBSTACLE_DISTANCE_THRESHOLD:
            print('Waiting for pedestrian...')
            self.activate_routines([])
            self.curr_state.var1 = last_seen_pedestrian_time = time()
        else:
            last_seen_pedestrian_time = self.curr_state.var1
            self.activate_routines([])
            curr_time = time()
            print(f'Pedestrian has cleared the road, keep waiting for: \
{curr_time - last_seen_pedestrian_time}/{PEDESTRIAN_TIMEOUT}')
            if curr_time - last_seen_pedestrian_time > PEDESTRIAN_TIMEOUT:
                sleep(SLEEP_AFTER_STOPPING)
                print(f'Switching back to {self.prev_state}')
                # NOTE
                self.switch_to_state(nac.LANE_FOLLOWING)

    def waiting_for_green(self):
        semaphore, tl_state = self.env.get_closest_semaphore_state(
                np.array([self.car.x_est, self.car.y_est]))
        print(f'Waiting at semaphore: {semaphore}, state: {tl_state}')
        if self.curr_state.just_switched:
            self.activate_routines([])
            if tl_state != nac.GREEN:
                self.car.drive_speed(0.0)
            # publish traffic light
            self.env.publish_obstacle(nac.TRAFFIC_LIGHT,
                                      self.car.x_est,
                                      self.car.y_est)
            self.curr_state.just_switched = False
        if tl_state == nac.GREEN or SEMAPHORE_IS_ALWAYS_GREEN:
            self.switch_to_state(nac.INTERSECTION_NAVIGATION)

    def waiting_at_stopline(self):
        # no routines
        self.activate_routines([])
        if STOP_WAIT_TIME > 0.0:
            if self.curr_state.just_switched:
                self.activate_routines([])
                self.car.drive_speed(0.0)
                self.curr_state.just_switched = False
            if (time() - self.curr_state.start_time) > STOP_WAIT_TIME:
                self.switch_to_state(nac.INTERSECTION_NAVIGATION)
        else:
            self.switch_to_state(nac.INTERSECTION_NAVIGATION)

    def overtaking_static_car(self):
        self.activate_routines([])
        # states
        OT_SWITCHING_LANE = 1
        OT_LANE_FOLLOWING = 2
        OT_SWITCHING_BACK = 3

        if self.curr_state.just_switched:
            self.curr_state.var1 = (OT_SWITCHING_LANE, True)
            self.curr_state.var2 = self.car.encoder_distance
            self.env.publish_obstacle(nac.STATIC_CAR_ON_ROAD,
                                      self.car.x_est, self.car.y_est)
            self.curr_state.just_switched = False
        sub_state, just_sub_switched = self.curr_state.var1
        dist_prev_manouver = self.curr_state.var2
        if sub_state == OT_SWITCHING_LANE:
            if just_sub_switched:
                self.car.drive_angle(-OVERTAKE_STEER_ANGLE)
                dist_prev_manouver = self.car.encoder_distance
                self.car.drive_speed(OVERTAKE_STATIC_CAR_SPEED)
                just_sub_switched = False
            dist = self.car.encoder_distance - dist_prev_manouver
            assert dist > -0.05
            print(f'Switching lane: {dist:.2f}/{OT_STATIC_SWITCH_1:.2f}')
            if dist > OT_STATIC_SWITCH_1:
                sub_state, just_sub_switched = OT_LANE_FOLLOWING, True
                dist_prev_manouver = self.car.encoder_distance
        elif sub_state == OT_LANE_FOLLOWING:
            self.activate_routines([nac.FOLLOW_LANE])
            dist = self.car.encoder_distance - dist_prev_manouver
            print(f'Following lane: {dist:.2f}/{OT_STATIC_LANE_FOLLOW:.2f}')
            if dist > OT_STATIC_LANE_FOLLOW:
                sub_state, just_sub_switched = OT_SWITCHING_BACK, True
                dist_prev_manouver = self.car.encoder_distance
        elif sub_state == OT_SWITCHING_BACK:
            if just_sub_switched:
                self.car.drive_angle(OVERTAKE_STEER_ANGLE)
                dist_prev_manouver = self.car.encoder_distance
                just_sub_switched = False
            dist = self.car.encoder_distance - dist_prev_manouver
            assert dist > -0.05
            print(f'Switching back: {dist:.2f}/{OT_STATIC_SWITCH_2:.2f}')
            if dist > OT_STATIC_SWITCH_2:
                self.switch_to_state(nac.LANE_FOLLOWING)
        else:
            self.error('ERROR: OVERTAKE: Wrong substate')

        self.curr_state.var1 = (sub_state, just_sub_switched)
        self.curr_state.var2 = dist_prev_manouver

    def overtaking_moving_car(self):
        self.activate_routines([])
        # states
        OT_SWITCHING_LANE = 1
        OT_LANE_FOLLOWING = 2
        OT_SWITCHING_BACK = 3

        if self.curr_state.just_switched:
            self.curr_state.var1 = (OT_SWITCHING_LANE, True)
            self.curr_state.var2 = self.car.encoder_distance
            self.curr_state.just_switched = False
        sub_state, just_sub_switched = self.curr_state.var1
        dist_prev_manouver = self.curr_state.var2
        if sub_state == OT_SWITCHING_LANE:
            if just_sub_switched:
                self.car.drive_angle(-OVERTAKE_STEER_ANGLE)
                dist_prev_manouver = self.car.encoder_distance
                self.car.drive_speed(OVERTAKE_MOVING_CAR_SPEED)
                just_sub_switched = False
            dist = self.car.encoder_distance - dist_prev_manouver
            assert dist > -0.05
            print(f'Switching lane: {dist:.2f}/{OT_MOVING_SWITCH_1:.2f}')
            if dist > OT_MOVING_SWITCH_1:
                sub_state, just_sub_switched = OT_LANE_FOLLOWING, True
                dist_prev_manouver = self.car.encoder_distance
        elif sub_state == OT_LANE_FOLLOWING:
            self.activate_routines([nac.FOLLOW_LANE])
            dist = self.car.encoder_distance - dist_prev_manouver
            print(f'Following lane: {dist:.2f}/{OT_MOVING_LANE_FOLLOW:.2f}')
            if dist > OT_MOVING_LANE_FOLLOW:
                sub_state, just_sub_switched = OT_SWITCHING_BACK, True
                dist_prev_manouver = self.car.encoder_distance
        elif sub_state == OT_SWITCHING_BACK:
            if just_sub_switched:
                self.car.drive_angle(OVERTAKE_STEER_ANGLE)
                dist_prev_manouver = self.car.encoder_distance
                just_sub_switched = False
            dist = self.car.encoder_distance - dist_prev_manouver
            assert dist > -0.05
            print(f'Switching back: {dist:.2f}/{OT_MOVING_SWITCH_2:.2f}')
            if dist > OT_MOVING_SWITCH_2:
                self.switch_to_state(nac.LANE_FOLLOWING)
        else:
            self.error('ERROR: OVERTAKE: Wrong substate')
        self.curr_state.var1 = (sub_state, just_sub_switched)
        self.curr_state.var2 = dist_prev_manouver

    def tailing_car(self):
        dist = self.car.filtered_sonar_distance
        if dist > OBSTACLE_DISTANCE_THRESHOLD+0.05:
            self.switch_to_state(nac.LANE_FOLLOWING)
        else:
            self.activate_routines([nac.FOLLOW_LANE])
            dist_to_drive = dist - TAILING_DISTANCE
            self.car.drive_distance(dist_to_drive)
            if self.conditions[nac.CAN_OVERTAKE]:
                if self.conditions[nac.HIGHWAY]:
                    self.switch_to_state(nac.OVERTAKING_MOVING_CAR)
                else:
                    if -0.05 < dist_to_drive < 0.05:
                        self.switch_to_state(nac.OVERTAKING_STATIC_CAR)

    def avoiding_roadblock(self):
        self.activate_routines([])
        # substates
        AR_WATING_FOR_GPS = 1
        AR_SWITCHING_LANE = 2
        if self.curr_state.just_switched:
            self.car.drive_distance(0.0)
            self.curr_state.var2 = time()
            self.curr_state.var3 = (AR_WATING_FOR_GPS, True)
            self.curr_state.var4 = True  # IN RIGHT LANE
            self.curr_state.just_switched = False

        substate, just_switched_substate = self.curr_state.var3
        print(f'Substate: {substate}, just switched: {just_switched_substate}')
        if substate == AR_WATING_FOR_GPS:
            curr_time = time()
            # <++>
            if ALWAYS_DISTRUST_GPS:
                self.curr_state.var3 = (AR_SWITCHING_LANE, True)
                self.curr_state.var4 = True
            elif self.conditions[nac.TRUST_GPS]:
                curr_pos = np.array([self.car.x_est, self.car.y_est])
                self.env.publish_obstacle(nac.ROADBLOCK, self.car.x_est,
                                          self.car.y_est)
                closest_node, distance = self.\
                    path_planner.get_closest_node(curr_pos)
                print(f'GPS converged, node: {closest_node}, distance: \
{distance:.2f}')
                if closest_node in RB_NODES_LEFT_LANE:
                    self.curr_state.var3 = (AR_SWITCHING_LANE, True)
                    self.curr_state.var4 = False
                elif closest_node in RB_NODES_RIGHT_LANE:
                    self.curr_state.var3 = (AR_SWITCHING_LANE, True)
                    self.curr_state.var4 = True
                else:
                    self.conditions[nac.TRUST_GPS] = False
                if distance > 0.8:
                    self.error('ERROR: REROUTING: GPS converged, but distance \
is too large, we are too far from the lane')
            else:
                start_time = self.curr_state.var2
                print(f'Waiting for gps: \
{(curr_time-start_time):.1f}/{GPS_TIMEOUT}')
                if curr_time - start_time > GPS_TIMEOUT:
                    # TODO manage this case
                    self.error('WARNING: AVOIDING ROADBLOCK: No gps signal')
        elif substate == AR_SWITCHING_LANE:
            in_right_lane = self.curr_state.var4
            avoid_angle = AVOID_ROADBLOCK_ANGLE if in_right_lane \
                else -AVOID_ROADBLOCK_DISTANCE
            curr_check_new = 135 if in_right_lane \
                else 140
            next_check_new = 145 if in_right_lane \
                else 146
            if just_switched_substate:
                print('Switching to left lane')
                self.car.drive_angle(-avoid_angle)
                sleep(STEER_ACTUATION_DELAY)
                self.car.drive_speed(AVOID_ROADBLOCK_SPEED)
                self.curr_state.var1 = \
                    start_encoder_pos = self.car.encoder_distance

                self.checkpoints[self.checkpoint_idx] = curr_check_new
                self.checkpoints.insert(self.checkpoint_idx+1, next_check_new)

                start_node = self.checkpoints[self.checkpoint_idx]
                # already checked in end_state
                end_node = self.checkpoints[self.checkpoint_idx+1]
                print(f'Start node: {start_node}, End node: {end_node}')
                # calculate path
                self.path_planner.compute_shortest_path(start_node, end_node)
                # initialize the list of events on the path
                print('Augmenting path...')
                events = self.path_planner.augment_path(draw=SHOW_IMGS)
                print('Path augmented')
                # add the events to the list of events, increasing it
                self.events = self.create_sequence_of_events(events)
                self.event_idx = 1
                self.next_event = self.events[0]
                self.prev_event.dist = 0.0
                self.car.reset_rel_pose()
                print(f'EVENTS: {self.next_event}, idx: {self.event_idx}')
                for e in self.events:
                    print(e)
                # draw the path
                self.path_planner.draw_path()
                print('Starting...')
                self.curr_state.var3 = (AR_SWITCHING_LANE, False)
            else:
                start_encoder_pos = self.curr_state.var1
                curr_encoder_dist = self.car.encoder_distance
                if curr_encoder_dist - start_encoder_pos > \
                        AVOID_ROADBLOCK_DISTANCE*0.6:
                    self.car.drive_angle(avoid_angle)
                if curr_encoder_dist - start_encoder_pos > \
                        AVOID_ROADBLOCK_DISTANCE:
                    print('Arrived, switching back to rerouting')
                    self.car.drive_speed(0.0)
                    self.switch_to_state(nac.START_STATE)
        else:
            self.error('ERROR: AVOIDING_ROADBLOCK: Wrong substate')

    def parking(self):
        if SPEED_CHALLENGE:
            self.car.drive(speed=0.4, angle=0.0)
            sleep(2)
            self.parking_end()
            return
        # Substates
        if self.curr_state.just_switched:
            # We just got in the parking state, we came from lane following,
            # we are reasonably close to the parking spot and we are not moving
            # self.curr_state.var1 will hold the parking substate,
            # the parking type, and if it has just changed state
            park_state = nac.LOCALIZING_PARKING_SPOT
            # find park type with position
            park_pos = self.next_event.point
            s_pos = np.array(self.path_planner.get_coord('177'))
            t_pos = np.array(self.path_planner.get_coord('162'))
            d_s = norm(park_pos - s_pos)  # distances from the s parking spot
            d_t = norm(park_pos - t_pos)  # distances from the t parking spot
            if d_s < d_t and d_s < 0.2:
                park_type = nac.S_PARK
            elif d_t <= d_s and d_t < 0.2:
                park_type = nac.T_PARK
            else:
                self.error('ERROR: PARKING -> parking spot is not close to \
expected parking spot position!')
            self.curr_state.var1 = (park_state, park_type, True)
            self.curr_state.var4 = self.car.encoder_distance
            self.curr_state.just_switched = False

        park_state, park_type, just_changed = self.curr_state.var1

        #######################################################################
        # first state: localizing with precision the parking spot
        if park_state == nac.LOCALIZING_PARKING_SPOT:
            self.parking_localizing(just_changed, park_state, park_type)

        #######################################################################
        # second state: checking if there are parked cars in the parking spots
        elif park_state == nac.CHECKING_FOR_PARKED_CARS:
            self.parking_checking(just_changed, park_state, park_type)

        #######################################################################
        # STEP 0 -> ALIGN WITH THE PARKING SPOT
        elif park_state == nac.STEP0:
            self.parking_st0(just_changed, park_state, park_type)

        # T parking manouver
        elif park_state in [nac.T_STEP2,
                            nac.T_STEP3,
                            nac.T_STEP4,
                            nac.T_STEP5]:
            self.parking_t(just_changed, park_state, park_type)

        # S parking manouver
        elif park_state in [nac.S_STEP2,
                            nac.S_STEP3,
                            nac.S_STEP4,
                            nac.S_STEP5,
                            nac.S_STEP6,
                            nac.S_STEP7]:
            self.car.drive(speed=0.2, angle=0)
            sleep(1)
            self.car.drive_speed(speed=0.0)
            park.parallel_parking(self.car)

            self.curr_state.var1 = (nac.PARK_END, park_type, True)
            # self.parking_s(just_changed, park_state, park_type)

        # end of manouver, go to next event
        elif park_state == nac.PARK_END:
            self.parking_end()

    def parking_localizing(self, just_changed, park_state, park_type):
        print('LOCALIZING_PARKING_SPOT')
        self.activate_routines([nac.FOLLOW_LANE])

        if (self.parking_method == 'gps' or ALWAYS_USE_GPS_FOR_PARKING) \
                and not ALWAYS_USE_SIGN_FOR_PARKING:
            print('Using gps for parking')
            if just_changed:
                # this will become true if we trusted the gps
                # at least once. We will use local pos afterward
                trusted_gps_once = True
                self.curr_state.var2 = trusted_gps_once  # var2
                self.curr_state.var1 = (park_state, park_type, False)
                self.car.reset_rel_pose()

            trusted_gps_once = self.curr_state.var2

            if not self.conditions[nac.TRUST_GPS] and not trusted_gps_once:
                self.car.drive_distance(0.0)
                curr_time = time()
                passed_time = curr_time - self.curr_state.start_time
                if passed_time > PARK_MAX_SECONDS_W8_GPS:
                    print('GPS Timout!')
                    print('Using sign for parking...')
                    self.curr_state.var1 = (nac.LOCALIZING_PARKING_SPOT,
                                            park_type, True)
                    self.parking_method = 'sign'
                    raise KeyboardInterrupt
                print(f'Parking: GPS not trusted, \
waiting for GPS to be trusted for {passed_time}/{PARK_MAX_SECONDS_W8_GPS} \
[s]...')
            else:  # gps is trusted or we have already trusted it
                # we trusted gps once
                self.curr_state.var2 = trusted_gps_once = True
                car_est_pos = np.array([self.car.x_est, self.car.y_est])
                # one sample for every cm in the path
                park_index_on_path = int(self.next_event.dist*100)
                path_to_analyze = self.\
                    path_planner.path[max(0, park_index_on_path -
                                          SUBPATH_LENGTH_FOR_PARKING):
                                      min(park_index_on_path +
                                          SUBPATH_LENGTH_FOR_PARKING,
                                          len(self.path_planner.path))]
                car_idx_on_path = np.argmin(norm(path_to_analyze -
                                                 car_est_pos, axis=1))
                park_index_on_path = SUBPATH_LENGTH_FOR_PARKING
                if ALWAYS_DISTRUST_GPS:
                    car_idx_on_path = int((self.car.encoder_distance -
                                           self.curr_state.var4) * 100 +
                                          115 + 68)
                # print("path_to_analyze ", path_to_analyze)
                # print("car_est_pos ", car_est_pos)
                # print("car_idx_on_path ", car_idx_on_path)
                # print("park_index_on_path ", park_index_on_path)
                # print("self.car.dist_loc ", self.car.dist_loc)
                # print("MAX_PARK_SEARCH_DIST ", MAX_PARK_SEARCH_DIST)
                if car_idx_on_path < park_index_on_path and \
                        self.car.dist_loc < MAX_PARK_SEARCH_DIST:
                    print('Behind parking spot')
                    self.car.drive_speed(PARK_SEARCH_SPEED)
                    if car_idx_on_path > park_index_on_path - \
                            IDX_OFFSET_FROM_SAVED_PARK_POSITION:
                        print('We arrived at the parking spot')
                        self.car.drive_speed(0.0)
                        self.curr_state.var1 = (
                            nac.CHECKING_FOR_PARKED_CARS,
                            park_type, True)
                    else:
                        print(f'getting closer...  dist: \
{self.car.dist_loc:.2f}/{MAX_PARK_SEARCH_DIST:.2f}')
                else:
                    self.error('ERROR: PARKING: In front of parking spot, \
or maximum search distance reached')
        elif (self.parking_method == 'sign' or ALWAYS_USE_SIGN_FOR_PARKING
              ) and not ALWAYS_USE_GPS_FOR_PARKING:
            print('Using sign for parking')
            if just_changed:
                # create a deque of past PARK_SIGN_DETETCTION_PATIENCE
                # sing detection results
                park_sign_counter = 0
                # assign var2 to the queue
                self.curr_state.var2 = park_sign_counter
                # reset the car pose to the current pose
                self.car.reset_rel_pose()
                # set var1, with just_changed to false
                self.curr_state.var1 = (park_state, park_type, False)
                # parking sign reached or not
                self.curr_state.var3 = False
                self.car.drive_speed(PARK_SEARCH_SPEED)

            sign, _, _, _ = self.detect.detect_sign(self.car.frame,
                                                    show_ROI=True)
            park_sign_counter = self.curr_state.var2
            parking_spot_reached = self.curr_state.var3

            if not parking_spot_reached:
                if sign == 'park':
                    park_sign_counter += 1
                else:
                    park_sign_counter = 0
                if park_sign_counter >= PARK_SIGN_DETETCTION_PATIENCE:
                    self.curr_state.var3 = True  # parking sign reached
                    park_sign_counter = 0
                    print('Reached parking spot, keep going until the \
sign disappears')
            else:  # parking sign reached
                if sign != 'park':
                    park_sign_counter += 1
                if park_sign_counter >= PARK_SIGN_DETETCTION_PATIENCE:
                    print('Sign disappeared, setting up things for \
searching for parked cars')
                    self.car.drive_speed(0.0)
                    # go to next substate
                    self.curr_state.var1 = (nac.CHECKING_FOR_PARKED_CARS,
                                            park_type, True)
            self.curr_state.var2 = park_sign_counter

    def parking_checking(self, just_changed, park_state, park_type):
        print('Checking for parked cars...')
        if just_changed:
            self.activate_routines([nac.FOLLOW_LANE])
            assert self.next_event.name == nac.PARKING_EVENT
            self.car.reset_rel_pose()
            # (car in spot1, car in spot2, looked for car in spot1,
            #  looked for car in spot2)
            self.curr_state.var2 = (False, False, False, False)
            self.car.drive_speed(PARK_MANOUVER_SPEED)
            self.curr_state.var1 = (park_state, park_type, False)

        car_in_spot1, car_in_spot2, checked1, checked2 = self.\
            curr_state.var2
        curr_dist = self.car.dist_loc

        if park_type == nac.T_PARK:
            dist_first_spot = DIST_SIGN_FIRST_T_SPOT
            dist_spots = DIST_T_SPOTS
            further_dist = FURTHER_DIST_T
        elif park_type == nac.S_PARK:
            dist_first_spot = DIST_SIGN_FIRST_S_SPOT
            dist_spots = DIST_S_SPOTS
            further_dist = FURTHER_DIST_S
        else:
            self.error('ERROR: PARKING: Unknown parking type!')

        if (dist_first_spot < curr_dist < (dist_first_spot+0.1)) and not \
                checked1:
            self.car.drive_speed(0.0)
            sleep(SLEEP_AFTER_STOPPING)
            # get lateral sonar distance
            lateral_sonar_dist = self.car.filtered_lateral_sonar_distance
            checked1 = True
            if lateral_sonar_dist < 0.5:
                print('Car in spot 1')
                self.env.publish_obstacle(nac.STATIC_CAR_PARKING,
                                          self.car.x_est,
                                          self.car.y_est)
                car_in_spot1 = True
            self.car.drive_speed(PARK_MANOUVER_SPEED)
        elif (dist_first_spot+dist_spots < curr_dist <
                (dist_first_spot+dist_spots+0.1)) and not checked2:
            self.car.drive_speed(0.0)
            sleep(SLEEP_AFTER_STOPPING)
            # get lateral sonar distance
            lateral_sonar_dist = self.car.filtered_lateral_sonar_distance
            checked2 = True
            if lateral_sonar_dist < 0.5:
                print('Car in spot 2')
                self.env.publish_obstacle(nac.STATIC_CAR_PARKING,
                                          self.car.x_est,
                                          self.car.y_est)
                car_in_spot2 = True
            self.car.drive_speed(PARK_MANOUVER_SPEED)
        elif dist_first_spot+dist_spots+further_dist < curr_dist:
            self.car.drive_speed(0.0)
            sleep(SLEEP_AFTER_STOPPING)
            print('Ending search for parked cars')
            print('Car in spot 1: {}'.format(car_in_spot1))
            print('Car in spot 2: {}'.format(car_in_spot2))
            sleep(1)
            self.curr_state.var1 = (nac.STEP0, park_type, True)
        if dist_first_spot+dist_spots+further_dist + \
                MAX_ERROR_ON_LOCAL_DIST < curr_dist:
            overshoot_distance = dist_first_spot+dist_spots+further_dist +\
                    MAX_ERROR_ON_LOCAL_DIST-curr_dist
            self.error(f'ERROR: PARKING: CHECKING_CARS: \
Overshoot distance, error: {overshoot_distance:.2f}')

        # update var2 at the end of every iteration
        self.curr_state.var2 = (car_in_spot1, car_in_spot2,
                                checked1, checked2)

    def parking_st0(self, just_changed, park_state, park_type):
        # we are standing still at the end of the 2 parking spots
        print('STEP0 -> Aligning with the parking spot...')

        car_in_spot1, car_in_spot2, _, _ = self.curr_state.var2

        if just_changed:
            if car_in_spot1 and car_in_spot2:
                # <++>
                # self.error('ERROR: PARKING: Car in both spots!')
                self.switch_to_state(nac.LANE_FOLLOWING)
            elif not car_in_spot2:
                print('Spot 2 is free. Going to spot 2 now')
                park_state = nac.T_STEP2 if park_type == nac.T_PARK else \
                    nac.S_STEP2
                self.curr_state.var1 = (park_state, park_type, True)
            else:  # car in spot2, spot1 free
                print('Spot 1 is free. Going to spot 1 now')
                self.activate_routines([nac.FOLLOW_LANE])
                self.car.reset_rel_pose()
                self.car.drive_speed(-PARK_MANOUVER_SPEED)
                self.curr_state.var1 = (park_state, park_type, False)

        if not car_in_spot1 and car_in_spot2:
            dist = self.car.dist_loc
            print(f'Distance: {dist}')
            if park_type == nac.S_PARK:
                dist_spots = DIST_S_SPOTS
            else:
                dist_spots = DIST_T_SPOTS
            if dist > dist_spots:
                self.car.drive_speed(0.0)
                sleep(SLEEP_AFTER_STOPPING)
                print('Aligned with first parking spot')
                # sleep(1)  # Another sleep?
                park_state = nac.T_STEP2 if park_type == nac.T_PARK else \
                    nac.S_STEP2
                self.curr_state.var1 = (park_state, park_type, True)
            if dist > dist_spots + MAX_ERROR_ON_LOCAL_DIST:
                overshoot_distance = dist_spots + MAX_ERROR_ON_LOCAL_DIST \
                        - dist
                self.error(f'ERROR: PARKING: STEP0: Overshoot distance, \
error: {overshoot_distance:.2f}')

    def parking_t(self, just_changed, park_state, park_type):
        if park_state == nac.T_STEP2:
            self.parking_t2(just_changed, park_state, park_type)

        elif park_state == nac.T_STEP3:
            self.parking_t3(just_changed, park_state, park_type)

        elif park_state == nac.T_STEP4:
            self.parking_t4(just_changed, park_state, park_type)

        elif park_state == nac.T_STEP5:
            self.parking_t5(just_changed, park_state, park_type)

    def parking_t2(self, just_changed, park_state, park_type):
        print('T-parking manouver step 2, we are aligned with the parking \
spot, going right and backward')
        self.activate_routines([])
        if just_changed:
            self.car.drive_angle(+T_ANGLE)
            sleep(STEER_ACTUATION_DELAY_PARK)
            self.car.reset_rel_pose()
            self.car.drive_speed(-PARK_MANOUVER_SPEED)
            self.curr_state.var1 = (park_state, park_type, False)
        if self.car.dist_loc > DIST_2T:
            self.car.drive_speed(0.0)
            sleep(SLEEP_AFTER_STOPPING)
            self.curr_state.var1 = (nac.T_STEP3, park_type, True)
        if self.car.dist_loc > DIST_2T + MAX_ERROR_ON_LOCAL_DIST:
            overshoot_distance = DIST_2T + MAX_ERROR_ON_LOCAL_DIST - \
                    self.car.dist_loc
            self.error(f'ERROR: PARKING: T_STEP2: Overshoot distance, \
error:{overshoot_distance:.2f}')

    def parking_t3(self, just_changed, park_state, park_type):
        print('T-parking manouver step 3, going backward')
        print(f'Distance: {self.car.dist_loc:.2f}/{DIST_3T}')
        self.activate_routines([])
        if just_changed:
            self.car.drive_angle(0.0)
            sleep(STEER_ACTUATION_DELAY_PARK)
            self.car.reset_rel_pose()
            self.car.drive_speed(-PARK_MANOUVER_SPEED)
            self.curr_state.var1 = (park_state, park_type, False)
        if self.car.dist_loc > DIST_3T:
            self.car.drive_speed(0.0)
            sleep(SLEEP_AFTER_STOPPING)
            print('Parked')
            # <><><><><><><><><><><> PARK ACHIEVED <><><><><><><><><><><>
            self.achievements[nac.PARK_ACHIEVED] = True
            sleep(3.0)
            self.curr_state.var1 = (nac.T_STEP4, park_type, True)
            if self.car.dist_loc > DIST_3T + MAX_ERROR_ON_LOCAL_DIST:
                overshoot_distance = DIST_3T + MAX_ERROR_ON_LOCAL_DIST - \
                        self.car.dist_loc
                self.error(f'ERROR: PARKING: T_STEP3: Overshoot distance, \
error:{overshoot_distance:.2f}')

    # Once we parked we get back in lane
    def parking_t4(self, just_changed, park_state, park_type):
        print('T-parking manouver step 4, going forward')
        print(f'Distance: {self.car.dist_loc:.2f}/{DIST_3T}')
        self.activate_routines([])
        if just_changed:
            self.car.drive_angle(0.0)
            sleep(STEER_ACTUATION_DELAY_PARK)
            self.car.reset_rel_pose()
            self.car.drive_speed(PARK_MANOUVER_SPEED)
            self.curr_state.var1 = (park_state, park_type, False)
        if self.car.dist_loc > DIST_3T:
            self.car.drive_speed(0.0)
            sleep(SLEEP_AFTER_STOPPING)
            self.curr_state.var1 = (nac.T_STEP5, park_type, True)
        if self.car.dist_loc > DIST_3T + MAX_ERROR_ON_LOCAL_DIST:
            overshoot_distance = DIST_3T + MAX_ERROR_ON_LOCAL_DIST - \
                    self.car.dist_loc
            self.error(f'ERROR: PARKING: T_STEP4: Overshoot distance, \
error:{overshoot_distance:.2f}')

    def parking_t5(self, just_changed, park_state, park_type):
        print('T-parking manouver step 5, going right and forward')
        print(f'Distance: {self.car.dist_loc:.2f}/{DIST_2T}')
        self.activate_routines([])
        if just_changed:
            self.car.drive_angle(+T_ANGLE)
            sleep(STEER_ACTUATION_DELAY_PARK)
            self.car.reset_rel_pose()
            self.car.drive_speed(+PARK_MANOUVER_SPEED)
            self.curr_state.var1 = (park_state, park_type, False)
        if self.car.dist_loc > DIST_2T:
            self.car.drive_speed(0.0)
            sleep(SLEEP_AFTER_STOPPING)
            print('Back in lane')
            self.curr_state.var1 = (nac.PARK_END, park_type, True)
        if self.car.dist_loc > DIST_2T + MAX_ERROR_ON_LOCAL_DIST:
            overshoot_distance = DIST_2T + MAX_ERROR_ON_LOCAL_DIST - \
                    self.car.dist_loc
            self.error(f'ERROR: PARKING: T_STEP5: Overshoot distance, \
error:{overshoot_distance:.2f}')

    def parking_s(self, just_changed, park_state, park_type):
        if park_state == nac.S_STEP2:
            self.parking_s2(just_changed, park_state, park_type)

        elif park_state == nac.S_STEP3:
            self.parking_s3(just_changed, park_state, park_type)

        elif park_state == nac.S_STEP4:
            self.parking_s4(just_changed, park_state, park_type)

        elif park_state == nac.S_STEP5:
            self.parking_s5(just_changed, park_state, park_type)

        elif park_state == nac.S_STEP6:
            self.parking_s6(just_changed, park_state, park_type)

        elif park_state == nac.S_STEP7:
            self.parking_s7(just_changed, park_state, park_type)

    def parking_s2(self, just_changed, park_state, park_type):
        print('S-parking manouver step 2')
        print(f'Distance: {self.car.dist_loc:.2f}/{DIST_2S}')
        self.activate_routines([])
        if just_changed:
            self.car.drive_angle(+S_ANGLE)
            sleep(STEER_ACTUATION_DELAY_PARK)
            self.car.reset_rel_pose()
            self.car.drive_speed(-PARK_MANOUVER_SPEED)
            self.curr_state.var1 = (park_state, park_type, False)
        if self.car.dist_loc > DIST_2S:
            self.car.drive_speed(0.0)
            sleep(SLEEP_AFTER_STOPPING)
            self.curr_state.var1 = (nac.S_STEP3, park_type, True)
        if self.car.dist_loc > DIST_2S + MAX_ERROR_ON_LOCAL_DIST:
            overshoot_distance = DIST_2S + MAX_ERROR_ON_LOCAL_DIST - \
                    self.car.dist_loc
            self.error(f'ERROR: PARKING: S_STEP2: Overshoot distance, \
error:{overshoot_distance:.2f}')

    def parking_s3(self, just_changed, park_state, park_type):
        print('S-parking manouver step 3')
        print(f'Distance: {self.car.dist_loc:.2f}/{DIST_2S}')
        self.activate_routines([])
        if just_changed:
            self.car.drive_angle(-S_ANGLE)
            sleep(STEER_ACTUATION_DELAY_PARK)
            self.car.reset_rel_pose()
            self.car.drive_speed(-PARK_MANOUVER_SPEED)
            self.curr_state.var1 = (park_state, park_type, False)
        if self.car.dist_loc > DIST_2S:
            self.car.drive_speed(0.0)
            sleep(SLEEP_AFTER_STOPPING)
            self.curr_state.var1 = (nac.S_STEP4, park_type, True)
        if self.car.dist_loc > DIST_2S + MAX_ERROR_ON_LOCAL_DIST:
            overshoot_distance = DIST_2S + MAX_ERROR_ON_LOCAL_DIST - \
                    self.car.dist_loc
            self.error(f'ERROR: PARKING: S_STEP3: Overshoot distance, \
error:{overshoot_distance:.2f}')

    def parking_s4(self, just_changed, park_state, park_type):
        print('S-parking manouver step 4')
        print(f'Distance: {self.car.dist_loc:.2f}/{DIST_4S}')
        self.activate_routines([])
        if just_changed:
            self.car.drive_angle(0.0)
            sleep(STEER_ACTUATION_DELAY_PARK)
            self.car.reset_rel_pose()
            self.car.drive_speed(+PARK_MANOUVER_SPEED)
            self.curr_state.var1 = (park_state, park_type, False)
        if self.car.dist_loc > DIST_4S:
            self.car.drive_speed(0.0)
            sleep(SLEEP_AFTER_STOPPING)
            print('Parked')
            # <><><><><><><><><><><> PARK ACHIEVED <><><><><><><><><><><>
            self.achievements[nac.PARK_ACHIEVED] = True
            sleep(3.0)
            self.curr_state.var1 = (nac.S_STEP5, park_type, True)
        if self.car.dist_loc > DIST_4S + MAX_ERROR_ON_LOCAL_DIST:
            overshoot_distance = DIST_4S + MAX_ERROR_ON_LOCAL_DIST - \
                    self.car.dist_loc
            self.error(f'ERROR: PARKING: S_STEP4: Overshoot distance, \
error:{overshoot_distance:.2f}')

    def parking_s5(self, just_changed, park_state, park_type):
        print('S-parking manouver step 5')
        print(f'Distance: {self.car.dist_loc:.2f}/{DIST_4S}')
        self.activate_routines([])
        if just_changed:
            self.car.drive_angle(0.0)
            sleep(STEER_ACTUATION_DELAY_PARK)
            self.car.reset_rel_pose()
            self.car.drive_speed(-PARK_MANOUVER_SPEED)
            self.curr_state.var1 = (park_state, park_type, False)
        if self.car.dist_loc > DIST_4S:
            self.car.drive_speed(0.0)
            sleep(SLEEP_AFTER_STOPPING)
            print('Parked')
            self.curr_state.var1 = (nac.S_STEP6, park_type, True)
        if self.car.dist_loc > DIST_4S + MAX_ERROR_ON_LOCAL_DIST:
            overshoot_distance = DIST_4S + MAX_ERROR_ON_LOCAL_DIST - \
                    self.car.dist_loc
            self.error(f'ERROR: PARKING: S_STEP5: Overshoot distance, \
error:{overshoot_distance:.2f}')

    def parking_s6(self, just_changed, park_state, park_type):
        print('S-parking manouver step 6')
        print(f'Distance: {self.car.dist_loc:.2f}/{DIST_2S}')
        self.activate_routines([])
        if just_changed:
            self.car.drive_angle(-S_ANGLE)
            sleep(STEER_ACTUATION_DELAY_PARK)
            self.car.reset_rel_pose()
            self.car.drive_speed(+PARK_MANOUVER_SPEED)
            self.curr_state.var1 = (park_state, park_type, False)
        if self.car.dist_loc > DIST_2S:
            self.car.drive_speed(0.0)
            sleep(SLEEP_AFTER_STOPPING)
            self.curr_state.var1 = (nac.S_STEP7, park_type, True)
        if self.car.dist_loc > DIST_2S + MAX_ERROR_ON_LOCAL_DIST:
            overshoot_distance = DIST_2S + MAX_ERROR_ON_LOCAL_DIST - \
                    self.car.dist_loc
            self.error(f'ERROR: PARKING: S_STEP6: Overshoot distance, \
error:{overshoot_distance:.2f}')

    def parking_s7(self, just_changed, park_state, park_type):
        print('S-parking manouver step 7')
        print(f'Distance: {self.car.dist_loc:.2f}/{DIST_2S}')
        self.activate_routines([])
        if just_changed:
            self.car.drive_angle(+S_ANGLE)
            sleep(STEER_ACTUATION_DELAY_PARK)
            self.car.reset_rel_pose()
            self.car.drive_speed(+PARK_MANOUVER_SPEED)
            self.curr_state.var1 = (park_state, park_type, False)
        if self.car.dist_loc > DIST_2S:
            self.car.drive_speed(0.0)
            sleep(SLEEP_AFTER_STOPPING)
            self.curr_state.var1 = (nac.PARK_END, park_type, True)
        if self.car.dist_loc > DIST_2S + MAX_ERROR_ON_LOCAL_DIST:
            overshoot_distance = DIST_2S + MAX_ERROR_ON_LOCAL_DIST - \
                    self.car.dist_loc
            self.error(f'ERROR: PARKING: S_STEP7: Overshoot distance, \
error:{overshoot_distance:.2f}')

    def parking_end(self):
        self.switch_to_state(nac.LANE_FOLLOWING)
        self.go_to_next_event()

    def crosswalk_navigation(self):
        self.activate_routines([nac.CONTROL_FOR_OBSTACLES])
        if STOP_WAIT_TIME > 0.0:
            if self.curr_state.just_switched:
                self.car.drive_speed(0.0)
                self.curr_state.just_switched = False
            if (time() - self.curr_state.start_time) > STOP_WAIT_TIME:
                self.car.reset_rel_pose()
                self.go_to_next_event()
                self.switch_to_state(nac.LANE_FOLLOWING)
        else:
            self.car.reset_rel_pose()
            self.go_to_next_event()
            self.switch_to_state(nac.LANE_FOLLOWING)

    def classifying_obstacle(self):
        self.activate_routines([nac.FOLLOW_LANE])
        dist = self.car.filtered_sonar_distance
        if self.curr_state.just_switched:
            # drive to fixed dist from the obstacle
            self.car.drive_distance(dist - OBSTACLE_CONTROL_DISTANCE)
            self.curr_state.just_switched = False

        if OBSTACLE_DISTANCE_THRESHOLD <= dist:  #
            print('Sonar got confused: switch back to previous state')
            self.switch_to_prev_state()
        elif OBSTACLE_IMGS_CAPTURE_STOP_DISTANCE <= dist < \
                OBSTACLE_DISTANCE_THRESHOLD:  # we are approaching the obstacle
            print('Capturing imgs')
        else:
            frames = self.get_frames_in_range(
                    start_dist=OBSTACLE_IMGS_CAPTURE_START_DISTANCE -
                    OBSTACLE_IMGS_CAPTURE_STOP_DISTANCE)
            print(f'Captured {len(frames)} imgs, running classification...')
            # forcing the classification
            frame = self.car.frame
            img = frame.copy()
            img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
            img = img[40:120, 105:215]
            cv.imwrite('test.png', img)
            obstacle = obs.detect_obstacle(cv.imread("test.png"))

            # Previous logic for obstacle classfification

            # if self.conditions[nac.CAN_OVERTAKE]:
            #     obstacle = nac.CAR
            # elif self.conditions[nac.BUMPY_ROAD]:
            #     obstacle = nac.CAR
            # else:  # we cannot overtake and we are NOT in a bumpy road
            #     obstacle = nac.PEDESTRIAN
            # # check for roadblock
            # closest_node, dist_closest = self.path_planner.get_closest_node(
            #         np.array([self.car.x_est, self.car.y_est]))

            # print(f'Closest node: {closest_node}, dist={dist_closest:.2f}')
            # print(f'Car pos= {(self.car.x_est, self.car.y_est)}')
            # print(f'coords = {self.path_planner.get_coord(closest_node)}')
            # if closest_node in RB_NODES_LEFT_LANE or closest_node in \
            #         RB_NODES_RIGHT_LANE:
            #     obstacle = nac.ROADBLOCK

            print(f'Obstacle: {obstacle}')
            if OBSTACLE_IS_ALWAYS_CAR:
                obstacle = nac.CAR
            if OBSTACLE_IS_ALWAYS_PEDESTRIAN:
                obstacle = nac.PEDESTRIAN
            if OBSTACLE_IS_ALWAYS_ROADBLOCK:
                obstacle = nac.ROADBLOCK

            # TODO: add conditions to filter out false positives

            if self.checkpoints[self.checkpoint_idx] == 134:
                obstacle = nac.ROADBLOCK
            elif (self.conditions[nac.BUMPY_ROAD] or
                    self.conditions[nac.HIGHWAY]):
                obstacle = nac.CAR

            if obstacle == nac.CAR:
                self.switch_to_state(nac.TAILING_CAR)
            elif obstacle == nac.PEDESTRIAN:
                if self.next_event.name == nac.CROSSWALK_EVENT:
                    pedestrian_obstacle = nac.PEDESTRIAN_ON_CROSSWALK
                else:
                    pedestrian_obstacle = nac.PEDESTRIAN_ON_ROAD
                self.env.publish_obstacle(pedestrian_obstacle,
                                          self.car.x_est,
                                          self.car.y_est)
                self.switch_to_state(nac.WAITING_FOR_PEDESTRIAN)
            elif obstacle == nac.ROADBLOCK:
                self.switch_to_state(nac.AVOIDING_ROADBLOCK)
            else:
                self.error('ERROR: OBSTACLE CLASSIFICATION: Unknown obstacle')

    def brainless(self):
        self.activate_routines([])
        if self.curr_state.just_switched:
            self.curr_state.var1 = self.car.encoder_distance
            self.curr_state.just_switched = False

        if self.car.encoder_distance - self.curr_state.var1 < 0.5:
            e2, e3, point_ahead = self.detect.detect_lane(self.car.frame,
                                                          SHOW_IMGS)
            _, output_angle = self.controller.get_control(e2, e3, 0,
                                                          self.desired_speed)
            self.car.drive(speed=0.5,
                           angle=np.rad2deg(output_angle))
        elif self.car.encoder_distance - self.curr_state.var1 < BRAINLESS_MAXD:
            e3, _ = self.detect.detect_lane_ahead(self.car.frame,
                                                  show_ROI=SHOW_IMGS)
            output_speed, output_angle = self.controller_ag.get_control(e3)
            self.car.drive(speed=output_speed,
                           angle=np.rad2deg(output_angle))
        else:
            self.switch_to_state(nac.LANE_FOLLOWING)
            self.go_to_next_event()
            self.go_to_next_event()

    # =============== ROUTINES =============== #
    def follow_lane(self):
        e2, e3, point_ahead = self.detect.detect_lane(self.car.frame,
                                                      SHOW_IMGS)

        hf.show_follow_lane(self, point_ahead, SHOW_IMGS)
        _, angle_ref = self.controller.get_control(e2, e3, 0,
                                                   self.desired_speed)
        angle_ref = np.rad2deg(angle_ref)
        self.car.drive_angle(angle_ref)

    def detect_stopline(self):
        # update the variable self.detect.est_dist_to_stopline
        stopline_x, _, _ = self.detect.detect_stopline(
            self.car.frame, show_ROI=SHOW_IMGS)
        dist = stopline_x + 0.05

        past_detections = self.routines[nac.DETECT_STOPLINE].var2
        # -0.1 #network is more accurate in this range
        if dist < STOPLINE_APPROACH_DISTANCE-0.05:
            DETECTION_DEQUE_LENGTH = 50            # move to nac?
            SAMPLE_BEFORE_CONFIDENCE = 20          # move to nac?
            # var1 holds last detection time
            if self.routines[nac.DETECT_STOPLINE].var1 is not None:
                last_detection_time = self.routines[nac.DETECT_STOPLINE].var1
            else:
                last_detection_time = time() - 1.0
            curr_time = time()

            if curr_time - last_detection_time > 0.5:
                # var2 holds the list of past detections, reset
                self.routines[nac.DETECT_STOPLINE].var2 = past_detections = \
                        deque(maxlen=DETECTION_DEQUE_LENGTH)

            adapted_distance = dist + self.car.encoder_distance

            past_detections.append(adapted_distance)
            if len(past_detections) > SAMPLE_BEFORE_CONFIDENCE:
                # NOTE consider also mean
                self.stopline_distance_median = np.median(past_detections)
            else:
                self.stopline_distance_median = None
            self.routines[nac.DETECT_STOPLINE].var1 = curr_time
        else:
            self.stopline_distance_median = None

    def slow_down(self):
        if np.abs(self.car.filtered_encoder_velocity -
                  SLOW_DOWN_CONST*self.desired_speed) > 0.1:
            self.car.drive_speed(self.desired_speed*SLOW_DOWN_CONST)

    def accelerate(self):
        if np.abs(self.car.filtered_encoder_velocity <
                  ACCELERATION_CONST*self.desired_speed):
            self.car.drive_speed(ACCELERATION_CONST*self.desired_speed)

    def control_for_signs(self):
        # return #debug TODO remove this
        if SPEED_CHALLENGE:
            return
        prev_sign = self.curr_sign
        if not self.conditions[nac.REROUTING]:
            if self.conditions[nac.TRUST_GPS]:
                car_pos_on_path = self.path_planner.path[min(int(round(
                    self.car_dist_on_path*100)), len(self.path_planner.path-1
                                                     ))]
                distances = norm(self.sign_points-car_pos_on_path, axis=1)

                print(f'MIN DISTANCE = {np.min(distances)}')
                idx_close_signs = np.where(distances < SIGN_DIST_THRESHOLD)[0]
                if len(idx_close_signs) > 0:
                    for i in idx_close_signs:
                        if self.sign_seen[i] == 0:
                            self.sign_seen[i] = 1
                            print(f'SEEN SIGN \
{nac.SIGN_NAMES[self.sign_types[i]]}, at pos {self.sign_points[i]}')
                            self.curr_sign = nac.SIGN_NAMES[self.sign_types[i]]
                else:
                    self.curr_sign = nac.NO_SIGN

            else:  # Use signs                # not implemented? <++>
                return  # TODO remove it and use better detection
                sign, _ = self.detect.detect_sign(self.car.frame,
                                                  show_ROI=SHOW_IMGS,
                                                  show_kp=SHOW_IMGS)
                if sign != nac.NO_SIGN and sign != self.curr_sign:
                    self.curr_sign = sign

            # publish sign
            if self.curr_sign != prev_sign and self.curr_sign != nac.NO_SIGN:
                self.env.publish_obstacle(self.curr_sign,
                                          self.car.x_est,
                                          self.car.y_est)

    def control_for_obstacles(self):
        # check for obstacles
        if not SPEED_CHALLENGE:
            if self.routines[nac.CONTROL_FOR_OBSTACLES].var1 is not None:
                last_obstacle_dist = self.routines[
                        nac.CONTROL_FOR_OBSTACLES].var1
            else:
                last_obstacle_dist = self.car.encoder_distance - 1.0
            curr_dist = self.car.encoder_distance
            if curr_dist - last_obstacle_dist > MIN_DIST_BETWEEN_OBSTACLES:
                dist = self.car.filtered_sonar_distance

                if dist < OBSTACLE_DISTANCE_THRESHOLD:
                    self.car.drive_speed(speed=self.desired_speed/10)
                    # print('detecting obstacle ...')
                    # print(f'sonar dist: {self.car.filtered_sonar_distance}')

                    self.switch_to_state(nac.CLASSIFYING_OBSTACLE)
                    self.routines[nac.CONTROL_FOR_OBSTACLES].var1 = curr_dist

    def drive_desired_speed(self):
        if np.abs(self.car.filtered_encoder_velocity - self.desired_speed) > \
                0.1:
            self.car.drive_speed(self.desired_speed)

    # STATE CHECKS
    def check_logic(self):              # USELESS ?? <++>
        pass

    # UPDATE CONDITIONS
    def update_state(self):
        """"
        This will update the conditions at every iteration, it is called at
        the end of a self.run
        """
        # deque of past imgs
        print('UPDATE STATE')
        if self.routines[nac.UPDATE_STATE].var1 is not None:
            prev_dist = self.routines[nac.UPDATE_STATE].var1
        else:
            prev_dist = self.car.encoder_distance
        curr_dist = self.car.encoder_distance
        print(f'VAR1: {self.routines[nac.UPDATE_STATE].var1}')
        print(f'DISTANCE CHANGE: {prev_dist-curr_dist}')
        if np.abs(prev_dist-curr_dist) > DISTANCES_BETWEEN_FRAMES:
            self.past_frames.append(self.car.frame)
            prev_dist = curr_dist
        self.routines[nac.UPDATE_STATE].var1 = prev_dist

        # mirror trust gps from automobile_data
        self.conditions[nac.TRUST_GPS] = self.car.trust_gps and not \
            ALWAYS_DISTRUST_GPS or ALWAYS_TRUST_GPS

        if self.conditions[nac.TRUST_GPS]:
            est_pos = np.array([self.car.x_est, self.car.y_est])
            closest_node, _ = self.\
                path_planner.get_closest_node(est_pos)

            # HIGHWAY
            self.conditions[nac.HIGHWAY] = closest_node in \
                self.path_planner.highway_nodes

            # OVERTAKE/DOTTED_LINE
            self.conditions[nac.CAN_OVERTAKE] = self.\
                path_planner.is_dotted(closest_node)

            # BUMPY_ROAD
            was_bumpy = self.conditions[nac.BUMPY_ROAD]
            self.conditions[nac.BUMPY_ROAD] = closest_node in \
                self.path_planner.bumpy_road_nodes
            if self.conditions[nac.BUMPY_ROAD] and not was_bumpy:
                self.env.publish_obstacle(nac.BUMPY_ROAD,
                                          self.car.x_est,
                                          self.car.y_est)

            # REROUTING updated in start state

            if not self.conditions[nac.REROUTING]:
                # CAR_ON_PATH
                path = self.path_planner.path
                diff = norm(est_pos - path, axis=1)
                min_diff = np.min(diff)
                # TODO: IMPLEMENT THIS CASE -> global tracking
                if min_diff > MAX_DIST_AWAY_FROM_LANE:
                    self.conditions[nac.CAR_ON_PATH] = False
                else:
                    self.conditions[nac.CAR_ON_PATH] = True

                # UPDATING CAR PATH INDEX
                GPS_DELAY = 0.4  # [s]               # move to nac ??
                dist_delay_increment = GPS_DELAY * \
                    self.car.filtered_encoder_velocity  # - self.car.WB/2
                print(f'IDX DELAY INCREMENT: {dist_delay_increment}')
                if self.conditions[nac.CAR_ON_PATH]:
                    # NOTE assume step length is 0.01 #NOTE 2:
                    # Assumes no self loops in the path
                    self.car_dist_on_path = np.argmin(
                            norm(self.path_planner.path - est_pos,
                                 axis=1))*0.01
                    self.car_dist_on_path += dist_delay_increment
        else:
            # aposifjanwlkerhqwe;lrhoqweifeihqkugik
            self.car_dist_on_path += curr_dist - prev_dist
            # print('PATH: ', self.path_planner.path)
            print('DIST ON PATH: ', self.car_dist_on_path)
            was_bumpy = self.conditions[nac.BUMPY_ROAD]
            self.conditions[nac.BUMPY_ROAD] = str(self.checkpoints[
                self.checkpoint_idx]) \
                in self.path_planner.bumpy_road_nodes and \
                (self.next_event.name == nac.JUNCTION_EVENT or
                 self.car_dist_on_path < 13.2)
            self.conditions[nac.CAN_OVERTAKE] = not \
                self.conditions[nac.BUMPY_ROAD]

            self.conditions[nac.HIGHWAY] = str(self.checkpoints[
                self.checkpoint_idx]) \
                in self.path_planner.highway_nodes and \
                self.car_dist_on_path < 9.5

            if self.conditions[nac.BUMPY_ROAD] and not was_bumpy:
                self.env.publish_obstacle(nac.BUMPY_ROAD,
                                          self.car.x_est,
                                          self.car.y_est)

    # ===================== STATE MACHINE MANAGEMENT ===================== #
    def run(self):
        print('===============================================================\
===========')
        print(f'CHECKPOINT:     {self.checkpoints[self.checkpoint_idx]}')
        print(f'STATE:          {self.curr_state}')
        print(f'PREV_EVENT:     {self.prev_event}')
        print(f'UPCOMING_EVENT: {self.next_event}')
        print(f'ROUTINES:    {self.active_routines_names+ALWAYS_ON_ROUTINES}')
        print(f'CONDITIONS:  {self.conditions}')
        print(f'ENCODER:  {self.car.encoder_distance}')
        print('===============================================================\
===========')
        self.run_current_state()
        print(f'CURR_SIGN: {self.curr_sign}')
        print('===============================================================\
===========')
        print()
        self.run_routines()
        print('===============================================================\
===========')
        print()
        self.check_logic()

    def run_current_state(self):
        self.curr_state.run()

    def run_routines(self):
        for k, r in self.routines.items():
            if r.active:
                r.run()
        for k in ALWAYS_ON_ROUTINES:
            self.routines[k].run()

    def activate_routines(self, routines_to_activate):
        """
        routines_to_activate are a list of strings (routines)
        ex: ['follow_lane', 'control_for_signs']
        """
        assert all([r in self.routines.keys() for r in routines_to_activate]),\
            'ERROR: activate_routines: routines_to_activate contains \
invalid routine'
        self.active_routines_names = []
        for k, r in self.routines.items():
            r.active = k in routines_to_activate
            if r.active:
                self.active_routines_names.append(k)

    def add_routines(self, routines):
        """
        add routines to the active routines without overwriting the other
        ex: ['follow_lane', 'control_for_signs']
        """
        assert all([r in self.routines.keys() for r in routines]), \
            'ERROR: add_routines: routines_to_activate contains \
invalid routine'
        for k in routines:
            self.routines[k].active = True

    def switch_to_state(self, to_state, interrupt=False):
        """
        to_state is the string of the desired state to switch to
        ex: 'lane_following'
        """
        assert to_state in self.states, f'{to_state} is not a valid state'
        self.prev_state = self.curr_state
        self.curr_state = self.states[to_state]
        for k, s in self.states.items():
            s.active = k == to_state
        if not interrupt:
            self.curr_state.start_time = time()
            self.curr_state.start_position = np.array(
                    [self.car.x_est, self.car.y_est])  # maybe another position
            self.curr_state.start_distance = self.car.encoder_distance
            self.curr_state.interrupted = False
        else:
            self.curr_state.interrupted = True
        self.curr_state.just_switched = True

    # used only once, could be handeled differently <++>
    def switch_to_prev_state(self):
        self.switch_to_state(self.prev_state.name)

    def go_to_next_event(self):
        """
        Switches to the next event on the path
        """
        self.prev_event = self.next_event
        if self.event_idx == len(self.events):
            # no more events, for now
            pass
        else:
            self.next_event = self.events[self.event_idx]
            self.event_idx += 1

    def next_checkpoint(self):
        self.checkpoint_idx += 1
        # check if it's last
        if self.checkpoint_idx < (len(self.checkpoints)-1):
            # update events
            self.prev_event = self.next_event  # deepcopy(self.next_event)
            pass
        else:
            # it was the last checkpoint
            print('Reached last checkpoint...\nExiting...')
            enc_start = self.car.encoder_distance
            if self.checkpoints[-1] == 85:
                extra_dist = 0.5
            elif self.checkpoints[-1] == 123:
                extra_dist = 1.0
            else:
                extra_dist = 0.2
            while self.car.encoder_distance - enc_start < extra_dist:
                self.follow_lane()
                sleep(0.1)
            self.car.stop()
            sleep(3)
            cv.destroyAllWindows() if SHOW_IMGS else None
            exit()

    def create_sequence_of_events(self, events):
        """
        events is a list of strings (events)
        ex: ['lane_following', 'control_for_signs']
        """
        to_ret = []
        for e in events:
            name = e[0]
            dist = e[1]
            point = e[2]
            path_ahead = e[3]  # path in global coordinates
            if path_ahead is not None:
                loc_path = path_ahead - point
                # get yaw of the stopline
                assert path_ahead.shape[0] > 10, f'path_ahead is too short: \
{path_ahead.shape[0]}'
                path_first_10 = path_ahead[:10]
                diff10 = path_first_10[1:] - path_first_10[:-1]
                yaw_raw = np.median(np.arctan2(diff10[:, 1], diff10[:, 0]))
                yaw_stopline = hf.get_yaw_closest_axis(yaw_raw)
                loc_path = loc_path @ hf.rot_matrix(yaw_stopline)
                path_to_ret = loc_path
                curv = hf.get_curvature(path_ahead)
                print(f'yaw_stopline: {yaw_stopline}, name: \
{name}, curv: {curv}')
                len_path_ahead = 0.01*len(path_ahead)
            else:
                path_to_ret = None
                curv = None
                yaw_stopline = None
                len_path_ahead = None

            # going straight is ~0.15, right is ~ -0.15
            if name == nac.HIGHWAY_EXIT_EVENT and curv > 0.05:
                pass  # skip this case
            else:
                event = Event(name, dist, point, yaw_stopline, path_to_ret,
                              len_path_ahead, curv)
                to_ret.append(event)
        # add end of path event
        ee_point = self.path_planner.path[-1]
        end_event = Event(nac.END_EVENT, dist=0.0, point=ee_point)
        to_ret.append(end_event)
        return to_ret

    # Utility functions
    # should be moved to help functions? <++>
    def get_frames_in_range(self, start_dist, end_dist=0.0):
        len_past_frames = len(self.past_frames)
        print(f'len_past_frames: {len_past_frames}')
        idx_start = int(round(start_dist/DISTANCES_BETWEEN_FRAMES))
        idx_end = int(round(end_dist/DISTANCES_BETWEEN_FRAMES))
        print(f'idx_start: {idx_start}, idx_end: {idx_end}')
        assert idx_start < len_past_frames and idx_end >= 0
        # reverse the idxs
        idx_start = len_past_frames-1 - idx_start
        idx_end = len_past_frames-1 - idx_end
        print(f'idx_start: {idx_start}, idx_end: {idx_end}')
        # convert self.past_frames to a list
        past_frames = list(self.past_frames)
        return past_frames[idx_start:idx_end]

    # DEBUG
    def error(self, error_msg):
        print(error_msg)
        self.car.stop()
        sleep(3)
        exit()
