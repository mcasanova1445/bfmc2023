#!/usr/bin/python3
from brain import SIMULATOR_FLAG, SPEED_CHALLENGE, SHOW_IMGS

import os
import signal
import cv2 as cv
import rospy
import numpy as np
from time import sleep, time
if SIMULATOR_FLAG:
    from automobile_data_simulator import AutomobileDataSimulator
    import helper_functions as hf
else:  # PI
    from control.automobile_data_pi import AutomobileDataPi
    import control.helper_functions as hf

from path_planning4 import PathPlanning
from controller3 import Controller
from controllerSP import ControllerSpeed
from controllerAG import ControllerSpeed as ControllerBL
from detection import Detection
from brain import Brain
from environmental_data_simulator import EnvironmentalData

os.system('clear')
print('Main brain starting...')

track = cv.imread('data/2021_VerySmall.png')

# PARAMETERS
TARGET_FPS = 30.0
sample_time = 0.01  # [s]

if not SPEED_CHALLENGE:
    DESIRED_SPEED = 0.35  # [m/s]
    SP_SPEED = 0.35  # [m/s]
    CURVE_SPEED = 0.25  # [m/s]
    BL_SP_SPEED = 0.8
    BL_CURVE_SPEED = 0.5
else:
    DESIRED_SPEED = 0.4  # [m/s]
    SP_SPEED = 0.6  # [m/s]
    CURVE_SPEED = 0.4  # [m/s]
    BL_SP_SPEED = 0.8
    BL_CURVE_SPEED = 0.4

path_step_length = 0.01  # [m]
# CONTROLLER
k1 = 0.0  # 0.0 gain error parallel to direction (speed)
k2 = 0.0  # 0.0 perpenddicular error gain
# pure paralllel k2 = 10 is very good at staying in the center of the lane
k3 = 0.7  # 1.0 yaw error gain .8 with ff
k3D = 0.08  # 0.08 derivative gain of yaw error
dist_point_ahead = 0.35  # distance of the point ahead in m

# dt_ahead = 0.5
# [s] how far into the future the curvature is estimated,
# feedforwarded to yaw controller
ff_curvature = 0.0  # feedforward gain

SEXY_FLAG = True
x_orig = 0.0
y_orig = 0.0

# load camera with opencv
if not SIMULATOR_FLAG:  # <++>
    cap = cv.VideoCapture(0)
    cap.set(cv.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, 240)
    cap.set(cv.CAP_PROP_FPS, 30)


# stop the car with ctrl+c
def handler(signum, frame):
    print("Exiting ...")
    car.stop()
    if SIMULATOR_FLAG:
        os.system('rosservice call gazebo/pause_physics')
    cv.destroyAllWindows()
    sleep(.99)
    exit()


if __name__ == '__main__':

    hf.create_frames(SHOW_IMGS)

    # init the car data
    if SIMULATOR_FLAG:
        os.system('rosservice call /gazebo/reset_simulation')
        os.system('rosservice call gazebo/unpause_physics')
        car = AutomobileDataSimulator(trig_cam=True,
                                      trig_gps=True,
                                      trig_bno=True,
                                      trig_enc=True,
                                      trig_control=True,
                                      trig_estimation=False,
                                      trig_sonar=True)  # <++>
    else:
        car = AutomobileDataPi(trig_cam=False,
                               trig_gps=True,
                               trig_bno=True,
                               trig_enc=True,
                               trig_control=True,
                               trig_estimation=True,
                               trig_sonar=True)
    sleep(1.5)
    car.encoder_distance = 0.0

    signal.signal(signal.SIGINT, handler)

    # init trajectory
    path_planner = PathPlanning(track)

    # init env
    env = EnvironmentalData(trig_v2v=True, trig_v2x=True, trig_semaphore=True)

    # init controller
    controller = Controller(k1=k1, k2=k2, k3=k3, k3D=k3D,
                            dist_point_ahead=dist_point_ahead,
                            ff=ff_curvature)
    controller_sp = ControllerSpeed(desired_speed=SP_SPEED,
                                    curve_speed=CURVE_SPEED)
    controller_ag = ControllerBL(straight_speed=BL_SP_SPEED,
                                 curve_speed=BL_CURVE_SPEED,
                                 lookahead=0.8)

    # initiliaze all the neural networks for detection and lane following
    detect = Detection()

    # initiliaze the brain
    brain = Brain(car=car, controller=controller, controller_sp=controller_sp,
                  controller_ag=controller_ag,
                  detection=detect, env=env, path_planner=path_planner,
                  desired_speed=DESIRED_SPEED)

    hf.show_track(track, car, SHOW_IMGS)

    try:
        car.stop()
        fps_avg = 0.0
        fps_cnt = 0
        while not rospy.is_shutdown():

            loop_start_time = time()
            # clear the screen
            print("\033c")

            # <++++++++++++++++++++>
            hf.show_car(track, car, brain, SHOW_IMGS)

            if not SIMULATOR_FLAG:
                ret, frame = cap.read()
                brain.car.frame = frame
                if not ret:
                    print("No image from camera")
                    frame = np.zeros((240, 320, 3), np.uint8)
                    continue

            # RUN BRAIN
            brain.run()

            # DEBUG INFO
            print(f'Lane detection time = \
                    {detect.avg_lane_detection_time:.1f} [ms]')
            # print(f'Sign detection time = \
            #         {detect.avg_sign_detection_time:.1f} [ms]')
            print(f'FPS = {fps_avg:.1f},  \
                    loop_cnt = {fps_cnt}, capped at {TARGET_FPS}')
            print('VALIDATION: ', brain.start_node_validated)
            print('PATH: ', brain.path_planner.route_graph.nodes)
            # if SEXY_FLAG:
            #     x_orig = car.x_est
            #     y_orig = car.y_est
            #     SEXY_FLAG = False
            # print(f'POSITION: X_EST = {car.x_est}, Y_EST = {car.y_est}')
            # print(f'POSITION: X_DIST = \
            #         {car.x_est - x_orig}, Y_DIST = {car.y_est - y_orig}')
            # if np.abs(x_orig - car.x_est) > 0.32 or \
            #         np.abs(y_orig - car.y_est) > 0.32:
            #     car.drive_speed(0.0)
            #     car.drive_angle(0.0)
            #     raise

            hf.show_camera(car, brain, SHOW_IMGS)

            if SHOW_IMGS:
                if cv.waitKey(1) == 27:
                    cv.destroyAllWindows()
                    break

            loop_time = time() - loop_start_time
            fps_avg = (fps_avg * fps_cnt + 1.0 / loop_time) / (fps_cnt + 1)
            fps_cnt += 1
            if loop_time < 1.0 / TARGET_FPS:
                sleep(1.0 / TARGET_FPS - loop_time)

    except KeyboardInterrupt:
        print("Shutting down")
        car.stop()
        sleep(.5)
        cv.destroyAllWindows()
        exit(0)
    except rospy.ROSInterruptException:
        pass
