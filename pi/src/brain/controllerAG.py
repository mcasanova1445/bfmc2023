#!/usr/bin/python3
import numpy as np
import os

from helper_functions import *
import time
from scipy.signal import butter, lfilter, lfilter_zi
import collections
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt


class ControllerSpeed():
    def __init__(self, 
                 wheelbase = 0.26,
                 straight_speed=0.5,
                 curve_speed=0.2,
                 ac_max=0.1, 
                 lookahead=0.8,
                 Kpp_straight = 1.0,
                 Kpp_curve = 2.0,
                 Kd = 0.1,
                 alpha_straight = 3,
                 alpha_curve = 11):
        # ============================================================
        # CLASS VARIABLES
        # ============================================================
        self.Ld = lookahead
        self.wb = wheelbase
        self.alpha      = 0.0
        self.alpha_prev = 0.0
        self.alpha_rate = 0.0
        self.prev_time  = None
        self.ac_max = ac_max

        # ============================================================
        # PURE PURSUIT - PP-D 
        # ============================================================
        self.Kpp_straight = Kpp_straight
        self.Kpp_curve = Kpp_curve
        self.Kd = Kd 
        self.Kd_straight = self.Kd
        self.Kd_curve = 1.5*self.Kd                

        # ============================================================
        # SPEED and GAIN PROFILES
        # ============================================================
        self.alpha_straight = np.deg2rad(alpha_straight)
        self.alpha_curve = np.deg2rad(alpha_curve)

        self.straight_speed = straight_speed
        self.curve_speed = curve_speed

        self.speed_profile_s2c = CubicSpline([self.alpha_straight, self.alpha_curve],
                                             [self.straight_speed, self.curve_speed], 
                                             bc_type=((1, 0.0), (1, 0.0))
                                            )
                                            
        self.gain_profile_s2c = CubicSpline([self.alpha_straight, self.alpha_curve],
                                             [self.Kpp_straight, self.Kpp_curve], 
                                             bc_type=((1, 0.0), (1, 0.0))
                                            )
        
        self.der_profile_s2c = CubicSpline([self.alpha_straight, self.alpha_curve],
                                             [self.Kd_straight, self.Kd_curve], 
                                             bc_type=((1, 0.0), (1, 0.0))
                                            )
        
        # ============================================================
        # PLOT PROFILES
        # ============================================================
        # x_new = np.linspace(self.alpha_straight, self.alpha_curve,100)
        # y_new = self.speed_profile_s2c(x_new)
        # x_neww = np.linspace(self.alpha_straight, self.alpha_curve,100)
        # y_neww = self.gain_profile_s2c(x_neww)
        # plt.figure(figsize = (10,8))
        # plt.plot(x_new, y_new, 'b')
        # plt.plot(x_neww, y_neww, 'b')
        # plt.title('Cubic Spline Interpolation')
        # plt.xlabel('x')
        # plt.ylabel('y')
        # plt.show()

    def get_control(self, alpha):
        curr_time = time.time()
        if self.prev_time is None:
            self.prev_time = curr_time
            return 0.0,0.0
        else:
            DT = curr_time - self.prev_time         # [s] - sampling time

            # ============================================================
            # MEASUREMENTS
            # ============================================================
            self.alpha = alpha                                          # [rad] - LHE long
            self.alpha_rate = (self.alpha - self.alpha_prev)/DT  # [rad/s] - LHE long rate
            
            # ============================================================
            # SPEED and GAIN PROFILES
            # ============================================================
            if abs(self.alpha) < self.alpha_straight:
                output_speed = self.straight_speed
                gain = self.Kpp_straight
                gain_der = self.Kd_straight
            elif self.alpha_straight <= abs(self.alpha) < self.alpha_curve:
                output_speed = self.speed_profile_s2c(abs(self.alpha))
                gain = self.gain_profile_s2c(abs(self.alpha))
                gain_der = self.der_profile_s2c(abs(self.alpha))
            else:
                output_speed = self.curve_speed
                gain = self.Kpp_curve
                gain_der = self.Kd_curve
            
            output_speed = min(self.straight_speed, np.sqrt(self.ac_max*self.Ld/(2*np.sin(np.abs(self.alpha)))))
            # gain_der = min(self.Kd, self.Kd/self.straight_speed*np.sqrt(self.ac_max*self.Ld/(2*np.sin(np.abs(self.alpha)))))
            gain_der = self.Kd

            # ============================================================
            # PP-D CONTROL LAW
            # ============================================================
            # delta_pp = - (gain * np.arctan((2*self.wb*np.sin(self.alpha))/(self.Ld)) + self.Kd*self.alpha_rate)
            delta_pp = - (gain * np.arctan((2*self.wb*np.sin(self.alpha))/(self.Ld)) + gain_der*self.alpha_rate)


            # ============================================================
            # SATURATION AND OUTPUT
            # ============================================================
            output_angle = delta_pp
            if output_angle > np.deg2rad(28):
                output_angle = np.deg2rad(28)
            if output_angle < np.deg2rad(-28):
                output_angle = np.deg2rad(-28)

            self.prev_time = curr_time
            self.alpha_prev = self.alpha

            # ============================================================
            # DEBUG
            # ============================================================
            # os.system('cls' if os.name=='nt' else 'clear')
            # print(f'DT is: {DT}')
            # print(f'output_speed is: {output_speed}')
            # print(f'output angle is: {np.rad2deg(output_angle)}')
            # print(f'alpha_long is {np.rad2deg(self.alpha_long)}')
            # print(f'alpha_short is {np.rad2deg(self.alpha_short)}')
            # print(f'alpha_long_rate is {np.rad2deg(self.alpha_long_rate)}')
            # print(f'alpha rate is : {np.rad2deg(alpha_rate)} rad/s')

        return output_speed, output_angle

class Controller():
    def __init__(self, 
                 wheelbase = 0.26,
                 speed = 0.3,
                 lookahead=0.5,
                 Kpp = 1.0):
        # ============================================================
        # CLASS VARIABLES
        # ============================================================
        self.Ld         = lookahead
        self.wb         = wheelbase
        self.alpha      = 0.0
        self.speed      = speed

        # ============================================================
        # PURE PURSUIT - PP
        # ============================================================
        self.Kpp = Kpp                    

    def get_control(self, alpha):
        # ============================================================
        # MEASUREMENTS
        # ============================================================
        self.alpha = alpha                                          # [rad] - LHE short

        # ============================================================
        # PP-D CONTROL LAW
        # ============================================================
        delta_pp = - self.Kpp * np.arctan((2*self.wb*np.sin(self.alpha))/(self.Ld))

        # ============================================================
        # SATURATION AND OUTPUT
        # ============================================================
        output_angle = delta_pp
        output_speed = self.speed
        if output_angle > np.deg2rad(28):
            output_angle = np.deg2rad(28)
        if output_angle < np.deg2rad(-28):
            output_angle = np.deg2rad(-28)

        # ============================================================
        # DEBUG
        # ============================================================
        # os.system('cls' if os.name=='nt' else 'clear')
        # print(f'DT is: {DT}')
        # print(f'output_speed is: {output_speed}')
        # print(f'output angle is: {np.rad2deg(output_angle)}')
        # print(f'alpha_long is {np.rad2deg(self.alpha_long)}')
        # print(f'alpha_short is {np.rad2deg(self.alpha_short)}')
        # print(f'alpha_long_rate is {np.rad2deg(self.alpha_long_rate)}')
        # print(f'alpha rate is : {np.rad2deg(alpha_rate)} rad/s')

        return output_speed, output_angle
        