#!/usr/bin/python3
import numpy as np

import time
from scipy.signal import butter, lfilter, lfilter_zi
from scipy.interpolate import CubicSpline

POINT_AHEAD_CM_SHORT = 35   # [points,cm] distance of the short point ahead
POINT_AHEAD_CM_LONG = 60+80    # [points,cm] distance of the long point ahead
MIN_ALPHA_RATE = np.deg2rad(20)        # [rad/s]
WB = 0.26   # [m]: wheelbase length


class Filter():
    def __init__(self, type='low', normal_cutoff=0.01, order=2) -> None:
        """ Discrete butterworth filter """

        self.normal_cutoff = normal_cutoff
        self.order = order
        self.ic = None
        if type == 'low' or type == 'high':
            self.type = type
        else:
            raise Exception('invalid filter type')
        self.update_filter_coefficients()

    def filter(self, data):
        if self.ic is None:
            self.ic = lfilter_zi(self.b, self.a) * data[0]
        else:
            self.ic = self.ic
        y, self.ic = lfilter(self.b, self.a, data, zi=self.ic)
        return y

    def update_filter_coefficients(self):
        self.b, self.a = butter(self.order, self.normal_cutoff,
                                btype=self.type, analog=False)

    def update_params(self, type=None, normal_cutoff=None, order=None):
        if normal_cutoff is not None:
            self.normal_cutoff = normal_cutoff
        else:
            self.normal_cutoff = self.normal_cutoff
        self.order = order if order is not None else self.order
        if type is not None:
            if type == 'low' or type == 'high':
                self.type = type
            else:
                raise Exception('invalid filter type')
        else:
            pass
        self.update_filter_coefficients()


class ControllerSpeed():
    def __init__(self, desired_speed=0.5, curve_speed=0.2):
        self.L_short = POINT_AHEAD_CM_SHORT/100.0
        self.L_long = POINT_AHEAD_CM_LONG/100.0
        self.l_f = WB/2          # [m]: distance of camera/CM from front axis
        self.curvature_ahead = 0.0
        self.path_ahead = None
        self.prev_time = None

        # ----- PURE PURSUIT -----
        self.Kpp_straight = 2.0
        self.Kpp_curve = 3.0

        self.Kd = 0.2

        # ----- LONGITUDINAL -----
        self.desired_speed = desired_speed
        self.curve_speed = curve_speed

        self.alpha_straight = np.deg2rad(1)
        self.alpha_curve = np.deg2rad(11)

        self.alpha_rate_straight = np.deg2rad(1.0)
        self.alpha_rate_curve = np.deg2rad(20)

        self.speed_profile_s2c = CubicSpline([self.alpha_straight,
                                              self.alpha_curve],
                                             [self.desired_speed,
                                              self.curve_speed],
                                             bc_type=((1, 0.0), (1, 0.0)))
        self.gain_profile_s2c = CubicSpline([self.alpha_straight,
                                             self.alpha_curve],
                                            [self.Kpp_straight,
                                             self.Kpp_curve],
                                            bc_type=((1, 0.0), (1, 0.0))
                                            )

        self.ey = 0.0
        self.ey_int = 0.0

        self.alpha_short = 0.0
        self.alpha_long = 0.0

        self.alpha_long_prev = 0.0
        self.alpha_long_rate = 0.0

    def get_control_speed(self, short_e2, short_e3, long_e3):
        curr_time = time.time()
        if self.prev_time is None:
            self.prev_time = curr_time
            return 0.0, 0.0
        else:
            DT = curr_time - self.prev_time

            # ----- MEASUREMENTS -----
            self.ey = short_e2   # lateral error
            self.alpha_short = -short_e3    # heading error - short
            self.alpha_long = -long_e3      # heading error - long
            self.alpha_long_rate = (self.alpha_long - self.alpha_long_prev)/DT

            # ----- VARIABLE GAIN and SPEED PROFILES -----
            if abs(self.alpha_long) < self.alpha_straight:
                output_speed = self.desired_speed
                gain = self.Kpp_straight
            elif self.alpha_straight <= abs(self.alpha_long) < \
                    self.alpha_curve:
                output_speed = self.speed_profile_s2c(abs(self.alpha_long))
                gain = self.gain_profile_s2c(abs(self.alpha_long))
            else:
                output_speed = self.curve_speed
                gain = self.Kpp_curve

            delta_pp = gain * np.arctan(
                    (2*WB*np.sin(self.alpha_long))/(self.L_long)) + \
                self.Kd*self.alpha_long_rate

            # ===== OUTPUT ANGLE =====
            output_angle = delta_pp  # + delta_i

            if output_angle > np.deg2rad(28):
                output_angle = np.deg2rad(28)
            if output_angle < np.deg2rad(-28):
                output_angle = np.deg2rad(-28)

            self.prev_time = curr_time
            self.alpha_long_prev = self.alpha_long

        return output_speed, output_angle
