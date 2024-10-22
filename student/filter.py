# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Kalman filter class
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np

# add project directory to python path to enable relative imports
import os
import sys
import misc.params as params 
from .trackmanagement import Track
from .measurements import Measurement

PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))


class Filter:
    '''Kalman filter class'''
    def __init__(self):
        pass

    def F(self):
        ############
        # TODO Step 1: implement and return system matrix F
        ############
        dt = params.dt
        
        return np.matrix([[1, 0 ,0, dt, 0 ,0],
                          [0, 1, 0, 0, dt, 0],
                          [0, 0, 1, 0, 0, dt],
                          [0, 0, 0, 1, 0, 0],
                          [0, 0, 0, 0, 1, 0],
                          [0, 0, 0, 0, 0, 1]])
        ############
        # END student code
        ############ 

    def Q(self):
        ############
        # TODO Step 1: implement and return process noise covariance Q
        ############
        q = params.q
        dt = params.dt
        q1 = ((dt ** 3) / 3) * q
        q2 = ((dt ** 2) / 2) * q
        q3 = dt * q
        return np.matrix([[q1, 0, 0, q2, 0, 0],
                          [0, q1, 0, 0, q2, 0],
                          [0, 0, q1, 0, 0, q2],
                          [q2, 0, 0, q3, 0, 0],
                          [0, q2, 0, 0, q3, 0],
                          [0, 0, q2, 0, 0, q3]
                          ])
        
        ############
        # END student code
        ############ 

    def predict(self, track: Track):
        ############
        # TODO Step 1: predict state x and estimation error covariance P to next timestep, save x and P in track
        ############
        x = self.F() * track.x
        P = self.F()*track.P*self.F().transpose() + self.Q()
        track.set_x(x)
        track.set_P(P)
        
        ############
        # END student code
        ############ 

    def update(self, track: Track, meas: Measurement):
        ############
        # TODO Step 1: update state x and covariance P with associated measurement, save x and P in track
        ############
        H = meas.sensor.get_H(track.x)
        gamma = self.gamma(track, meas)
        S = self.S(track, meas, H)
        K = track.P * H.transpose() *np.linalg.inv(S)
        x = track.x + K*gamma
        I = np.identity(params.dim_state)
        P = (I - K*H)*track.P
        track.set_x(x)
        track.set_P(P)
        ############
        # END student code
        ############ 
        track.update_attributes(meas)
    
    def gamma(self, track: Track, meas: Measurement):
        ############
        # TODO Step 1: calculate and return residual gamma
        ############
        z = meas.z
        Hx = meas.sensor.get_hx(track.x)
        return z - Hx
        
        ############
        # END student code
        ############ 

    def S(self, track: Track, meas: Measurement, H):
        ############
        # TODO Step 1: calculate and return covariance of residual S
        ############
        P = track.P
        H = meas.sensor.get_H(track.x)
        R = meas.R
        return H*P*H.transpose() + R
        
        ############
        # END student code
        ############ 