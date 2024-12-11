import numpy as np
from utils import (approx__leq, approx_geq)

def velocityFunction(v0, vmax, t_delta, t):    
    if vmax < 0:
        vt = max(v0 + (vmax - v0)/t_delta * t, vmax)
    else:
        vt = min(v0 + (vmax - v0)/t_delta * t, vmax)
    return vt

def positionFunction(p0, v0, vmax, t_delta, t):
    if t < t_delta:
        vt = velocityFunction(v0, vmax, t_delta, t)
        pt = p0 + (v0 + vt)*t/2.0
    else:
        pt = p0 + (v0 + vmax)*t_delta/2.0
        pt += (vmax * (t - t_delta))        
    return pt

def positionRotationFunction(theta_0, omega_0, omega_max, t_delta, t):
    if t < t_delta:
        omega_t = velocityFunction(omega_0, omega_max, t_delta, t)
        theta_t = theta_0 + (omega_0 + omega_t)*t/2.0
    else:
        theta_t = theta_0 + (omega_0 + omega_max)*t_delta/2.0
        theta_t += (omega_max * (t - t_delta))        
    return theta_t

def velocityRotationFunction(omega_0, omega_max, t_delta, t):    
    if omega_max < 0:
        omega_t = max(omega_0 + (omega_max - omega_0)/t_delta * t, omega_max)
    else:
        omega_t = min(omega_0 + (omega_max - omega_0)/t_delta * t, omega_max)
    return omega_t


class CoMTrajectoryPlanner:
    def __init__(self,
                 xinit = 0.0, yinit = 0.0, zinit = 0.28,
                 vxdes = 0.0, vydes = 0.0, zdes = 0.28,
                 yaw_init = 0.0, yaw_rate_des = 0.0) -> None:
        self.xinit_ = xinit
        self.yinit_ = yinit
        self.zinit_ = zinit
        self.vxdes_ = vxdes
        self.vydes_ = vydes
        self.zdes_ = zdes
        self.yaw_des_ = 0.0  
        self.yaw_init_ = yaw_init  # robot rotate about z-axis
        self.yaw_rate_des_ = yaw_rate_des  # robot rotate about z-axis
        self.pitch_des_ = 0.0
        self.roll_des_ = 0.0
        self.transition_time_ = 0.0
    
    def setInitialCondition(self, xinit, yinit, zinit, yaw_init):
        self.xinit_ = xinit
        self.yinit_ = yinit
        self.zinit_ = zinit
        self.yaw_init_ = yaw_init

    def setTargets(self, vxdes = 0.0, vydes = 0.0, zdes = 0.28, yaw_rate_des = 0.0):
        self.vxdes_ = vxdes
        self.vydes_ = vydes
        self.zdes_ = zdes
        self.yaw_rate_des_ = yaw_rate_des

    def setTransitionTime(self, transition_time):
        self.transition_time_ = transition_time

    def getCoMState(self, t):
        com_vel = self.getCoMVelocity(t)
        com_pos = self.getCoMPosition(t)
        return np.hstack((com_pos, com_vel))

    def getCoMVelocity(self, t):
        vx_t = velocityFunction(0.0, self.vxdes_, self.transition_time_, t)
        vy_t = velocityFunction(0.0, self.vydes_, self.transition_time_, t)
        vz_t = 0.0     
        return np.array([vx_t, vy_t, vz_t])

    def getEulRate(self, t):
        yaw_rate_t = velocityRotationFunction(0.0, self.yaw_rate_des_, self.transition_time_, t)
        pitch_rate_t = 0.0
        roll_rate_t = 0.0
        return np.array([yaw_rate_t, pitch_rate_t, roll_rate_t]) 

    def getCoMPosition(self, t):
        vx_t = velocityFunction(0.0, self.vxdes_, self.transition_time_, t)
        vy_t = velocityFunction(0.0, self.vydes_, self.transition_time_, t)
        x_t = positionFunction(self.xinit_, vx_t, self.vxdes_, self.transition_time_, t)
        y_t = positionFunction(self.yinit_, vy_t, self.vydes_, self.transition_time_, t)
        z_t = min(self.zinit_ + (self.zdes_-self.zinit_)/self.transition_time_*t, self.zdes_)       
        return np.array([x_t, y_t, z_t])
    
    def getEuler(self, t):
        yaw_rate_t = velocityRotationFunction(0.0, self.yaw_rate_des_, self.transition_time_, t)
        yaw_t = positionRotationFunction(self.yaw_init_, yaw_rate_t, self.yaw_rate_des_, self.transition_time_, t)
        pitch_t = 0.0
        roll_t = 0.0
        return np.array([yaw_t, pitch_t, roll_t])



