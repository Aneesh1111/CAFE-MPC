from dataclasses import dataclass
import sys
sys.path.append('/home/robocup/Documents/CAFE-MPC/')
from gait_schedule import GaitSchedule, Stance, Trot
from gait_schedule import Bound
from gait_schedule import FlyTrot
from gait_schedule import Pace
from gait_schedule import FlyPace
from gait_schedule import Pronk
from gait_schedule import Jump_backwards
from gait_schedule import Jump_forwards
from gait_schedule import Sideways_Jump_negativeVy
from gait_schedule import Sideways_Jump_positiveVy
from gait_schedule import Jump_Rotate_positive_yawRate
from reference_management import ReferenceManager
import utils
import numpy as np
from mini_cheetah_pybullet import MiniCheetah
import copy


# Desired Trajectories
xinit, yinit, zinit, yaw_init = 0.0, 0.0, 0.24, 0.0
vx_des, vy_des, z_des, yaw_rate_des = 0.0, 0.0, 0.24, 1*2*3.14
swingHeight = 0.12

transition_time = 10
dt = 0.01

# Desired Gait
endGait = copy.copy(Stance)
endGait.switchingTimes = np.array([0.0, 0.02])


# Define a jump gait
# Jump = copy.deepcopy(Jump)
# Jump.switchingTimes = np.array([0.0, 0.1, 0.16, 0.5, 1.12])

# Define gait schedule
gaitScheule = GaitSchedule() # empty gait shcedule
gaitScheule.addOneGait(Stance)
# gaitScheule.addOneGait(Bound)
# gaitScheule.addOneGait(Bound)
# gaitScheule.addOneGait(Bound)
# gaitScheule.addOneGait(Bound)
# gaitScheule.addOneGait(Bound)
# gaitScheule.addOneGait(Bound)
# gaitScheule.addOneGait(Bound)
# gaitScheule.addOneGait(Jump_backwards)
# gaitScheule.addOneGait(Jump_forwards)
gaitScheule.addOneGait(Jump_Rotate_positive_yawRate)
# gaitScheule.addOneGait(Sideways_Jump_negativeVy)
# gaitScheule.addOneGait(Sideways_Jump_positiveVy)
# gaitScheule.addOneGait(Stance)
# gaitScheule.addOneGait(Bound)
# gaitScheule.addOneGait(Bound)
# gaitScheule.addOneGait(Bound)
# gaitScheule.addOneGait(Bound)
# gaitScheule.addOneGait(Bound)
# gaitScheule.addOneGait(Trot)
# gaitScheule.addOneGait(Trot)
# gaitScheule.addOneGait(Trot)
# gaitScheule.addOneGait(Trot)
# gaitScheule.addOneGait(Trot)
# gaitScheule.addOneGait(Trot)
# gaitScheule.addOneGait(Trot)
# gaitScheule.addOneGait(Stance)
# gaitScheule.addOneGait(Bound)
# gaitScheule.addOneGait(Bound)
# gaitScheule.addOneGait(Bound)
# gaitScheule.addOneGait(Bound)
gaitScheule.addOneGait(endGait)
# gaitScheule.addOneGait(Trot)
# gaitScheule.addOneGait(Stance)


# Setup the planners
reference_planner = ReferenceManager()
reference_planner.setGaitSchedule(gaitScheule)
reference_planner.setInitialCoMPosition(xinit, yinit, zinit, yaw_init)
reference_planner.setCoMTargetAndTransitionTime(vx_des, vy_des, z_des, yaw_rate_des, transition_time)
reference_planner.setSwingHeight(swingHeight)
reference_planner.computeReferenceTrajectoryOnce2()

N = round(gaitScheule.getFinalTime()/dt) 

print(gaitScheule.switchingTimes_)

# Create a pybullet model for ik computation
urdf_filename =  "/home/robocup/Documents/CAFE-MPC/urdf/mini_cheetah_simple_correctedInertia.urdf"
robot = MiniCheetah(urdf_file=urdf_filename)

pos_tau, vel_tau = [], []
z_tau, pf_tau, vf_tau = [], [], []
pfoot_tau = []
contact_tau = []
eul_tau, eulrate_tau = [],[]
time = []
jnt_tau = []
jntvel_tau = []
for k in range(N):
    t = k * dt
    pos = reference_planner.getCoMPositionAtTime(t)
    vel = reference_planner.getCoMVelAtTime(t)
    z = np.array([0.0, 0.0, 0.0, 0.0])
    contact = reference_planner.getContactStatusAtTime(t)
    pf = []
    vf = []
    for l in range(4):
        if contact[l] == 0:
            pf.append(reference_planner.getSwingFootPositionAtTime(l, t))
            vf.append(reference_planner.getSwingFootVelocityAtTime(l, t))
        else:
            pf.append(reference_planner.getFootholdLocationAtTime(l, t))
            vf.append(np.array([0,0,0]))
        z[l] = pf[l][2]
    eul = reference_planner.getEulerAngleAtTime(t)
    jnt_pos = robot.ik(pos, eul, np.hstack(pf))
    eul_rate = reference_planner.getEulerRateAtTime(t)

    pos_tau.append(pos)
    vel_tau.append(vel)
    eul_tau.append(eul)
    eulrate_tau.append(eul_rate)
    jnt_tau.append(jnt_pos)    
    jntvel_tau.append(np.zeros(12))
    pf_tau.append(np.hstack(pf))
    vf_tau.append(np.hstack(vf))
    contact_tau.append(contact)    
    time.append(t)

utils.write_traj_to_file(time, pos_tau, eul_tau, vel_tau, eulrate_tau, 
                         pf_tau, vf_tau, jnt_tau, jntvel_tau, contact_tau)
utils.publish_trajectory_lcm(time, pos_tau, eul_tau, vel_tau, eulrate_tau, 
                             jnt_tau, jntvel_tau, contact_tau)

utils.plot_eul(time, eul_tau)
utils.plot_com_pos(time, pos_tau)
utils.plot_com_vel(time, vel_tau)
# utils.plot_swing_height(time, z_tau)
# utils.plot_foothold_locations(time, pfoot_tau)
utils.plot_foot_positions(time, pf_tau)
utils.plot_footPosition_and_CoM(pf_tau, pos_tau)
utils.animate_footPositions_and_CoM(0, pf_tau, pos_tau)
utils.plot_jnt_position(time, jnt_tau, 0)


