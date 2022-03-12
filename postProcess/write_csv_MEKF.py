#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
import csv

from real_time_simulator.msg import FSM
from real_time_simulator.msg import State
from real_time_simulator.msg import Control
from real_time_simulator.msg import Sensor
from real_time_simulator.msg import Trajectory
from real_time_simulator.msg import Waypoint
from real_time_simulator.msg import StateCovariance


import numpy as np
import math
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from scipy import interpolate

import time

import rosbag

tStart = 0
tEnd = 22

# Simulated state
position = np.zeros((1,3))
speed = np.zeros((1,3))
attitude = np.zeros((1,4))
omega = np.zeros((1,3))
prop_mass = np.zeros((1,1))
time_state = np.zeros((1,1))

# Estimated state from Navigation
position_est = np.zeros((1,3))
speed_est = np.zeros((1,3))
attitude_est = np.zeros((1,4))
omega_est = np.zeros((1,3))
prop_mass_est = np.zeros((1,1))
time_state_est = np.zeros((1,1))

# state covariance matrix from navigation
covariance = np.zeros((1,22))

# Controled forces and torque
control_force = np.zeros((1,3))
z_torque = np.zeros((1,1))
time_force = np.zeros((1,1))

# Measured force from AV
measured_force = np.zeros((1,3))
time_actuation = []

# Guidance optimal trajectory
target_positionZ = []
target_speedZ = []
target_prop_mass = []
time_target = []
thrust_target = []

rospack = rospkg.RosPack()
bag = rosbag.Bag(rospack.get_path('real_time_simulator') + '/log/log.bag')

for topic, msg, t in bag.read_messages(topics=['/fsm_pub']):
    if msg.state_machine != "Idle":
        time_init = t.to_sec()
        break


for topic, msg, t in bag.read_messages(topics=['/rocket_state']):
    new_pos = msg.pose.position
    new_speed = msg.twist.linear
    new_attitude = msg.pose.orientation
    new_omega = msg.twist.angular
    new_mass = msg.propeller_mass

    position = np.append(position, [[new_pos.x, new_pos.y, new_pos.z]], axis = 0)
    speed = np.append(speed, [[new_speed.x, new_speed.y, new_speed.z]], axis = 0)
    attitude = np.append(attitude, [[ new_attitude.x, new_attitude.y, new_attitude.z, new_attitude.w]], axis = 0)
    omega = np.append(omega, [[new_omega.x, new_omega.y, new_omega.z]], axis = 0)
    prop_mass = np.append(prop_mass, [[new_mass]])
    time_state = np.append(time_state, [[t.to_sec()]])

for topic, msg, t in bag.read_messages(topics=['/state_covariance']):
    new_cov = np.asarray(msg.covariance)
    new_cov.flatten()
    covariance = np.append(covariance,[new_cov],axis = 0)
    #print(covariance)

for topic, msg, t in bag.read_messages(topics=['/kalman_rocket_state']):
    new_pos = msg.pose.position
    new_speed = msg.twist.linear
    new_attitude = msg.pose.orientation
    new_omega = msg.twist.angular
    new_mass = msg.propeller_mass

    position_est = np.append(position_est, [[new_pos.x, new_pos.y, new_pos.z]], axis = 0)
    speed_est = np.append(speed_est, [[new_speed.x, new_speed.y, new_speed.z]], axis = 0)
    attitude_est = np.append(attitude_est, [[ new_attitude.x, new_attitude.y, new_attitude.z, new_attitude.w]], axis = 0)
    omega_est = np.append(omega_est, [[new_omega.x, new_omega.y, new_omega.z]], axis = 0)
    prop_mass_est = np.append(prop_mass_est, [[new_mass]])
    time_state_est = np.append(time_state_est, [[t.to_sec()]])

for topic, msg, t in bag.read_messages(topics=['/control_pub']):
    new_force = msg.force
    control_force = np.append(control_force, [[new_force.x, new_force.y, new_force.z]], axis = 0)
    z_torque = np.append(z_torque, [[msg.torque.z]])
    time_force = np.append(time_force, [[t.to_sec()]])

for topic, msg, t in bag.read_messages(topics=['/control_measured']):
    new_force = msg.force
    measured_force = np.append(measured_force, [[new_force.x, new_force.y, new_force.z]], axis = 0)
    time_actuation = np.append(time_actuation, [[t.to_sec()]])




bag.close()




# Only keep ROS data
prop_mass = prop_mass[1:]
speed = speed[1:]
omega = omega[1:]
position = position[1:]
attitude = attitude[1:]
time_state = time_state[1:]
covariance = covariance[1:]

prop_mass_est = prop_mass_est[1:]
speed_est = speed_est[1:]
omega_est = omega_est[1:]
position_est = position_est[1:]
attitude_est = attitude_est[1:]
time_state_est = time_state_est[1:]

control_force = control_force[1:]
z_torque = z_torque[1:]
time_force = time_force[1:]

measured_force = measured_force[1:]


# Synchronize time
time_force = time_force - time_init
time_state = time_state - time_init
time_state_est = time_state_est - time_init
time_actuation = time_actuation - time_init

# export state to csv
with open('state_data.csv','w',newline='') as csvfile:
    fieldnames = ['time_state','pos_x','pos_y','pos_z','vel_x','vel_y','vel_z','q0','q1','q2','q3','omega_x','omega_y','omega_z','mass']
    thewriter = csv.DictWriter(csvfile,fieldnames=fieldnames)
    #thewriter.writeheader()

    state_index = 0
    for time_state_index in time_state:
        thewriter.writerow({'time_state':time_state[state_index],'pos_x':position[state_index,0],'pos_y':position[state_index,1],'pos_z':position[state_index,2],'vel_x':speed[state_index,0],'vel_y':speed[state_index,1],'vel_z':speed[state_index,2],'q0':attitude[state_index,0],'q1':attitude[state_index,1],'q2':attitude[state_index,2],'q3':attitude[state_index,3],'omega_x':omega[state_index,0],'omega_y':omega[state_index,1],'omega_z':omega[state_index,2],'mass':prop_mass[state_index]})
        state_index +=1

# export kalman state to csv
with open('kalman_state_data.csv','w',newline='') as csvfile:
    fieldnames = ['time_state','pos_x','pos_y','pos_z','vel_x','vel_y','vel_z','q0','q1','q2','q3','omega_x','omega_y','omega_z','mass']
    thewriter = csv.DictWriter(csvfile,fieldnames=fieldnames)
    #thewriter.writeheader()

    state_index = 0
    for time_state_index in time_state_est:
        thewriter.writerow({'time_state':time_state_est[state_index],'pos_x':position_est[state_index,0],'pos_y':position_est[state_index,1],'pos_z':position_est[state_index,2],'vel_x':speed_est[state_index,0],'vel_y':speed_est[state_index,1],'vel_z':speed_est[state_index,2],'q0':attitude_est[state_index,0],'q1':attitude_est[state_index,1],'q2':attitude_est[state_index,2],'q3':attitude_est[state_index,3],'omega_x':omega_est[state_index,0],'omega_y':omega_est[state_index,1],'omega_z':omega_est[state_index,2],'mass':prop_mass_est[state_index]})
        state_index +=1

        # export kalman covarience to csv
with open('kalman_covariance_diagonal.csv','w',newline='') as csvfile:
    fieldnames = ['time_state','pos_x','pos_y','pos_z','vel_x','vel_y','vel_z','alpha_x','alpha_y','alpha_z','omega_x','omega_y','omega_z','mass']
    thewriter = csv.DictWriter(csvfile,fieldnames=fieldnames)
    #thewriter.writeheader()

    state_index = 0
    for time_state_index in time_state_est:
        thewriter.writerow({'time_state':time_state_est[state_index],'pos_x':covariance[state_index,0],'pos_y':covariance[state_index,1],'pos_z':covariance[state_index,2],'vel_x':covariance[state_index,3],'vel_y':covariance[state_index,4],'vel_z':covariance[state_index,5],'alpha_x':covariance[state_index,6],'alpha_y':covariance[state_index,7],'alpha_z':covariance[state_index,8],'omega_x':covariance[state_index,9],'omega_y':covariance[state_index,10],'omega_z':covariance[state_index,11],'mass':covariance[state_index,12]})
        state_index +=1
