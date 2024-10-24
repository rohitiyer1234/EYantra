import numpy as np

BODY_MASS = 0.05  
WHEEL_MASS = 0.018  
MAX_TORQUE = 2.5  

angle_Kp = 3.5
angle_Ki = 2.35
angle_Kd = 2.85
vel_Kp = -0.002855
vel_Ki = -0.0025
vel_Kd = 0
dt = 0.01

avg_vel = right_vel = left_vel = 0
left_add = right_add = 0
joint_handle_right = joint_handle_left = robot_handle = None
desired_angle = desired_vel = prev_ang_error = prev_vel_error = 0
ang_err = vel_err = ang_integral = vel_integral = output = 0

def sysCall_init():
    global joint_handle_right, joint_handle_left, robot_handle
    sim = require('sim')
    robot_handle = sim.getObjectHandle('body')
    joint_handle_right = sim.getObjectHandle('right_joint')
    joint_handle_left = sim.getObjectHandle('left_joint')

def sysCall_actuation():
    global ang_integral, prev_ang_error, desired_angle, vel_integral, prev_vel_error, output
    vel_integral += vel_err * dt
    vel_derivative = (vel_err - prev_vel_error) / dt
    desired_angle = vel_Kp * vel_err + vel_Ki * vel_integral + vel_Kd * vel_derivative
    prev_vel_error = vel_err

    ang_integral += ang_err * dt
    derivative = (ang_err - prev_ang_error) / dt
    output += angle_Kp * ang_err + angle_Ki * ang_integral + angle_Kd * derivative
    prev_ang_error = ang_err

    sim.setJointTargetVelocity(joint_handle_right, output + right_add)
    sim.setJointTargetVelocity(joint_handle_left, output + left_add)

def sysCall_sensing():
    global ang_err, avg_vel, right_vel, left_vel, vel_err, left_add, right_add, desired_vel  # Declare desired_vel as global
    orientation_quat = sim.getObjectQuaternion(robot_handle, -1)
    current_angle = np.arctan2(
        2 * (orientation_quat[3] * orientation_quat[0] + orientation_quat[1] * orientation_quat[2]),
        1 - 2 * (orientation_quat[0] ** 2 + orientation_quat[1] ** 2)  # Corrected squaring here
    )
    
    right_vel = sim.getJointVelocity(joint_handle_right)
    left_vel = sim.getJointVelocity(joint_handle_left)
    avg_vel = (right_vel + left_vel) / 2
    vel_err = desired_vel - avg_vel
    ang_err = desired_angle - current_angle

    # Handle keyboard input
    message, data, _ = sim.getSimulatorMessage()
    if message == sim.message_keypress:
        if data[0] == 2007:  # Up arrow
            if (desired_vel <0):
                    desired_vel=0
            desired_vel += 2
        elif data[0] == 2008:  # Down arrow
            if (desired_vel >0):
                desired_vel=0
            desired_vel -= 2
        elif data[0] == 2009: # Left arrow
            if (left_add<0 and right_add >0):
                left_add=0
                right_add=0
            left_add += 0.2  # Slow down left wheel to turn right
            right_add -= 0.2  # Speed up right wheel to turn right
        elif data[0] == 2010:  # Right arrow
            if (left_add>0 and right_add <0):
                left_add=0
                right_add=0
            left_add -= 0.2  # Speed up left wheel to turn left