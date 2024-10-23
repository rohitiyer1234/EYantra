BODY_MASS = 0.05  # kg, mass of the bot's body
WHEEL_MASS = 0.018  # kg, mass of each wheel
MAX_TORQUE = 2.5  # Nm, max torque for the motors

angle_Kp=3.5
angle_Ki=1.9
angle_Kd=3
vel_Kp=-0.0027480
vel_Ki=0.000000
vel_Kd=0

dt=0.01

avg_vel= 0
right_vel= 0
left_vel = 0

# Global variables to hold state information
joint_handle_right = None
joint_handle_left = None
robot_handle = None
desired_angle = 0  # Target upright position in radians
desired_vel= 0
prev_ang_error=0
prev_vel_error=0
ang_err=0
vel_err=0
ang_integral=0
vel_integral=0
output=0


def sysCall_init():
    global joint_handle_right, joint_handle_left, robot_handle, graph1, errstream
    sim = require('sim')
    robot_handle=sim.getObjectHandle('body')
    # Get handles for the robot's joints and body using the object names you provided
    joint_handle_right = sim.getObjectHandle('right_joint')  # Right wheel's actuating joint
    joint_handle_left = sim.getObjectHandle('left_joint')  # Left wheel's actuating joint # Bot's body
    #graph1 = sim.getObject('/Graph')
    #errstream=sim.addGraphStream(graph1, 'Error','deg', 0, [1, 0, 0])

    
def sysCall_actuation():
    # put your actuation code here
    # This function will be executed at each simulation time step
    global ang_integral,prev_ang_error,desired_vel,vel_integral,prev_vel_error,output, desired_angle
    
    
    vel_integral += vel_err * dt
    vel_derivative = (vel_err - prev_vel_error) / dt
    desired_angle = (vel_Kp * vel_err) + (vel_Ki* vel_integral) + (vel_Kd * vel_derivative)
    prev_vel_error = vel_err
    #sim.setGraphStreamValue(graph1, errstream, vel_err)
    
    
    ang_integral += ang_err * dt
    derivative = (ang_err - prev_ang_error) / dt
    output += (angle_Kp * ang_err) + (angle_Ki * ang_integral) + (angle_Kd * derivative)
    prev_ang_error = ang_err
    


    sim.setJointTargetVelocity(joint_handle_right, output)
    sim.setJointTargetVelocity(joint_handle_left, output)
    
    pass

def sysCall_sensing():
    global ang_err,avg_vel,right_vel,left_vel,vel_err
    # put your sensing code here
    # This function will be executed at each simulation time step
    current_angle=sim.getObjectOrientation(robot_handle,-1)[0]
    right_vel=sim.getJointVelocity(joint_handle_right)
    left_vel=sim.getJointVelocity(joint_handle_left)
    avg_vel=(right_vel+left_vel)/2
    vel_err = desired_vel - avg_vel

    
    ####### ADD YOUR CODE HERE ######
    # Hint: Take feedback here & do the ang_error calculation
    ang_err=desired_angle - current_angle
    
    #################################
    pass

def sysCall_cleanup():
    
    pass

