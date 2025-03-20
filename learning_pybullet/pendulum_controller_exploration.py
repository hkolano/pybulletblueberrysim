import pybullet as p
import time
import pybullet_data
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def run_single_pend_simulation():
    # Environment Parameters
    dt = .01 # pybullet simulation step
    q0 = 0.5   # starting position (radian)
    physicsClient = p.connect(p.GUI) # or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0,0,-10)
    planeId = p.loadURDF("plane.urdf")
    pd_ctrl = p.loadPlugin("pdControlPlugin")

    # Set up data collection
    data = []

    # Load pendulums
    pend_pd = p.loadURDF("urdf/single_pendulum.urdf", [0, 0, 0], useFixedBase=True)

    # Debug text and parameters
    textColor = [1, 1, 1]
    textOffset = [0.05, 0, 0.1]
    p.addUserDebugText("PD Ctrl", textOffset, 
                       textColor, 
                       parentObjectUniqueId=pend_pd,
                       parentLinkIndex=0)

    kpSlider = p.addUserDebugParameter("KpBranch", 0, 5, .5)
    kdSlider = p.addUserDebugParameter("KdBranch", 0, .25, .05)

    # go to the starting position
    p.setJointMotorControl2(bodyIndex=pend_pd, jointIndex=1, targetPosition=q0, controlMode=p.POSITION_CONTROL)
    for _ in range(1000):
        p.stepSimulation()

    # turn off the motor for the free motion
    p.setJointMotorControl2(bodyIndex=pend_pd, jointIndex=1, targetVelocity=0, controlMode=p.VELOCITY_CONTROL, force=0)
    p.enableJointForceTorqueSensor(bodyUniqueId=pend_pd, jointIndex=1,enableSensor=True)
    # p.setRealTimeSimulation(1)

    while p.isConnected():
        Kp = p.readUserDebugParameter(kpSlider)
        Kd = p.readUserDebugParameter(kdSlider)

        p.setJointMotorControl2(bodyUniqueId=pend_pd,
                                jointIndex=1,
                                controlMode=p.PD_CONTROL,
                                targetVelocity=0,
                                targetPosition=0,
                                positionGain=Kp,
                                velocityGain=Kd)
        p.stepSimulation()
        [cur_pos, cur_vel, reactf,appliedf] = p.getJointState(bodyUniqueId=pend_pd, jointIndex=1)
        data.append({'poses':cur_pos, 'vels':cur_vel, 'RFx':reactf[0], 
                'RFy':reactf[1],
                'RFz':reactf[2],
                'RMx':reactf[3],
                'RMy':reactf[4],
                'RMz':reactf[5],
                'AppF':appliedf})
        time.sleep(dt)

    df = pd.DataFrame(data)
    df['times'] = np.linspace(dt, (len(df))*dt, len(df))
    print(df)

    plot_first_joint_pos_and_vel(df)
    plot_reaction_forces(df)
                                 

def plot_reaction_forces(df):
    """ Plots a 3 x 2 grid of the reaction forces for this run """
    fig, axs = plt.subplots(3,2)
    
    axs[0,0].set_ylabel('Force X (N)')
    Fx_line = axs[0,0].plot(df['times'], df['RFx'])

    axs[1,0].set_ylabel('Force Y (N)')
    Fy_line = axs[1,0].plot(df['times'], df['RFy'])

    axs[2,0].set_ylabel('Force Z (N)')
    Fz_line = axs[2,0].plot(df['times'], df['RFz'])

    axs[0,1].set_ylabel('Moment X (Nm)')
    Mx_line = axs[0,1].plot(df['times'], df['RMx'])

    axs[1,1].set_ylabel('Moment Y (N)')
    
    My_line = axs[1,1].plot(df['times'], df['RMy'])

    axs[2,1].set_ylabel('Moment Z (N)')
    Mz_line = axs[2,1].plot(df['times'], df['RMz'])
    
    plt.show()

def plot_first_joint_pos_and_vel(df):
    """ plots position and velocity of the first joint for this run """
    fig, axs = plt.subplots(2,1)
    for ax in axs:
        ax.grid(True)
    pose_line = axs[0].plot(df['times'], df['poses'])
    axs[0].set_ylabel('Position (rad)')
    vel_line = axs[1].plot(df['times'], df['vels'])
    axs[1].set_ylabel('Velocity (rad/s)')
    axs[1].set_xlabel('Time (s)')
    fig.suptitle('Pendulum Position and Velocity')
    plt.show()

if __name__ == '__main__':
    run_single_pend_simulation()