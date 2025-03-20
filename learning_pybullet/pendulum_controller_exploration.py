import pybullet as p
import time
import pybullet_data
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

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
# p.setRealTimeSimulation(1)
[init_pos, init_vel, _,_] = p.getJointState(bodyUniqueId=pend_pd, jointIndex=1)
data.append({'poses':init_pos, 'vels':init_vel})

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
    [cur_pos, cur_vel, _,_] = p.getJointState(bodyUniqueId=pend_pd, jointIndex=1)
    data.append({'poses':cur_pos, 'vels':cur_vel})
    time.sleep(dt)

df = pd.DataFrame(data)
df['times'] = np.linspace(0, (len(df)-1)*dt, len(df))
print(df)

usr_input = input("Plot position and velocity? y/n:")
if usr_input == "y" or "Y":
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