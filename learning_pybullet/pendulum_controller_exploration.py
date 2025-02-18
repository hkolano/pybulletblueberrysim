import pybullet as p
import time
import pybullet_data

# Environment Parameters
dt = .01 # pybullet simulation step
q0 = 0.5   # starting position (radian)
physicsClient = p.connect(p.GUI) # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
pd = p.loadPlugin("pdControlPlugin")

# Load pendulums
pend_pos = p.loadURDF("urdf/single_pendulum.urdf", [0,0,0], useFixedBase=True)
pend_pd = p.loadURDF("urdf/single_pendulum.urdf", [0, 1, 0], useFixedBase=True)

# Debug text and parameters
textColor = [1, 1, 1]
textOffset = [0.05, 0, 0.1]
p.addUserDebugText("Pos Ctrl", textOffset, 
                   textColor, 
                   parentObjectUniqueId=pend_pos,
                   parentLinkIndex=0)
p.addUserDebugText("PD Ctrl", textOffset, 
                   textColor, 
                   parentObjectUniqueId=pend_pd,
                   parentLinkIndex=0)


angularDampingSlider = p.addUserDebugParameter("angularDamping", 0, 1, 0)

# go to the starting position
p.setJointMotorControl2(bodyIndex=pend_pos, jointIndex=1, targetPosition=q0, controlMode=p.POSITION_CONTROL)
p.setJointMotorControl2(bodyIndex=pend_pd, jointIndex=1, targetPosition=q0, controlMode=p.POSITION_CONTROL)
for _ in range(1000):
    p.stepSimulation()

# turn off the motor for the free motion
p.setJointMotorControl2(bodyIndex=pend_pos, jointIndex=1, targetVelocity=0, controlMode=p.VELOCITY_CONTROL, force=0)
p.setJointMotorControl2(bodyIndex=pend_pd, jointIndex=1, targetVelocity=0, controlMode=p.VELOCITY_CONTROL, force=0)
p.setRealTimeSimulation(1)
while True:
    angularDamping = p.readUserDebugParameter(angularDampingSlider)
    p.changeDynamics(pend_pos, 1, angularDamping=angularDamping)
    p.changeDynamics(pend_pd, 1, angularDamping=angularDamping)
    p.stepSimulation()
    time.sleep(dt)
p.disconnect()