import pybullet as p
import time
import pybullet_data

dt = 1/240 # pybullet simulation step
q0 = 0.5   # starting position (radian)
physicsClient = p.connect(p.GUI) # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
boxId = p.loadURDF("urdf/single_pendulum.urdf", useFixedBase=True)

# get rid of all the default damping forces
p.changeDynamics(boxId, 1, linearDamping=0, angularDamping=0)

angularDampingSlider = p.addUserDebugParameter("angularDamping", 0, 1, 0)

# go to the starting position
p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetPosition=q0, controlMode=p.POSITION_CONTROL)
for _ in range(1000):
    p.stepSimulation()

# turn off the motor for the free motion
p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetVelocity=0, controlMode=p.VELOCITY_CONTROL, force=0)
p.setRealTimeSimulation(1)
while True:
    angularDamping = p.readUserDebugParameter(angularDampingSlider)
    p.changeDynamics(boxId, 1, angularDamping=angularDamping)
    p.stepSimulation()
    time.sleep(dt)
p.disconnect()