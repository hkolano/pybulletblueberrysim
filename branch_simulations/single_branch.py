import pybullet as p
import time
import pybullet_data

dt = 1/240 # pybullet simulation step
q0 = 0.5   # starting position (radian)
physicsClient = p.connect(p.GUI) # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)

planeId = p.loadURDF("plane.urdf")
boxId = p.loadURDF("urdf/single_inverted_pendulum.urdf", useFixedBase=True)

# get rid of all the default damping forces
p.changeDynamics(boxId, 1, linearDamping=0, angularDamping=0)

pgainSlider = p.addUserDebugParameter("ProportionalGain", 0, 1000, 100)
dgainSlider = p.addUserDebugParameter("DerivativeGain", 0, 5, 0)

# go to the starting position
print("Going to start position.")
p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetPosition=q0, controlMode=p.POSITION_CONTROL)
for _ in range(1000):
    p.stepSimulation()

# turn off the motor for the free motion
print("Starting control.")
p.setRealTimeSimulation(1)
# p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetPosition=0, targetVelocity=0, controlMode=p.POSITION_CONTROL)
while True:
    user_p_gain = p.readUserDebugParameter(pgainSlider)
    user_d_gain = p.readUserDebugParameter(dgainSlider)
    p.setJointMotorControl2(bodyIndex=boxId, 
                            jointIndex=1, 
                            targetPosition=0, 
                            targetVelocity=0,
                            positionGain = user_p_gain,
                            velocityGain = user_d_gain,
                            controlMode=p.POSITION_CONTROL)
    time.sleep(dt)
p.disconnect()