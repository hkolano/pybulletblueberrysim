import pybullet as p
import time
import pybullet_data

# Set up the environment
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath()) 
p.setGravity(0,0,-10)
planeID = p.loadURDF("plane.urdf")

# Load the robot
startPos = [0, 0, 1]
startOrientation = p.getQuaternionFromEuler([0,0,0])
boxID = p.loadURDF("r2d2.urdf",startPos, startOrientation, useFixedBase = False)


for i in range (10000):
    p.stepSimulation()
    time.sleep(1./240.)
    cubePos,cubeOrn = p.getBasePositionAndOrientation(boxID)
    print(f'Current Position:{cubePos}', f'Current Orientation: {cubeOrn}')
p.disconnect()
