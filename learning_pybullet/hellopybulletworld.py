import pybullet as p
import time
import pybullet_data
 
 # Environment Parameters
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
p.resetDebugVisualizerCamera(cameraDistance=1.46,
                        cameraYaw=-180,
                        cameraPitch=-94,
                        cameraTargetPosition=[0, -.5, 0])

# Load in pendulum
pend = p.loadURDF("urdf/triple_pendulum_attached_probe.urdf", [0, 0, 0], useFixedBase=True)
print(p.getLinkState(pend, 6))
p.createConstraint(pend, 6, -1, -1, p.JOINT_FIXED, [0, 0, 0], [-.141, -0.5, 1.205], [0,0,0])
probe = p.loadURDF("urdf/probes/probe.urdf", [0.003, 0, 1.2], useFixedBase=True)
     