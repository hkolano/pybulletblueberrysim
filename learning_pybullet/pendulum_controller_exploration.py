import pybullet as p
import time
import pybullet_data
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import sys
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import QTimer, QThread
import pyqtgraph as pg

class MainWindow(QMainWindow):
    def __init__(self, sim, *args, **kwargs):
        '''initializes a plot at all zeros'''
        super(MainWindow, self).__init__(*args, **kwargs)
        QThread.__init__(self)
        self.graphWidget = pg.PlotWidget()
        self.setCentralWidget(self.graphWidget)

        # self.update_func = update_func
        # print(self.update_func)
        self.sim = sim

        x_timespan_secs = 2
        self.x = np.linspace(-1*x_timespan_secs+dt, 0, int(x_timespan_secs/dt))
        self.y = np.zeros(len(self.x))
        
        self.graphWidget.setBackground('w')
        
        pen = pg.mkPen(color=(0, 0, 255))
        self.data_line = self.graphWidget.plot(self.x, self.y, pen=pen)
        self.timer = QTimer()
        self.timer.setInterval(int(1/dt))
        self.timer.timeout.connect(self.update_plot_data)
        self.timer.start()
        
    def update_plot_data(self):
        self.x = self.x[1:]  
        self.x = np.append(self.x, self.x[-1] + dt)
        
        self.y = self.y[1:]  
        data = self.sim.step_simulation()
        new_y_pt = data[-1]['poses']
        self.y = np.append(self.y, new_y_pt)
        
        self.data_line.setData(self.x, self.y)

class SinglePendulumSim():
    def __init__(self):
        # Environment Parameters
        q0 = 0.5   # starting position (radian)
        physicsClient = p.connect(p.GUI) # or p.DIRECT for non-graphical version
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0,0,-10)
        planeId = p.loadURDF("plane.urdf")

        # Set up data collection
        self.data = []
        self.data.append({'time':0, 'poses':q0, 'vels':0, 'RFx':0, 
                'RFy':0,
                'RFz':0,
                'RMx':0,
                'RMy':0,
                'RMz':0,
                'AppF':0})
        # self.plotter = plotter

        # Load pendulums
        self.pend = p.loadURDF("urdf/single_pendulum.urdf", [0, 0, 0], useFixedBase=True)

        # Debug text and parameters
        textColor = [1, 1, 1]
        textOffset = [0.05, 0, 0.1]
        p.addUserDebugText("PD Ctrl", textOffset, 
                           textColor, 
                           parentObjectUniqueId=self.pend,
                           parentLinkIndex=0)

        self.kpSlider = p.addUserDebugParameter("KpBranch", 0, 5, .5)
        self.kdSlider = p.addUserDebugParameter("KdBranch", 0, .25, .05)

        self.move_to_starting_position(q0)

    def move_to_starting_position(self, init_pos):
        # go to the starting position
        p.setJointMotorControl2(bodyIndex=self.pend, jointIndex=1, targetPosition=init_pos, controlMode=p.POSITION_CONTROL)
        for _ in range(1000):
            p.stepSimulation()

        # turn off the motor for the free motion
        p.setJointMotorControl2(bodyIndex=self.pend, jointIndex=1, targetVelocity=0, controlMode=p.VELOCITY_CONTROL, force=0)
        p.enableJointForceTorqueSensor(bodyUniqueId=self.pend, jointIndex=1,enableSensor=True)

    def step_simulation(self):
        # while p.isConnected():
        Kp = p.readUserDebugParameter(self.kpSlider)
        Kd = p.readUserDebugParameter(self.kdSlider)

        p.setJointMotorControl2(bodyUniqueId=self.pend,
                                jointIndex=1,
                                controlMode=p.PD_CONTROL,
                                targetVelocity=0,
                                targetPosition=0,
                                positionGain=Kp,
                                velocityGain=Kd)
        p.stepSimulation()
        [cur_pos, cur_vel, reactf,appliedf] = p.getJointState(bodyUniqueId=self.pend, jointIndex=1)
        time = self.data[-1]['time']+dt
        self.data.append({'time':time, 'poses':cur_pos, 'vels':cur_vel, 'RFx':reactf[0], 
                'RFy':reactf[1],
                'RFz':reactf[2],
                'RMx':reactf[3],
                'RMy':reactf[4],
                'RMz':reactf[5],
                'AppF':appliedf})

        # df = pd.DataFrame(self.data)
        # df['times'] = np.linspace(dt, (len(df))*dt, len(df))
        # print(df)
        return self.data
                             

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

    # Environmental parameters
    dt = .05 # pybullet simulation step

    app = QApplication(sys.argv)
    sim = SinglePendulumSim() # start simulation
    window = MainWindow(sim)
    window.show()
  
    sys.exit(app.exec_())