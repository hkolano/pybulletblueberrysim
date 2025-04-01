from turtle import pendown
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
        self.sim = sim
        self.win = pg.GraphicsLayoutWidget(show=True, title="Simulation Readouts")

        # Universal setup for all plots
        pg.setConfigOptions(antialias=True)
        pen = pg.mkPen(color=(0, 0, 255))
        self.win.setBackground('w')
        self.win.resize(1000,600)
      
        x_timespan_secs = 2
        self.x = np.linspace(-1*x_timespan_secs+dt, 0, int(x_timespan_secs/dt))
        self.y = np.zeros(len(self.x))
        self.y2 = np.zeros(len(self.x))
        self.Fxs = np.zeros(len(self.x))
        self.Fys = np.zeros(len(self.x))
        self.Fzs = np.zeros(len(self.x))

        # Position plot
        self.p_pos_plot = self.win.addPlot(title="Probe Position")
        self.pos_data_line = self.p_pos_plot.plot(x=self.x, y=self.y, pen=pen)
        # self.p_pos_plot.setYRange(-0.5, 0.5, padding=0)
        self.p_pos_plot.setLabel('bottom', "Time (s)", unit='s')
        self.p_pos_plot.setLabel('left', "Position (rad)", unit='rad')

        # # Velocity plot
        # self.p_vel_plot = self.win.addPlot(title="Pend Velocity")
        # self.vel_data_line = self.p_vel_plot.plot(x=self.x, y=self.y2, pen=pen)
        # # self.p_vel_plot.setYRange(-1, 1, padding=0)
        # self.p_vel_plot.setLabel('bottom', "Time (s)", unit='s')
        # self.p_vel_plot.setLabel('left', "Velocity (rad/s)", unit='rad/s')

        self.win.nextRow()
        self.Fx_plot = self.win.addPlot(title="Reaction Force X")
        self.Fx_line = self.Fx_plot.plot(x=self.x, y=self.Fxs, pen=pen)
        # self.Fx_plot.setYRange(-1, 1, padding=0)
        self.Fx_plot.setLabel('bottom', "Time (s)", unit='s')
        self.Fx_plot.setLabel('left', "Fx (N)", unit='N')

        # self.Fy_plot = self.win.addPlot(title="Reaction Force Y")
        # self.Fy_line = self.Fy_plot.plot(x=self.x, y=self.Fys, pen=pen)
        # # self.Fy_plot.setYRange(-1, 1, padding=0)
        # self.Fy_plot.setLabel('bottom', "Time (s)", unit='s')
        # self.Fy_plot.setLabel('left', "Fy (N)", unit='N')

        self.Fz_plot = self.win.addPlot(title="Reaction Force Z")
        self.Fz_line = self.Fz_plot.plot(x=self.x, y=self.Fzs, pen=pen)
        # self.Fz_plot.setYRange(-.1, .1, padding=0)
        self.Fz_plot.setLabel('bottom', "Time (s)", unit='s')
        self.Fz_plot.setLabel('left', "Fz (N)", unit='N')

        
        # Perform the loop and call the simulation         
        self.timer = QTimer()
        self.timer.setInterval(int(1/dt))
        self.timer.timeout.connect(self.update_plot_data)
        self.timer.start()
        
    def update_plot_data(self):
        # Update the time 
        self.x = self.x[1:]  
        self.x = np.append(self.x, self.x[-1] + dt)
        
        self.y = self.y[1:]  
        data = self.sim.do_simulation_step()
        new_y_pt = data[-1]['probe_pos']
        self.y = np.append(self.y, new_y_pt)

        # self.y2 = self.y2[1:]
        # new_y2_pt = data[-1]['vels']
        # self.y2 = np.append(self.y2, new_y2_pt)

        self.Fxs = self.Fxs[1:]
        new_Fx = data[-1]['probe_x']
        self.Fxs = np.append(self.Fxs, new_Fx)

        # self.Fys = self.Fys[1:]
        # new_Fy = data[-1]['probe_y']
        # self.Fys = np.append(self.Fys, new_Fy)

        self.Fzs = self.Fzs[1:]
        new_Fz = data[-1]['probe_z']
        self.Fzs = np.append(self.Fzs, new_Fz)
        
        self.pos_data_line.setData(self.x, self.y)
        # self.vel_data_line.setData(self.x, self.y2)
        self.Fx_line.setData(self.x, self.Fxs)
        # self.Fy_line.setData(self.x, self.Fys)
        self.Fz_line.setData(self.x, self.Fzs)

class MultiPendulumSim():
    def __init__(self, urdf_path):
        # Environment Parameters
        physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0,0,-10)
        planeId = p.loadURDF("plane.urdf")
        p.resetDebugVisualizerCamera(cameraDistance=2,
                             cameraYaw=17.5,
                             cameraPitch=-35,
                             cameraTargetPosition=[0, -.5, 0])

        # Load in pendulum
        self.pend = p.loadURDF(urdf_path, [0, 0, 0], useFixedBase=True)
        self.probe = p.loadURDF("urdf/probe.urdf", [0, -0.5, 1.2], useFixedBase=True)
        self.n_joints = p.getNumJoints(self.pend)
        self.ctrl_jt_idxs = list(range(self.n_joints))[1:] # the list of joints idxs we can actually control (not fixed joints)
        # will have to change the above if adding any artificial fixed joints 
        
        # Move to inital position
        boring_start_pos = [0.0]*(self.n_joints-1)
        self.move_to_position(boring_start_pos)

        # Set up global joint stiffness and damping parameters
        self.kpSlider = p.addUserDebugParameter("KpBranch", 0, 7.5, 1.5)
        self.kdSlider = p.addUserDebugParameter("KdBranch", 0, .5, .15)

        p.enableJointForceTorqueSensor(bodyUniqueId=self.probe, jointIndex=2,enableSensor=True)
        self.setup_df()
        # self.run_simulation()

    def setup_df(self):
        self.col_names_ea_joint = ['pos', 'vel', 'Fx', 'Fz', 'My']
        self.cols = ['time', 'probe_x', 'probe_y', 'probe_z']
        self.data = []
        self.data.append({'probe_pos':0,'probe_x':0, 'probe_z':0})

    def move_to_position(self, pos_list):
        # Go to the given position
        p.setJointMotorControlArray(bodyUniqueId=self.pend, 
                                    jointIndices=self.ctrl_jt_idxs, 
                                    targetPositions=pos_list, 
                                    targetVelocities=[0]*(self.n_joints-1),
                                    controlMode=p.POSITION_CONTROL)
        for _ in range(1000):
            p.stepSimulation()
        
        # Turn the motor off for free motion 
        p.setJointMotorControlArray(bodyUniqueId=self.pend, 
                                    jointIndices=self.ctrl_jt_idxs, 
                                    targetVelocities=[0]*(self.n_joints-1),
                                    controlMode=p.VELOCITY_CONTROL,
                                    forces=[0]*(self.n_joints-1))

    def run_simulation(self):
        while p.isConnected():
            self.do_simulation_step()
            time.sleep(dt)

    def do_simulation_step(self):
        Kp = p.readUserDebugParameter(self.kpSlider)
        Kd = p.readUserDebugParameter(self.kdSlider)

        p.setJointMotorControlArray(bodyUniqueId=self.pend,
                                jointIndices=self.ctrl_jt_idxs,
                                controlMode=p.PD_CONTROL,
                                targetVelocities=[0]*len(self.ctrl_jt_idxs),
                                targetPositions=[0]*len(self.ctrl_jt_idxs),
                                positionGains=[Kp]*len(self.ctrl_jt_idxs),
                                velocityGains=[Kd]*len(self.ctrl_jt_idxs))

        p.setJointMotorControl2(bodyUniqueId=self.probe,
                                    jointIndex=1,
                                    controlMode=p.VELOCITY_CONTROL,
                                    targetVelocity=0.25)

        [pos, _, _,_] = p.getJointState(bodyUniqueId=self.probe, jointIndex=1)
        [_, _, reactf,_] = p.getJointState(bodyUniqueId=self.probe, jointIndex=2)
        self.data.append({'probe_pos':pos,'probe_x':reactf[0],'probe_z':reactf[2]})

        # p.applyExternalForce(objectUniqueId=self.pend, 
        #                      linkIndex=2,
        #                      forceObj=[0.2,0,0],
        #                      posObj=[0,-0.5,1.2],
        #                      flags=p.WORLD_FRAME)
        p.stepSimulation()
        return self.data

class SinglePendulumSim():
    def __init__(self):
        # Environment Parameters
        q0 = 0.5   # starting position (radian)
        physicsClient = p.connect(p.GUI) # or p.DIRECT for non-graphical version
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0,0,-10)
        planeId = p.loadURDF("plane.urdf")
        p.resetDebugVisualizerCamera(cameraDistance=2,
                             cameraYaw=17.5,
                             cameraPitch=-35,
                             cameraTargetPosition=[0, -.5, 0])

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
    sim = MultiPendulumSim("urdf/double_pendulum.urdf")
    window = MainWindow(sim)
    # # window.show()
  
    sys.exit(app.exec_())