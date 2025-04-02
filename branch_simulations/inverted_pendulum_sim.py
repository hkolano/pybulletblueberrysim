from re import S
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
    def __init__(self, sim, plotting_list, *args, **kwargs):
        '''initializes a plot at all zeros'''
        super(MainWindow, self).__init__(*args, **kwargs)
        self.sim = sim
        self.win = pg.GraphicsLayoutWidget(show=True, title="Simulation Readouts")
        self.plot_list_strings = plotting_list
        self.num_plots =len(plotting_list)

        # Universal setup for all plots
        pg.setConfigOptions(antialias=True)
        pen = pg.mkPen(color=(0, 0, 255))
        self.win.setBackground('w')
        self.win.resize(1000,600)

        self.y_vals = {}
        self.lines = {}
        self.plots = {}
      
        x_timespan_secs = 2
        self.x = np.linspace(-1*x_timespan_secs+dt, 0, int(x_timespan_secs/dt))
        
        # Set up one plot for each of the values listed in the initialization
        for var_name in self.plot_list_strings:
            y = np.zeros(len(self.x))
            plot = self.win.addPlot(title=var_name)
            line = plot.plot(x=self.x, y=y, pen=pen)
            plot.setLabel('bottom', "Time (s)", unit='s')
            if var_name == "probe_pos":
                plot.setLabel('left', "Probe Position (m)", unit='m')
            elif "pos" in var_name:
                plot.setLabel('left', "Joint Position (rad)", unit='rad')
            elif "vel" in var_name:
                plot.setLabel('left', "Joint Velocity (rad/s)", unit='rad/s')
            elif "M" in var_name:
                plot.setLabel('left', "Reaction Moment (Nm)", unit='Nm')
            else:
                plot.setLabel('left', "Force (N)", unit='N')
            self.y_vals[var_name] = y 
            self.lines[var_name] = line
            self.plots[var_name] = plot
        
        # Perform the loop and call the simulation         
        self.timer = QTimer()
        self.timer.setInterval(int(1/dt))
        self.timer.timeout.connect(self.update_plot_data)
        self.timer.start()
        
    def update_plot_data(self):
        # Update the time 
        self.x = self.x[1:]  
        self.x = np.append(self.x, self.x[-1] + dt)
        
        # Run one simulation step
        data = self.sim.do_simulation_step()

        # go through each of the plots and update it with the new value
        for var_name in self.plot_list_strings:
            self.y_vals[var_name] = self.y_vals[var_name][1:]
            new_pt = list(data[var_name])[-1]
            self.y_vals[var_name] = np.append(self.y_vals[var_name], new_pt)
            self.lines[var_name].setData(self.x, self.y_vals[var_name])


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
        self.probe_text = p.addUserDebugText("probePos=0",[-0.1,-0.5,1.25])

        p.enableJointForceTorqueSensor(bodyUniqueId=self.probe, jointIndex=2,enableSensor=True)
        self.setup_df()
        # self.run_simulation()

    def setup_df(self):
        self.col_names_ea_joint = ['pos', 'vel', 'Fx', 'Fz', 'My']
        self.cols = ['probe_pos', 'probe_ft_x', 'probe_ft_z']
        for i in self.ctrl_jt_idxs:
            p.enableJointForceTorqueSensor(bodyUniqueId=self.pend, jointIndex=i,enableSensor=True)
            cols_to_add = [x + "_" + str(i) for x in self.col_names_ea_joint]
            self.cols.extend(cols_to_add)
        # first_vec = self.get_current_state()
        self.df = pd.DataFrame(columns=self.cols)
        self.add_current_state_to_df()

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

        self.add_current_state_to_df()
        [pos, _, _,_] = p.getJointState(bodyUniqueId=self.probe, jointIndex=1)

        txt = "probePos=" + str(pos)
        prevTextId = self.probe_text
        self.probe_text = p.addUserDebugText(txt, [-0.1,-0.5,1.25])
        p.removeUserDebugItem(prevTextId)
        # p.applyExternalForce(objectUniqueId=self.pend, 
        #                      linkIndex=2,
        #                      forceObj=[0.2,0,0],
        #                      posObj=[0,-0.5,1.2],
        #                      flags=p.WORLD_FRAME)
        p.stepSimulation()
        return self.df

    def get_current_state(self):
        data_vec = []
        [pos, _, _,_] = p.getJointState(bodyUniqueId=self.probe, jointIndex=1)
        data_vec.append(pos)
        [_, _, probeft,_] = p.getJointState(bodyUniqueId=self.probe, jointIndex=2)
        data_vec.extend([probeft[0], probeft[2]])
        for i in self.ctrl_jt_idxs:
            [pos, vel, Rf, _] = p.getJointState(bodyUniqueId=self.pend, jointIndex=i)
            resp = [pos, vel, Rf[0], Rf[2], Rf[4]]
            data_vec.extend(resp)
        return data_vec

    def add_current_state_to_df(self):
        self.df.loc[len(self.df)] = self.get_current_state()
                                

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
    plot_list = ['probe_ft_x', 'probe_ft_z', 'My_2']
    window = MainWindow(sim, plot_list)
    # # window.show()
  
    sys.exit(app.exec_())