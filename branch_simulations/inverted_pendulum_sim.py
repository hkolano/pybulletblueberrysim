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

from CaneDynamics import CaneDynamics

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
            # plot.setYRange(0, 0.75, padding=0)
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

        if self.x[-1] >= sim_length:
            self.timer.stop()
            self.win.close()
            plot_probe_info(self.sim.df)
            self.sim.plot_all_angles()
        elif list(data['probe_norm'])[-1] > f_stop_thresh:
            self.timer.stop()



class MultiPendulumSim():
    def __init__(self, urdf_path):
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
        self.pend = p.loadURDF(urdf_path, [0, 0, 0], useFixedBase=True)
        self.probe = p.loadURDF("urdf/probes/forked_probe.urdf", [0.003, -0.5, 1.2], useFixedBase=True)
        self.n_joints = p.getNumJoints(self.pend)
        self.ctrl_jt_idxs = list(range(self.n_joints))[1:] # the list of joints idxs we can actually control (not fixed joints)
        # will have to change the above if adding any artificial fixed joints 
        
        # Move to inital position
        boring_start_pos = [0.0]*(self.n_joints-1)
        self.move_to_position(boring_start_pos)

        # Set up global joint stiffness and damping parameters
        # self.kpSlider = p.addUserDebugParameter("KpBranch", 0, 7.5, 1.5)
        # self.kdSlider = p.addUserDebugParameter("KdBranch", 0, .5, .15)
        self.dyn = CaneDynamics(len(self.ctrl_jt_idxs))
        # self.dyn.reset_values()
        self.dyn.set_to_uniform_values(30, 0.1)
        # self.dyn.set_linearly_decreasing(160, 20, 1, .1)
        # self.dyn.set_linearly_increasing(5, 20, .01, .1)
        self.probe_text = p.addUserDebugText("probePos=0",[-0.1,-0.5,1.25])

        p.enableJointForceTorqueSensor(bodyUniqueId=self.probe, jointIndex=2,enableSensor=True)
        self.step_ctr = 0
        self.setup_df()
        # self.run_simulation()

    def setup_df(self):
        self.col_names_ea_joint = ['pos', 'vel', 'Fx', 'Fz', 'My']
        self.cols = ['times', 'probe_pos', 'probe_norm']
        # to connect the two things together
        # p.createConstraint(parentBodyUniqueId=self.probe,
        #                   parentLinkIndex=2,
        #                   childBodyUniqueId=self.pend,
        #                   childLinkIndex=3, # will need to change this dynamically when have more links or when the probe moves
        #                   jointType=p.JOINT_POINT2POINT,
        #                   jointAxis=[0,0,0],
        #                   parentFramePosition=[0,0,.005],
        #                   childFramePosition=[0,0,.18])
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
        p.setJointMotorControl2(bodyUniqueId=self.probe,
                                jointIndex=1,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=0)
        for _ in range(1000):
            p.stepSimulation()
        
        # Turn the motor off for free motion 
        p.setJointMotorControlArray(bodyUniqueId=self.pend, 
                                    jointIndices=self.ctrl_jt_idxs, 
                                    targetVelocities=[0]*(self.n_joints-1),
                                    controlMode=p.VELOCITY_CONTROL,
                                    forces=[0]*(self.n_joints-1))
        p.setJointMotorControl2(bodyUniqueId=self.probe,
                                jointIndex=1,
                                controlMode=p.VELOCITY_CONTROL,
                                targetVelocity=0,
                                force=0)

    def run_simulation(self):
        while p.isConnected():
            self.do_simulation_step()
            time.sleep(dt)

    def do_simulation_step(self):
        # Kp = p.readUserDebugParameter(self.kpSlider)
        # Kd = p.readUserDebugParameter(self.kdSlider)

        p.setJointMotorControlArray(bodyUniqueId=self.pend,
                                jointIndices=self.ctrl_jt_idxs,
                                controlMode=p.PD_CONTROL,
                                targetVelocities=[0]*len(self.ctrl_jt_idxs),
                                targetPositions=[0]*len(self.ctrl_jt_idxs),
                                positionGains=self.dyn.Kps,
                                velocityGains=self.dyn.Kds)

        p.setJointMotorControl2(bodyUniqueId=self.probe,
                                    jointIndex=1,
                                    controlMode=p.VELOCITY_CONTROL,
                                    targetVelocity=0.001)

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
        self.step_ctr += 1
        return self.df

    def get_current_state(self):
        time=self.step_ctr*dt
        data_vec = [time]
        [pos, _, _,_] = p.getJointState(bodyUniqueId=self.probe, jointIndex=1)
        data_vec.append(pos)
        q = p.getContactPoints(bodyA=self.probe, 
                               linkIndexA=2)
        if len(q) == 0:
            probe_norm = 0
        else:
            qs = []
            for i in range(len(q)):
                qs.append(q[i][9])
            probe_norm = max(qs) # eighth value of the contact point info is normal force
        # [_, _, probeft,_] = p.getJointState(bodyUniqueId=self.probe, jointIndex=2)
        # data_vec.extend([probeft[0], probeft[2])
        data_vec.append(probe_norm)
        for i in self.ctrl_jt_idxs:
            [pos, vel, Rf, _] = p.getJointState(bodyUniqueId=self.pend, jointIndex=i)
            resp = [pos, vel, Rf[0], Rf[2], Rf[4]]
            data_vec.extend(resp)
        return data_vec

    def add_current_state_to_df(self):
        self.df.loc[len(self.df)] = self.get_current_state()

    def plot_all_angles(self):
        n_plots = len(self.ctrl_jt_idxs)
        fig,axs = plt.subplots(n_plots, 1, sharey=True)
        ax_ctr = n_plots-1 # start plotting at the bottom
        for ax in axs:
            ax.grid(True)
        for i in self.ctrl_jt_idxs:
            pose_name = "pos" + "_" + str(i)
            line = axs[ax_ctr].plot(self.df['times'], self.df[pose_name])
            axs[ax_ctr].set_ylabel('Position of Joint' + str(i) + '(rad)')
            ax_ctr = ax_ctr - 1
        axs[ax_ctr].set_xlabel('Time (s)')
        fig.suptitle('Pendulum Joint Positions')
        plt.show()

    def plot_all_react_forces(self):
        n_rows = len(self.ctrl_jt_idxs)
        fig, axs = plt.subplots(n_rows, 3)
        ax_ctr = n_rows-1
        force_names = ['Fx', 'Fz', 'My']
        for i in self.ctrl_jt_idxs: # iterate over rows/joints
            for j in range(3):  # iterate through reaction forces
                pass

                               
def plot_probe_info(df):
    fig, axs = plt.subplots(2, 1)
    for ax in axs:
        ax.grid(True)
    disp_line = axs[0].plot(df['times'], df['probe_pos'])
    axs[0].set_ylabel('Position (m)')
    force_line = axs[1].plot(df['times'], df['probe_norm'])
    axs[1].set_ylabel('Probe Force (N)')
    axs[1].set_xlabel('Time (s)')
    fig.suptitle('Probe Position and Force')
    plt.show()

if __name__ == '__main__':

    # Environmental parameters
    dt = 1/240. # pybullet simulation step
    f_stop_thresh = 9.8067 # N of force (1000gf)
    sim_length = 10. # time simulation will run for, in seconds

    app = QApplication(sys.argv)
    sim = MultiPendulumSim("urdf/typed_pends/triple_pendulum.urdf")
    plot_list = ['probe_norm', 'pos_1', 'pos_2']
    window = MainWindow(sim, plot_list)
    # # window.show()
    sys.exit(app.exec_())