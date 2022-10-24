from time import sleep
import matplotlib.pyplot as plt

class visualizationManager:

    x_pose_plot = None
    y_pose_plot = None
    z_pose_plot = None

    sim_time_data = []
    ekf_time_data = []

    sim_x_pose_data = []
    sim_y_pose_data = []
    sim_z_pose_data = []

    ekf_x_pose_data = []
    ekf_y_pose_data = []
    ekf_z_pose_data = []

    last_update_time = 0 # the time the view has been last updated
    update_interval = 1# number of seconds between view updates

    initialDraw = True # is this the first time the plots have been drawn

    def __init__(self):
        #init plots
        self.figure, plots = plt.subplots(3)
        self.x_pose_plot = plots[0]
        self.y_pose_plot = plots[1]
        self.z_pose_plot = plots[2]

        # self.x_pose_plot.plot([1,2,4], [2,3,4])
        # self.y_pose_plot.plot([4,5,6], [3,4,5])
        # self.z_pose_plot.plot([6,7,8], [4,65,6])

        #prep plot 
        self.figure.canvas.draw()
        self.figure.canvas.manager.set_window_title('X, Y, Z Positions') #set window title
        self.figure.tight_layout(pad=1.0) #set spacing
        self.figure.set_figheight(8)
        self.figure.set_figwidth(11)
        self.renderer = self.figure.canvas.renderer
        self.figure.draw(self.renderer)

        plt.draw()
        plt.pause(1e-17)

    def update_plots(self):
        if(len(self.sim_time_data) > 1):
            self.lastUpdateTime = self.sim_time_data[len(self.sim_time_data) - 1]

        #add data to plots
        self.x_pose_plot.plot(self.sim_time_data, self.sim_x_pose_data, "-k", label="Real X")
        self.x_pose_plot.plot(self.ekf_time_data, self.ekf_x_pose_data, "-r", label="EKF X")

        self.y_pose_plot.plot(self.sim_time_data, self.sim_y_pose_data, "-k", label="Real Y")
        self.y_pose_plot.plot(self.ekf_time_data, self.ekf_y_pose_data, "-r", label="EKF Y")

        self.z_pose_plot.plot(self.sim_time_data, self.sim_z_pose_data, "-k", label="Real Z")
        self.z_pose_plot.plot(self.ekf_time_data, self.ekf_z_pose_data, "-r", label ="EKF Z")




        #resize
        if(self.initialDraw):
            self.x_pose_plot.relim()
            self.x_pose_plot.autoscale_view()
            self.y_pose_plot.relim()
            self.y_pose_plot.autoscale_view()
            self.z_pose_plot.relim()
            self.z_pose_plot.autoscale_view()

        #add labels
        #x labels
        self.x_pose_plot.set_xlabel("Time (sec)")
        self.x_pose_plot.set_ylabel("X Position (m)")

        #y labels
        self.y_pose_plot.set_xlabel("Time (sec)")
        self.y_pose_plot.set_ylabel("Y Position (m)")

        #z labels
        self.z_pose_plot.set_xlabel("Time (sec)")
        self.z_pose_plot.set_ylabel("Z Position (m)")

        self.initialDraw = False

        #redraw
        self.figure.canvas.draw()
        self.figure.canvas.flush_events()
        plt.draw()
        plt.pause(1e-17)

    def append_sim_pose_data(self, time, x, y, z):
        #add new data
        self.sim_time_data.append(time)
        self.sim_x_pose_data.append(x)
        self.sim_y_pose_data.append(y)
        self.sim_z_pose_data.append(z)

        if time > (self.last_update_time + self.update_interval):
            self.update_plots()

    def append_ekf_pose_data(self, time, x, y, z):
        
        #add new data
        self.ekf_time_data.append(time)
        self.ekf_x_pose_data.append(x)
        self.ekf_y_pose_data.append(y)
        self.ekf_z_pose_data.append(z)

        self.update_plots()


