from time import sleep
import matplotlib.pyplot as plt

class visualizationManager:

    x_pose_plot = None
    y_pose_plot = None
    z_pose_plot = None

    sim_time_data = []
    sim_x_pose_data = []
    sim_y_pose_data = []
    sim_z_pose_data = []

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
        self.renderer = self.figure.canvas.renderer
        self.figure.draw(self.renderer)

        plt.draw()
        plt.pause(1e-17)

    def update_plots(self):
        #add data to plots
        self.x_pose_plot.plot(self.sim_time_data, self.sim_x_pose_data)
        self.y_pose_plot.plot(self.sim_time_data, self.sim_y_pose_data)
        self.z_pose_plot.plot(self.sim_time_data, self.sim_z_pose_data)

        #resize
        if(self.initialDraw):
            self.x_pose_plot.relim()
            self.x_pose_plot.autoscale_view()
            self.y_pose_plot.relim()
            self.y_pose_plot.autoscale_view()
            self.z_pose_plot.relim()
            self.z_pose_plot.autoscale_view()

        self.initialDraw = False

        #redraw
        self.figure.canvas.draw()
        self.figure.canvas.flush_events()
        plt.draw()
        plt.pause(1e-17)

    def append_pose_data(self, time, x, y, z):
        #add new data
        self.sim_time_data.append(time)
        self.sim_x_pose_data.append(x)
        self.sim_y_pose_data.append(y)
        self.sim_z_pose_data.append(z)

        self.update_plots()

