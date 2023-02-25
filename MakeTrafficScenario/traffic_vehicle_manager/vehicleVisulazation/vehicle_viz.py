import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.patches import Rectangle
from matplotlib.animation import FuncAnimation
import numpy as np

class VehicleAnimation:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        plt.ion()  # Turn on interactive mode to update the plot in real-time
        self.fig, self.ax = plt.subplots()  # Create a new figure and axis object for plotting
        self.box = Rectangle((self.x - 1.4, self.y - 2), 2.8, 4, color='gray')  # Create a red box object for the plot
        self.ax.add_patch(self.box)  # Add the box object to the plot
        self.ax.set_xlim(-10, 10)  # Set the x-axis limits
        self.ax.set_ylim(-10, 10)  # Set the y-axis limits

        # Initialize animation
        self.anim = FuncAnimation(self.fig, self.update, interval=50)


    def plotting(self, dt, x, y):
        self.x = x
        self.y = y
        print(f'x: {self.x:.4f}, y: {self.y:.4f}')
        self.box.set_xy((self.x - 1.4, self.y - 2))  # Update the position of the box object
        self.ax.set_xlim(self.x - 5, self.x + 5)  # Adjust the x-axis limits based on the new x value
        self.ax.set_ylim(self.y - 5, self.y + 5)  # Adjust the y-axis limits based on the new y value
        plt.draw()  # Redraw the plot
        plt.pause(dt)  # Pause for a short time to allow the plot to be displayed
