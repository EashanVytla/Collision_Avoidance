import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

class Simulator:
    def __init__(self):
        self.simulated_fps = 10.0 #Expected FPS of software on Jetson
        self.reaction_time = 2.0 #Amount of time given to react in seconds
        self.max_drone_speed = 20.0 #Max drone speed in m/s

        self.max_sim_speed = self.max_drone_speed/self.simulated_fps

        self.sphere_size = 1000

        # Initial positions
        self.sphere_position = np.array([[40, 20, 10.0]])  # Starting high up in the center
        self.ownship_position = np.array([[0.0, 20, 0]])  # Starting at the left end of the environment

        self.sphere_speed = (self.sphere_position[0][2] - self.ownship_position[0][2]) / (self.sphere_position[0][0] - self.ownship_position[0][0])

        # Initial velocity for ownship (moving forward)
        self.ownship_velocity = np.array([self.max_drone_speed/self.simulated_fps, 0, 0])  # Adjust the velocity vector as needed

        # Initialize ownship path array
        self.ownship_path = [self.ownship_position.copy()]

    def update_positions(self):
        # Update sphere position (fall down)
        self.sphere_position[0][2] -= self.sphere_speed * (self.max_drone_speed/self.simulated_fps)

        # Update ownship position (move with velocity)
        self.ownship_position += self.ownship_velocity  # Adjust the ownship velocity as needed

        # Record ownship position for the path
        self.ownship_path.append(self.ownship_position.copy())

    def get_own_pos(self):
        return self.ownship_position[0]
    
    def get_occ_pos(self):
        return self.sphere_position[0]

    def plot_3d_environment(self, ax):
        ax.cla()  # Clear previous plot

        # Plot the sphere with larger size (adjust the s parameter)
        ax.scatter(*self.sphere_position.T, c='b', marker='o', alpha=0.3, s=self.sphere_size)

        # Plot the ownship drone
        ax.scatter(*self.ownship_position.T, c='r', marker='o')

        # Plot the ownship path as trailing black dots
        ownship_path_array = np.array(self.ownship_path)
        ax.scatter(*ownship_path_array.T, c='k', marker='.', alpha=0.5)

        # Set plot limits
        ax.set_xlim([0, 50])
        ax.set_ylim([0, 50])
        ax.set_zlim([0, 10])

        # Set labels
        ax.set_xlabel('X-axis')
        ax.set_ylabel('Y-axis')
        ax.set_zlabel('Z-axis')

        plt.pause(1/self.simulated_fps)  # Pause to visualize each timestep

    def set_own_velo(self, vec):
        # Initial velocity for ownship (moving forward)
        self.ownship_velocity = vec * self.max_sim_speed