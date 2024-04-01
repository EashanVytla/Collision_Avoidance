import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

class Simulator:
    def __init__(self):
        self.simulated_fps = 10.0 #Expected FPS of software on Jetson
        self.reaction_time = 2.0 #Amount of time given to react in seconds
        self.max_drone_speed = 20.0 #Max drone speed in m/s
        self.max_sim_speed = self.max_drone_speed/self.simulated_fps
        self.occ_size_plot = 100

        # Initial stuff
        self.occ_num = np.random.randint(3, 11)
        self.occ_rad = (np.random.rand(self.occ_num,) + 1) * 0.75

        # Initial positions
        self.occ_position = (np.random.rand(self.occ_num,3) - 0.5) * 2 * 100

        self.ownship_position = np.array([[100.0, 100.0, 100.0]])

        # Velocity for occ
        self.occ_velocity = (np.random.rand(self.occ_num,3) - 0.5) * 2 * 11

        # Initial velocity for ownship (moving forward)
        self.ownship_velocity = np.array([[-6.0, 0.0, 0.0]])  # Adjust the velocity vector as needed

        # Initialize ownship path array
        self.ownship_path = [self.ownship_position.copy()]

    def update_velocity(self):
        self.occ_velocity = -1 * ((self.occ_position - self.ownship_position)/np.array([np.linalg.norm(self.occ_position - self.ownship_position, axis=-1)]).T) * np.array([np.linalg.norm(self.occ_velocity, axis=-1)]).T

    def update_positions(self):
        # Update occ position
        self.occ_position += self.occ_velocity/self.simulated_fps

        # Update ownship position (move with velocity)
        self.ownship_position += self.ownship_velocity/self.simulated_fps  # Adjust the ownship velocity as needed

        # Record ownship position for the path velocity
        self.ownship_path.append(self.ownship_position.copy())

    def get_own_pos(self):
        return self.ownship_position.T
    
    def get_occ_pos(self):
        return self.occ_position.T
    
    def get_own_velo(self):
        return self.ownship_velocity.T
    
    def get_occ_velo(self):
        return self.occ_velocity.T
    
    def get_occ_rad(self):
        return self.occ_rad

    def plot_3d_environment(self, ax):
        ax.cla()  # Clear previous plot

        # Plot the occ with larger size (adjust the s parameter)
        for i in range(0,self.occ_num):
            ax.scatter(*self.occ_position[i,:].T, c='b', marker='o', alpha=0.3, s=self.occ_size_plot)

        # Plot the ownship drone
        ax.scatter(*self.ownship_position.T, c='r', marker='o')

        # Plot the ownship path as trailing black dots
        ownship_path_array = np.array(self.ownship_path)
        ax.scatter(*ownship_path_array.T, c='k', marker='.', alpha=0.5)

        # Set plot limits
        ax.set_xlim([-100, 100])
        ax.set_ylim([-100, 100])
        ax.set_zlim([-100, 100])

        # Set labels
        ax.set_xlabel('X-axis')
        ax.set_ylabel('Y-axis')
        ax.set_zlabel('Z-axis')

        plt.pause(1/self.simulated_fps)  # Pause to visualize each timestep

    def set_own_velo(self, vec):
        # Initial velocity for ownship (moving forward)
        self.ownship_velocity = vec.T