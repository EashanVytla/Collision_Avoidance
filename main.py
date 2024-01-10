from simulator import Simulator
import numpy as np
import matplotlib.pyplot as plt

def main_loop():
    if(sim.get_own_pos()[0] > 25):
        sim.set_own_velo(np.array([0, 1, 0]))
    else:
        sim.set_own_velo(np.array([1, 0, 0]))
    return

if __name__ == "__main__":
    sim = Simulator()
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    for t in range(int(sim.reaction_time * sim.simulated_fps) + 10):
        main_loop()
        sim.update_positions()
        sim.plot_3d_environment(ax)

    plt.show()