from simulator import Simulator
import numpy as np
import matplotlib.pyplot as plt
import math

rpz = 0.0

def init():
    rpz = sim.sphere_size

    return

def main_loop():
    doi = sim.get_occ_pos() - sim.get_own_pos()
    norm = np.linalg.norm(doi)
    print(norm)
    rvo = rpz * ((math.sqrt(pow(norm, 2) - pow(rpz, 2)))/norm)
    dvo = (pow(norm, 2) - pow(rpz, 2))/norm
    thetavo = math.atan(rvo/dvo)
    thetaaz = math.atan((sim.get_occ_pos()[1] - sim.get_own_pos()[1])/(sim.get_occ_pos()[0] - sim.get_own_pos()[0]))
    thetael = math.atan((sim.get_occ_pos()[2] - sim.get_own_pos()[2])/(math.sqrt(pow(sim.get_occ_pos()[1] - sim.get_own_pos()[1], 2) + pow(sim.get_occ_pos()[0] - sim.get_own_pos()[0], 2))))

    
    
    if(sim.get_own_pos()[0] > 25):
        sim.set_own_velo(np.array([0, 1, 0]))
    else:
        sim.set_own_velo(np.array([1, 0, 0]))
    return

if __name__ == "__main__":
    sim = Simulator()
    init()
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    init()

    for t in range(int(sim.reaction_time * sim.simulated_fps) + 10):
        main_loop()
        sim.update_positions()
        sim.plot_3d_environment(ax)

    plt.show()