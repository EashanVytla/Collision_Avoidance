from simulator import Simulator
import numpy as np
import matplotlib.pyplot as plt
import math

rpz = 0.0

def init():
    rpz = sim.sphere_size

    return

def get_az(v): #TEST/FIX
    v[0, 2] = 0
    az = np.arctan(v[0,1]/v[0,0])

    return az

def get_el(v): #TEST/FIX
    h = v
    h[0, 2] = 0
    h = np.linalg.norm(h)
    el = np.arctan(v[0,2]/h)

    return el

def main_loop():
    doi = sim.get_occ_pos()[0,:] - sim.get_own_pos()[0,:]
    doi_norm = np.linalg.norm(doi)
    if (doi_norm == 0):
        doi_norm = 0.1

    rvo = rpz*((math.sqrt(pow(doi_norm, 2) - pow(rpz, 2)))/doi_norm)
    dvo = (pow(doi_norm, 2) - pow(rpz, 2))/doi_norm
    #theta_az = get_az(sim.get_occ_pos()[0,:])
    #theta_el = get_el(sim.get_occ_pos()[0,:])

    testAZ = get_az(np.array([[-1,0,0]]))
    testEL = get_el(np.array([[-1,0,0]]))
    testAZ = np.rad2deg(testAZ)
    testEL = np.rad2deg(testEL)
    print(testAZ)
    #print(testEL)
    
    # if(sim.get_own_pos()[0, 0] > 25):
    #     sim.set_own_velo(np.array([0, 1, 0]))
    # else:
    #     sim.set_own_velo(np.array([1, 0, 0]))
    # return

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