from simulator import Simulator
import numpy as np
import matplotlib.pyplot as plt
import math

rpz = 10 #idk

def init():
    rpz = sim.sphere_size
    return

def main_loop():
    #Calc Geometry
    doi = (sim.get_occ_pos() - sim.get_own_pos()).T
    doi_norm = np.linalg.norm(doi) #problem if same pos - doi = 0
    rvo = rpz*((np.sqrt(pow(doi_norm, 2) - pow(rpz, 2)))/doi_norm)
    dvo = (pow(doi_norm, 2) - pow(rpz, 2))/doi_norm

    #Calc Transform Matrix  #Fails if in same pos - doi = 0 #Trig matrix didnt seem to work idk - see pdf coord sys
    e1 = doi / np.linalg.norm(doi)
    e2 = np.zeros((3,1))
    m = np.nonzero(e1)[0][0]
    if (m > 0):
        n = 0
    else:
        n = 1
    e2[n,0] = e1[m,0]
    e2[m,0] = -1*e1[n,0]
    e2 = e2 / np.linalg.norm(e2)
    e3 = np.cross(e1.T, e2.T).T
    transform = np.concatenate((e1,e2,e3),1)
    transform = np.linalg.inv(transform)

#For each obstacle, determine: t-imminence, d-imminence, divergence, collision
    #MODE DETERMINATION
    #if d non-imminent, t non-imminent: restore
    #if d imminent, t imminent: avoid
    #if d non-imminent, t imminent: avoid
    #if d imminent, t non-imminent: 1) divergent - restore 2) non-divergent: a) non-colliding: maintain b) colliding: avoid
    
    #Select Mode #FINISH THIS PART
    t_horizon = 10 #10seconds
    d_horizon = 20 #20meters
    velo_rel_transform = transform @ (sim.get_own_velo()-sim.get_occ_velo()).T
    if (velo_rel_transform[0,0] <= 0 or np.sqrt(pow(velo_rel_transform[1,0],2)+pow(velo_rel_transform[2,0],2))/velo_rel_transform[0,0] >= rvo/dvo):
        colliding = False
        t_imminent = False
    elif(np.linalg.norm(velo_rel_transform) > (doi_norm/t_horizon)): #technically should be shortest distance not doi
        colliding = True
        t_imminent = True
    else:
        colliding = True
        t_imminent = False
    #maybe put below in if(t_imminent==false): to skip unnecessary checks
    if (np.linalg.norm(velo_rel_transform) <= (doi_norm/t_horizon)): #technically should be shortest distance not doi
        d_imminent = False
    else:
        d_imminent = True
    #ADD DIVERGENCE CHECK
        #Current divergence check idea: transform absolute velocity to cc cone frame, take the coordinate point the velo vector lands at, place the obstacle velocity vector (infinite) backwards at this point and see if it intersects the cc cone


    #Calc RV
    phi_range = np.arange(0,22)*2*np.pi/22
    theta_range = np.arange(1,11)*np.pi/11
    FAunit = np.array([[0,0,1],[0,0,-1]]).T
    for phi in phi_range:
        for theta in theta_range:
            v = np.array([[np.sin(theta)*np.cos(phi),np.sin(theta)*np.sin(phi),np.cos(theta)]]).T
            FAunit = np.concatenate((FAunit, v),1)
    delta_t = 1 #not sure what timestep to use - also consider shifting own & occ pos by v*timestep for accuracy
    accel_max = 8 #8.5 in m/s^2 (rounded for int)
    FA = FAunit.copy()
    for k in range(2,(accel_max)+1):
        FA = np.concatenate((FA, FAunit*k),1)
    RV = FA*delta_t + sim.get_own_velo().T

    #Calc RAV
    RRV_transform = transform @ (RV-sim.get_occ_velo().T)
    RAV = np.array([[],[],[]])
    for j in range(0, np.shape(RRV_transform)[1]):
        if (RRV_transform[0,j] <= 0 or np.sqrt(pow(RRV_transform[1,j],2)+pow(RRV_transform[2,j],2))/RRV_transform[0,j] >= rvo/dvo):
            RAV = np.concatenate((RAV, np.array([RV[:,j]]).T),1)
            
    #Select Velocity     #random right now
    rand = np.random.randint(0, np.shape(RAV)[1])
    velo = np.array([RAV[:,rand]]) #horizontal

    sim.set_own_velo(velo/20) #/20 cuz idk how sim function works


    




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