from simulator import Simulator
import numpy as np
import matplotlib.pyplot as plt
import math

rpz = 18 #graph visualization varies (18)
t_horizon = 10 #10seconds
d_horizon = 20 #20meters

def init():

    return

def get_transform(d): #Fails if in same pos: doi (d) = 0
    e1 = d / np.linalg.norm(d)
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
    return transform

def get_mode(transform, doi_norm, rvo, dvo, v1, v2): #Probably can restructure #dont need some booleans
    #Restore --> mode 0; Maintain --> mode 1; Avoid --> mode 2

    #Check t_imminent and colliding
    velo_rel_transform = transform @ (v1-v2).T
    if (velo_rel_transform[0,0] <= 0 or np.sqrt(pow(velo_rel_transform[1,0],2)+pow(velo_rel_transform[2,0],2))/velo_rel_transform[0,0] >= rvo/dvo):
        colliding = False
        t_imminent = False
    elif(np.linalg.norm(velo_rel_transform) > ((doi_norm-rpz)/t_horizon)):
        colliding = True
        t_imminent = True
        mode = 2 #avoid
    else:
        colliding = True
        t_imminent = False

    #Check d_imminent and diverging
    if (t_imminent == False):
        if ((doi_norm-rpz) > d_horizon):
            d_imminent = False
            mode = 0 #restore
        else:
            d_imminent = True

            #Check diverging #might not be right, see notes.txt
            startpoint = transform @ v1.T
            dir_vector = -1*(transform @ v2.T)
            if(np.linalg.norm(dir_vector) != 0):
                dir_vector = dir_vector / np.linalg.norm(dir_vector)
            xv = startpoint[0,0]
            yv = startpoint[1,0]
            zv = startpoint[2,0]
            xn = dir_vector[0,0]
            yn = dir_vector[1,0]
            zn = dir_vector[2,0]
            denom = ((rvo/dvo)**2)*(xn**2)-(yn**2)-(zn**2)
            numer = ((rvo/dvo)**2) * ((((zv*xn)-(zn*xv))**2)+(((yv*xn)-(yn*xv))**2)) - ((zv*yn)-(zn*yv))**2
            if (colliding == True):
                diverging = False
            elif (denom != 0 and numer >= 0):
                d1 = ((zn*zv + yn*yv - ((rvo/dvo)**2)*xn*xv) / denom) + (np.sqrt(numer)/denom)
                d2 = ((zn*zv + yn*yv - ((rvo/dvo)**2)*xn*xv) / denom) - (np.sqrt(numer)/denom)
                if (d1 >= 0 and (xv+(d1*xn)) >= 0):
                    diverging = False
                elif (d2 >= 0 and (xv+(d2*xn)) >= 0):
                    diverging = False
                else:
                    diverging = True
                    mode = 0 #restore
            else:
                diverging = True
                mode = 0 #restore
            if (diverging == False and colliding == False):
                mode = 1 #maintain
            if (diverging == False and colliding == True):
                mode = 2 #avoid
    return mode


def main_loop(): #fix sim.get here
    own_pos = sim.get_own_pos()
    own_velo = sim.get_own_velo()
    occ_pos = sim.get_occ_pos()
    occ_velo = sim.get_occ_velo()

    #Calc Geometry
    doi = (occ_pos - own_pos).T
    doi_norm = np.linalg.norm(doi)
    rvo = rpz*((np.sqrt((doi_norm**2) - (rpz**2)))/doi_norm) #problem if same pos: doi_norm = 0
    dvo = ((doi_norm**2) - (rpz**2))/doi_norm

    #Calc Transform Matrix
    transform = get_transform(doi)

    #Select Mode
    mode = get_mode(transform, doi_norm, rvo, dvo, own_velo, occ_velo)
    #print("Candidate Mode:" + str(mode))
    
    #Restore Test
    if (mode == 0):
        #Calculate Restore Velocity
        test_vec = np.array([[90, 0, 0]]) - own_pos
        test_velo_restore = ((test_vec / np.linalg.norm(test_vec))) * 12 #mess around with this, idk what the control will look like
        #Forecast Positions
        test_own_pos = own_pos + (test_velo_restore/10) #10 is simulated_fps
        test_occ_pos = occ_pos + (occ_velo/10) #10 is simulated_fps

        #Calc Geometry
        test_doi = (test_occ_pos - test_own_pos).T
        test_doi_norm = np.linalg.norm(test_doi)
        if (test_doi_norm > rpz):
            test_rvo = rpz*((np.sqrt((test_doi_norm**2) - (rpz**2)))/test_doi_norm) #problem if same pos: doi_norm = 0
            test_dvo = ((test_doi_norm**2) - (rpz**2))/test_doi_norm

            #Calc Transform Matrix
            test_transform = get_transform(test_doi)

            #Get Mode
            test_mode = get_mode(test_transform, test_doi_norm, test_rvo, test_dvo, test_velo_restore, occ_velo)
            #("Test Mode:" + str(test_mode))
            if (test_mode == 2):
                mode = 1
        else:
            mode = 1
    
    #print("Selected Mode:" + str(mode))
    #AVOIDANCE MODE
    if (mode == 2):
        #Calc RV
        phi_range = np.arange(0,22)*2*np.pi/22
        theta_range = np.arange(1,11)*np.pi/11
        FAunit = np.array([[0,0,1],[0,0,-1]]).T
        for phi in phi_range:
            for theta in theta_range:
                v = np.array([[np.sin(theta)*np.cos(phi),np.sin(theta)*np.sin(phi),np.cos(theta)]]).T
                FAunit = np.concatenate((FAunit, v),1)
        delta_t = .1 #not sure what timestep to use - also consider forecasting own & occ pos by v*timestep for accuracy
        accel_max = 8 #8 in m/s^2
        FA = FAunit.copy()
        for k in range(2,(accel_max)+1):
            FA = np.concatenate((FA, FAunit*k),1)
        RV = FA*delta_t + own_velo.T

        #Calc RAV
        RRV_transform = transform @ (RV-occ_velo.T)
        RAV = np.array([[],[],[]])
        for j in range(0, np.shape(RRV_transform)[1]):
            if (RRV_transform[0,j] <= 0 or np.sqrt(pow(RRV_transform[1,j],2)+pow(RRV_transform[2,j],2))/RRV_transform[0,j] >= rvo/dvo):
                RAV = np.concatenate((RAV, np.array([RV[:,j]]).T),1)
                
        #Select Velocity
        if (np.size(RAV) != 0): #selects first V in RAV that is closest to current velocity
            RAV_diff = np.linalg.norm((RAV - own_velo.T), axis=0)
            ind = np.argmin(RAV_diff)
            velo_avoid = np.array([RAV[:, ind]]) #horizontal
        else: #selects first V in RV that is farthest from collision (doi vector)
            RRV_unit_transform = RRV_transform / np.linalg.norm(RRV_transform, axis=0)
            RRV_angle = np.arccos(np.clip(np.array([[1,0,0]]) @ RRV_unit_transform, -1.0, 1.0))
            ind = np.argmax(np.absolute(RRV_angle))
            velo_avoid = np.array([RV[:,ind]]) #horizontal

        sim.set_own_velo(velo_avoid.T)

    #MAINTAIN MODE
    elif (mode == 1):
        sim.set_own_velo(own_velo.T)
    
    #RESTORE MODE #would give control to path planner irl
    else:
        vec = np.array([[90, 0, 0]]) - own_pos
        velo_restore = ((vec / np.linalg.norm(vec))) * 12 #mess around with this, idk what the control will look like
        sim.set_own_velo(velo_restore.T)


if __name__ == "__main__":
    sim = Simulator()
    init()
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    init()

    for t in range(int(sim.reaction_time * sim.simulated_fps) + 200):
        main_loop()
        sim.update_positions()
        sim.plot_3d_environment(ax)

    plt.show()