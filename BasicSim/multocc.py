from simulator import Simulator
import numpy as np
import matplotlib.pyplot as plt
import math

own_rad = 0 #meters
d_lidar_correction = 1.05 #occ_rad scale to account for lidar error
d_safety = 5 #meters #minimmum distance allowed between closest points of own and occ (change this for more clearance)
d_horizon = 20 #meters #distance to consider collision
t_horizon = 10 #seconds #timeframe to consider collision
d_buffer = 10 #meters #buffers own_pos from rpz by accounting for real world error, disturbances, and nonlinearity

def init():

    return

def get_transform_all(D,num): #Fails if in same pos: doi (d) = 0
    transform_all = np.array([[[],[],[]],[[],[],[]],[[],[],[]]])
    for i in range(0,num):
        d = np.array([D[:,i]]).T
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
        transform = np.linalg.inv(np.concatenate((e1,e2,e3),-1)) #maybe just build matrix transposed since inv should = transpose
        transform = np.array([np.array([transform[:,0]]).T, np.array([transform[:,1]]).T, np.array([transform[:,2]]).T])
        transform_all = np.concatenate((transform_all, transform), -1)
    return transform_all #each obstacles matrix is [:,:,num-1]

def get_mode(transform_all, doi_norm_all, rpz_all, v1, v2_all, num): #Probably can restructure #dont need some booleans
    #Restore --> mode 0; Maintain --> mode 1; Avoid --> mode 2
    mode_all = np.array([])
    rbpz_all = rpz_all + d_buffer #this is so we go into avoid when heading into buffer zone
    if ((doi_norm_all-rbpz_all<0).any()):
        return 2.0
    rbvo_all = rbpz_all*np.sqrt((doi_norm_all**2)-(rbpz_all**2))/doi_norm_all #problem if same pos: doi_norm = 0
    dbvo_all = ((doi_norm_all**2)-(rbpz_all**2))/doi_norm_all
    for i in range(0,num):
        transform = transform_all[:,:,i].T
        doi_norm = doi_norm_all[i]
        rbvo = rbvo_all[i]
        dbvo = dbvo_all[i]
        rbpz = rbpz_all[i]
        v2 = np.array([v2_all[:,i]]).T
        #Check t_imminent and colliding
        velo_rel_trans = transform @ (v1-v2)
        if (velo_rel_trans[0,0] <= 0 or np.sqrt(pow(velo_rel_trans[1,0],2)+pow(velo_rel_trans[2,0],2))/velo_rel_trans[0,0] >= rbvo/dbvo):
            colliding = False
            t_imminent = False
        elif(np.linalg.norm(velo_rel_trans) > ((doi_norm-rbpz)/t_horizon)): #not the exact collision time because of sphere curvature
            colliding = True
            t_imminent = True
            mode = 2 #avoid
        else:
            colliding = True
            t_imminent = False

        #Check d_imminent and diverging
        if (t_imminent == False):
            if ((doi_norm-rbpz) > d_horizon):
                d_imminent = False
                mode = 0 #restore
            else:
                d_imminent = True

                #Check diverging #might not be right, see notes.txt
                startpoint = transform @ v1
                dir_vector = -1*(transform @ v2)
                if(np.linalg.norm(dir_vector) != 0):
                    dir_vector = dir_vector / np.linalg.norm(dir_vector)
                xv = startpoint[0,0]
                yv = startpoint[1,0]
                zv = startpoint[2,0]
                xn = dir_vector[0,0]
                yn = dir_vector[1,0]
                zn = dir_vector[2,0]
                denom = ((rbvo/dbvo)**2)*(xn**2)-(yn**2)-(zn**2)
                numer = ((rbvo/dbvo)**2) * ((((zv*xn)-(zn*xv))**2)+(((yv*xn)-(yn*xv))**2)) - ((zv*yn)-(zn*yv))**2
                if (colliding == True):
                    diverging = False
                elif (denom != 0 and numer >= 0):
                    d1 = ((zn*zv + yn*yv - ((rbvo/dbvo)**2)*xn*xv) / denom) + (np.sqrt(numer)/denom)
                    d2 = ((zn*zv + yn*yv - ((rbvo/dbvo)**2)*xn*xv) / denom) - (np.sqrt(numer)/denom)
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
        mode_all = np.append(mode_all, mode)
    mode = np.max(mode_all)
    return mode


def main_loop():
    own_pos = sim.get_own_pos()
    own_velo = sim.get_own_velo()
    occ_pos_all = sim.get_occ_pos()
    occ_velo_all = sim.get_occ_velo()
    rpz_all = sim.get_occ_rad()*d_lidar_correction + d_safety + own_rad
    occ_num = np.shape(occ_pos_all)[-1]

    #Calc Geometry
    doi_all = occ_pos_all - own_pos
    doi_norm_all = np.linalg.norm(doi_all, axis=-2)
    rvo_all = rpz_all*np.sqrt((doi_norm_all**2)-(rpz_all**2))/doi_norm_all #problem if same pos: doi_norm = 0
    dvo_all = ((doi_norm_all**2)-(rpz_all**2))/doi_norm_all

    #Calc Transform Matrix
    transform_all = get_transform_all(doi_all, occ_num) #each obstacles matrix is [:,:,num-1]

    #Select Mode
    mode = get_mode(transform_all, doi_norm_all, rpz_all, own_velo, occ_velo_all, occ_num)
    #print("Candidate Mode:" + str(mode))
    
    #Restore Test
    if (mode == 0):
        #Calculate Restore Velocity
        test_vec = np.array([[0, 0, 0]]).T - own_pos
        test_velo_restore = ((test_vec / np.linalg.norm(test_vec))) * 12 #mess around with this, idk what the control will look like
        #Forecast Positions
        test_own_pos = own_pos + (test_velo_restore/10) #10 is simulated_fps
        test_occ_pos_all = occ_pos_all + (occ_velo_all/10) #10 is simulated_fps

        #Calc Geometry & Get Mode
        test_doi_all = (test_occ_pos_all - test_own_pos)
        test_doi_norm_all = np.linalg.norm(test_doi_all, axis=-2)
        if (np.min(test_doi_norm_all-rpz_all) > 0):
            test_rvo_all = rpz_all*np.sqrt((test_doi_norm_all**2)-(rpz_all**2))/test_doi_norm_all #problem if same pos: doi_norm = 0
            test_dvo_all = ((test_doi_norm_all**2)-(rpz_all**2))/test_doi_norm_all

            #Calc Transform Matrix
            test_transform_all = get_transform_all(test_doi_all, occ_num)

            #Get Mode
            test_mode = get_mode(test_transform_all, test_doi_norm_all, rpz_all, test_velo_restore, occ_velo_all, occ_num)
            #print("Test Mode:" + str(test_mode))
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
        fa_unit = np.array([[0,0,1],[0,0,-1]]).T
        for phi in phi_range:
            for theta in theta_range:
                v = np.array([[np.sin(theta)*np.cos(phi),np.sin(theta)*np.sin(phi),np.cos(theta)]]).T
                fa_unit = np.concatenate((fa_unit, v),-1)
        t_delta = .1 #s #timestep
        accel_max = 8 #m/s^2
        fa = fa_unit.copy()
        for k in range(2,(accel_max)+1):
            fa = np.concatenate((fa, fa_unit*k),-1)
        rv = fa*t_delta + own_velo

        #Calc RAV
        rav = rv.copy()
        for i in range(0,occ_num):
            rav_rel_trans = transform_all[:,:,i].T @ (rav-np.array([occ_velo_all[:,i]]).T)
            for j in range(0, np.shape(rav_rel_trans)[1]):
                if ((rav_rel_trans[0,j] > 0 and np.sqrt(pow(rav_rel_trans[1,j],2)+pow(rav_rel_trans[2,j],2))/rav_rel_trans[0,j] < rvo_all[i]/dvo_all[i]) and (np.linalg.norm(rav_rel_trans[:,j]) > (doi_norm_all[i]-rpz_all[i])/t_horizon)): #not the exact collision time because of sphere curvature
                    rav[:,j] = np.array([[np.nan],[np.nan],[np.nan]]).T
            rav = rav[:, ~np.isnan(rav).any(axis=0)]
            if (np.size(rav) == 0):
                break

        #Select Velocity #DONT DO THIS BTW (just keeping for buffer math)
        if (np.size(rav) != 0): #Selects optimal RAV
            #Find minimum distance between velocity path and all obstacles+bufferzone
            dist_all = np.empty((0,np.shape(rav)[1]))
            for i in range(0,occ_num):
                occ_pos_rel = np.array([occ_pos_all[:,i]-own_pos[:,0]]).T #relative to own
                rav_rel = rav-np.array([occ_velo_all[:,i]]).T #relative to occ
                comp = (occ_pos_rel.T @ rav_rel) / np.linalg.norm((rav_rel), axis=-2) #component of occ_pos_rel in direction of rav_rel
                comp[comp < 0] = 0
                time = comp / np.linalg.norm((rav), axis=-2) #time do reach closest point
                time = time[0,:]
                proj = (rav / np.linalg.norm((rav), axis=-2)) * comp #projection of occ_pos_rel in direction of rav_rel
                proj[:, time > t_horizon] = rav[:, time > t_horizon] * t_horizon
                dist = np.array([(np.linalg.norm((proj - occ_pos_rel), axis=-2)) - rpz_all[i] - d_buffer]) #distance from closest point to buffer zone
                dist_all = np.concatenate((dist_all, dist), -2)
            dist_all = dist_all.min(axis=-2)
            #Select velocity corresponding to positive dist_all closest to own_velo (or just maximum negative dist_all when there's no positive values)
            if(np.size(dist_all[dist_all>0]) != 0):
                rav_posi = rav.copy()
                rav_posi[:,dist_all<=0] = np.inf
                ind = np.argmin(np.linalg.norm(rav_posi - own_velo, axis=-2))
            else:
                ind = np.argmax(dist_all) #not right because if own_pos is closest point, could select a slow velocity

            velo_avoid = np.array([rav[:,ind]]).T

        else: #Selects most avoiding RV
            ind_closest_occ = np.argmin(doi_norm_all)
            occ_pos_rel = np.array([occ_pos_all[:,ind_closest_occ]-own_pos[:,0]]).T #relative to own
            rv_rel = rv-np.array([occ_velo_all[:,ind_closest_occ]]).T #relative to occ
            rdot = -1*(occ_pos_rel.T @ rv_rel)/np.linalg.norm(occ_pos_rel) #postiive means increasing doi
            ind = np.argmax(rdot)
            velo_avoid = np.array([rv[:,ind]]).T

        sim.set_own_velo(velo_avoid)

    #MAINTAIN MODE
    elif (mode == 1):
        sim.set_own_velo(own_velo)
    
    #RESTORE MODE #would give control to path planner irl
    else:
        vec = np.array([[0, 0, 0]]).T - own_pos
        velo_restore = ((vec / np.linalg.norm(vec))) * 12 #mess around with this, idk what the control will look like
        sim.set_own_velo(velo_restore)

    #going slow: (1) check why we are avoiding so much - I think it is because the avoidance manuever selected is a slow down manuever, barely missing buffer within t_horizon
    #            (2) maybe select rav closest to restore manuever instead of own_velo so it doesn't keep falling


if __name__ == "__main__":
    sim = Simulator()
    init()
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    init()

    for t in range(int(sim.reaction_time * sim.simulated_fps) + 5*200):
        main_loop()
        sim.update_velocity()
        sim.update_positions()
        sim.plot_3d_environment(ax)

    plt.show()