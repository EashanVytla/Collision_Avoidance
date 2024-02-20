from simulator import Simulator
import numpy as np
import matplotlib.pyplot as plt
import math

own_rad = 0 #meters
d_lidar_correction = 1.05 #occ_rad scale to account for lidar error
d_safety = 5 #meters #minimmum distance allowed between closest points of own and occ (change this for more clearance)
d_buffer = 4 #meters #buffers own_pos from rpz by accounting for real world error, disturbances, and nonlinearity
d_horizon = 20 #meters #distance to consider collision
t_horizon = 10 #seconds #timeframe to consider collision

own_velo_max = 20 #m/s
accel_max = 8 #m/s^2
t_delta = .1 #s #timestep

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

def get_mode(transform_all, doi_norm_all, rpz_all, rvo_all, dvo_all, v1, v2_all, num): #Probably can restructure #dont need some booleans
    #Restore --> mode 0; Maintain --> mode 1; Avoid --> mode 2
    mode_all = np.array([])
    if ((doi_norm_all-rpz_all<0).any()):
        return 2.0
    for i in range(0,num):
        transform = transform_all[:,:,i].T
        doi_norm = doi_norm_all[i]
        rvo = rvo_all[i]
        dvo = dvo_all[i]
        rpz = rpz_all[i]
        v2 = np.array([v2_all[:,i]]).T
        #Check t_imminent and colliding
        velo_rel_trans = transform @ (v1-v2)
        if (velo_rel_trans[0,0] <= 0 or np.sqrt(pow(velo_rel_trans[1,0],2)+pow(velo_rel_trans[2,0],2))/velo_rel_trans[0,0] >= rvo/dvo):
            colliding = False
            t_imminent = False
        elif(np.linalg.norm(velo_rel_trans) > ((doi_norm-rpz)/t_horizon)): #not the exact collision time because of sphere curvature - address this
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
        mode_all = np.append(mode_all, mode)
    mode = np.max(mode_all)
    return mode

def get_velo_restore(own_pos, own_velo):
    goal = np.array([[0,0,0]]).T
    vec = goal - own_pos
    vec = (vec/np.linalg.norm(vec)) * accel_max * t_delta
    velo_restore = ((vec + own_velo)/np.linalg.norm(vec + own_velo)) * 15
    return velo_restore


def main_loop():
    own_pos = sim.get_own_pos()
    own_velo = sim.get_own_velo()
    occ_pos_all = sim.get_occ_pos()
    occ_velo_all = sim.get_occ_velo()
    rpz_all = sim.get_occ_rad()*d_lidar_correction + d_safety + d_buffer + own_rad
    occ_num = np.shape(occ_pos_all)[-1]

    #Calc Geometry
    doi_all = occ_pos_all - own_pos
    doi_norm_all = np.linalg.norm(doi_all, axis=-2)
    rvo_all = rpz_all*np.sqrt((doi_norm_all**2)-(rpz_all**2))/doi_norm_all #problem if same pos: doi_norm = 0
    dvo_all = ((doi_norm_all**2)-(rpz_all**2))/doi_norm_all

    #Calc Transform Matrix
    transform_all = get_transform_all(doi_all, occ_num) #each obstacles matrix is [:,:,num-1]

    #Select Mode
    mode = get_mode(transform_all, doi_norm_all, rpz_all, rvo_all, dvo_all, own_velo, occ_velo_all, occ_num)
    #print("Candidate Mode:" + str(mode))
    
    #Restore Test
    if (mode == 0):
        #Calculate Restore Velocity
        test_velo_restore = get_velo_restore(own_pos, own_velo)
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
            test_mode = get_mode(test_transform_all, test_doi_norm_all, rpz_all, test_rvo_all, test_dvo_all, test_velo_restore, occ_velo_all, occ_num)
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
        fa = fa_unit.copy()
        for k in range(2,(accel_max)+1):
            fa = np.concatenate((fa, fa_unit*k),-1)
        rv = fa*t_delta + own_velo
        rv = np.delete(rv, np.linalg.norm(rv, axis=-2)>own_velo_max, -1)

        if ((doi_norm_all-rpz_all>=0).all()): #outside rpz
            #Calc rav
            rav = rv.copy()
            for i in range(0,occ_num):
                rav_rel_trans = transform_all[:,:,i].T @ (rav-np.array([occ_velo_all[:,i]]).T)
                for j in range(0, np.shape(rav_rel_trans)[1]):
                    if ((rav_rel_trans[0,j] > 0 and np.sqrt(pow(rav_rel_trans[1,j],2)+pow(rav_rel_trans[2,j],2))/rav_rel_trans[0,j] < rvo_all[i]/dvo_all[i]) and (np.linalg.norm(rav_rel_trans[:,j]) > (doi_norm_all[i]-rpz_all[i])/t_horizon)): #not the exact collision time because of sphere curvature
                        rav[:,j] = np.array([[np.nan],[np.nan],[np.nan]]).T
                rav = rav[:, ~np.isnan(rav).any(axis=0)]
                if (np.size(rav) == 0):
                    break

            #Select Velocity #NEED TO VERIFY THIS
            if (np.size(rav) != 0): #Selects rav closest to velo_restore
                ind = np.argmin(np.linalg.norm(rav - get_velo_restore(own_pos, own_velo), axis=-2))
                velo_avoid = np.array([rav[:,ind]]).T

            else: #Selects most avoiding rv
                ind_closest_occ = np.argmin(doi_norm_all)
                occ_pos_rel = np.array([occ_pos_all[:,ind_closest_occ]-own_pos[:,0]]).T #relative to own
                rv_rel = rv-np.array([occ_velo_all[:,ind_closest_occ]]).T #relative to occ
                rdot = -1*(occ_pos_rel.T @ rv_rel)/np.linalg.norm(occ_pos_rel) #postiive means increasing doi
                ind = np.argmax(rdot)
                velo_avoid = np.array([rv[:,ind]]).T

        else: #within rpz #Selects most avoiding rv
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
        velo_restore = get_velo_restore(own_pos, own_velo)

        sim.set_own_velo(velo_restore)

    print(np.min(doi_norm_all - rpz_all))

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