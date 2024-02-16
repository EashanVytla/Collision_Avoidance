from simulator import Simulator
import numpy as np
import matplotlib.pyplot as plt
import math

own_rad = 0 #meters
d_lidar_correction = 1.05 #occ_rad scale to account for lidar error
d_safety = 5 #meters #minimmum distance allowed between closest points of own and occ
d_horizon = 20 #meters #distance to consider collision
t_horizon = 10 #seconds #timeframe to consider collision
d_buffer = 1 #meters #buffers own_pos from RPZ by accounting for real world error and disturbances

def init():

    return

def get_transform(D,num): #Fails if in same pos: doi (d) = 0
    TRANSFORM = np.array([[[],[],[]],[[],[],[]],[[],[],[]]])
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
        transform_matrix = np.linalg.inv(np.concatenate((e1,e2,e3),-1)) #maybe just build matrix transposed since inv should = transpose
        transform_matrix = np.array([np.array([transform_matrix[:,0]]).T, np.array([transform_matrix[:,1]]).T, np.array([transform_matrix[:,2]]).T])
        TRANSFORM = np.concatenate((TRANSFORM, transform_matrix), -1)
    return TRANSFORM #each obstacles matrix is [:,:,num-1]

def get_mode(TRANSFORM, DOI_NORM, RPZ, v1, V2, num): #Probably can restructure #dont need some booleans
    #Restore --> mode 0; Maintain --> mode 1; Avoid --> mode 2
    MODE = np.array([])
    RBPZ = RPZ + d_buffer #this is so we go into avoid when heading into buffer zone
    RBVO = RBPZ*np.sqrt((DOI_NORM**2)-(RBPZ**2))/DOI_NORM #problem if same pos: doi_norm = 0 #RN having problem where doi_norm < RBPZ (buffer related issue)
    DBVO = ((DOI_NORM**2)-(RBPZ**2))/DOI_NORM
    for i in range(0,num):
        transform = TRANSFORM[:,:,i].T
        doi_norm = DOI_NORM[i]
        rbvo = RBVO[i]
        dbvo = DBVO[i]
        rbpz = RBPZ[i]
        v2 = np.array([V2[:,i]]).T
        #Check t_imminent and colliding
        velo_rel_transform = transform @ (v1-v2)
        if (velo_rel_transform[0,0] <= 0 or np.sqrt(pow(velo_rel_transform[1,0],2)+pow(velo_rel_transform[2,0],2))/velo_rel_transform[0,0] >= rbvo/dbvo):
            colliding = False
            t_imminent = False
        elif(np.linalg.norm(velo_rel_transform) > ((doi_norm-rbpz)/t_horizon)): #not the exact collision time because of sphere curvature
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
        MODE = np.append(MODE, mode)
    mode = np.max(MODE)
    return mode


def main_loop():
    own_pos = sim.get_own_pos()
    own_velo = sim.get_own_velo()
    occ_pos = sim.get_occ_pos()
    occ_velo = sim.get_occ_velo()
    RPZ = sim.get_occ_rad()*d_lidar_correction + d_safety + own_rad
    occ_num = np.shape(occ_pos)[-1]

    #Calc Geometry
    DOI = occ_pos - own_pos
    DOI_NORM = np.linalg.norm(DOI, axis=-2)
    RVO = RPZ*np.sqrt((DOI_NORM**2)-(RPZ**2))/DOI_NORM #problem if same pos: doi_norm = 0
    DVO = ((DOI_NORM**2)-(RPZ**2))/DOI_NORM

    #Calc Transform Matrix
    TRANSFORM = get_transform(DOI, occ_num) #each obstacles matrix is [:,:,num-1]

    #Select Mode
    mode = get_mode(TRANSFORM, DOI_NORM, RPZ, own_velo, occ_velo, occ_num)
    #print("Candidate Mode:" + str(mode))
    
    #Restore Test
    if (mode == 0):
        #Calculate Restore Velocity
        test_vec = np.array([[0, 0, 0]]).T - own_pos
        test_velo_restore = ((test_vec / np.linalg.norm(test_vec))) * 12 #mess around with this, idk what the control will look like
        #Forecast Positions
        test_own_pos = own_pos + (test_velo_restore/10) #10 is simulated_fps
        test_occ_pos = occ_pos + (occ_velo/10) #10 is simulated_fps

        #Calc Geometry & Get Mode
        test_DOI = (test_occ_pos - test_own_pos)
        test_DOI_NORM = np.linalg.norm(test_DOI, axis=-2)
        if (np.min(test_DOI_NORM-RPZ) > 0):
            test_RVO = RPZ*np.sqrt((test_DOI_NORM**2)-(RPZ**2))/test_DOI_NORM #problem if same pos: doi_norm = 0
            test_DVO = ((test_DOI_NORM**2)-(RPZ**2))/test_DOI_NORM

            #Calc Transform Matrix
            test_TRANSFORM = get_transform(test_DOI, occ_num)

            #Get Mode
            test_mode = get_mode(test_TRANSFORM, test_DOI_NORM, RPZ, test_velo_restore, occ_velo, occ_num)
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
        FAunit = np.array([[0,0,1],[0,0,-1]]).T
        for phi in phi_range:
            for theta in theta_range:
                v = np.array([[np.sin(theta)*np.cos(phi),np.sin(theta)*np.sin(phi),np.cos(theta)]]).T
                FAunit = np.concatenate((FAunit, v),-1)
        delta_t = .1 #s #timestep
        accel_max = 8 #m/s^2
        FA = FAunit.copy()
        for k in range(2,(accel_max)+1):
            FA = np.concatenate((FA, FAunit*k),-1)
        RV = FA*delta_t + own_velo

        #Calc RAV
        RAV = RV.copy()
        RRV_TRANSFORM_all = np.array([[],[],[]])
        for i in range(0,occ_num):
            RRV_TRANSFORM = TRANSFORM[:,:,i].T @ (RAV-np.array([occ_velo[:,i]]).T)
            RRV_TRANSFORM_all = np.concatenate((RRV_TRANSFORM_all, RRV_TRANSFORM), -1)
            for j in range(0, np.shape(RRV_TRANSFORM)[1]):
                if ((RRV_TRANSFORM[0,j] > 0 and np.sqrt(pow(RRV_TRANSFORM[1,j],2)+pow(RRV_TRANSFORM[2,j],2))/RRV_TRANSFORM[0,j] < RVO[i]/DVO[i]) and (np.linalg.norm(RRV_TRANSFORM[:,j]) > (DOI_NORM[i]-RPZ[i])/t_horizon)): #not the exact collision time because of sphere curvature
                    RAV[:,j] = np.array([[np.nan],[np.nan],[np.nan]]).T
            RAV = RAV[:, ~np.isnan(RAV).any(axis=0)]
            if (np.size(RAV) == 0):
                break

        #Select Velocity #NEED TO VERIFY THIS #getting some nan (rarely) in DIST #getting different values each run #rn get_mode selects avoid if going into buffer zone so that own doesnt maintain into buffer
        if (np.size(RAV) != 0): #describe this loop once finished
            #Find minimum distance between velocity path and all obstacles+bufferzone
            DIST = np.empty((1,np.shape(RAV)[1]))
            for i in range(0,occ_num):
                rel_occ_pos = np.array([occ_pos[:,i]-own_pos[:,0]]).T
                RRAV = RAV-np.array([occ_velo[:,i]]).T
                COMP = (rel_occ_pos.T @ RRAV) / np.linalg.norm((RRAV), axis=-2)
                COMP[COMP < 0] = 0 #probably not most efficient way to account for negative values, but it works (origin is closest point)
                PROJ = (RAV / np.linalg.norm((RAV), axis=-2)) * COMP
                TIME = COMP / np.linalg.norm((RAV), axis=-2)
                TIME = TIME[0,:]
                PROJ[:, TIME > t_horizon] = RAV[:, TIME > t_horizon] * t_horizon
                DIST = np.concatenate((DIST, np.array([(np.linalg.norm((PROJ - rel_occ_pos), axis=-2)) - RPZ[i] - d_buffer])), -2)
            DIST = DIST.min(axis=-2)
            #print(DIST[3:6])
            #Select velocity corresponding to minimum positive DIST (or maximum negative DIST when there's no positive values)
            if(np.size(DIST[DIST>0]) != 0):
                DIST_POS = DIST.copy()
                DIST_POS[DIST_POS<0] = np.inf
                ind = np.argmin(DIST_POS)
                print("Positive")
                print(DIST[ind])
            else:
                ind = np.argmax(DIST)
                print("Negative")
                print(DIST[ind])
            velo_avoid = np.array([RAV[:,ind]]).T

        else: #selects first V in RV that is farthest from collision (doi vector)
            RRV_unit_TRANSFORM = RRV_TRANSFORM_all / np.linalg.norm(RRV_TRANSFORM_all, axis=-2)
            RRV_angle = np.arccos(np.clip(np.array([[1,0,0]]) @ RRV_unit_TRANSFORM, -1.0, 1.0))
            ind = np.argmax(np.absolute(RRV_angle))
            velo_avoid = np.array([RV[:,ind]]).T

        sim.set_own_velo(velo_avoid)

    #MAINTAIN MODE
    elif (mode == 1):
        sim.set_own_velo(own_velo)
    
    #RESTORE MODE #would give control to path planner irl
    else:
        vec = np.array([[0, 0, 0]]).T - own_pos
        velo_restore = ((vec / np.linalg.norm(vec))) * 12 #mess around with this, idk what the control will look like
        sim.set_own_velo(velo_restore)

    #print(DOI_NORM[0] - RPZ[0])


if __name__ == "__main__":
    sim = Simulator()
    init()
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    init()

    for t in range(int(sim.reaction_time * sim.simulated_fps) + 5*200):
        main_loop()
        sim.update_positions()
        sim.plot_3d_environment(ax)

    plt.show()