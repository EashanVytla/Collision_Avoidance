import asyncio
import numpy as np
from scipy.spatial.transform import Rotation

from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw, PositionGlobalYaw, VelocityBodyYawspeed)

#notes: drone.something ex. drone.telemetry points to mavsdk.telemetry.Telemetry; other classes like telemetry.PositionBody need to be imported separately if needed
#ALGO Global Variables
own_rad = 0 #meters
d_lidar_correction = 1.05 #occ_rad scale to account for lidar error
d_safety = 5 #meters #minimmum distance allowed between closest points of own and occ (change this for more clearance)
d_buffer = 4 #meters #buffers own_pos from rpz by accounting for real world error, disturbances, and nonlinearity
d_horizon = 20 #meters #distance to consider collision
t_horizon = 10 #seconds #timeframe to consider collision

own_velo_max = 20 #m/s
accel_max = 8 #m/s^2
t_delta = .1 #s #timestep
t_delta_oscill = 1


async def run():
    """ Does Offboard control using position NED coordinates. """

    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    print("-- Arming")
    await drone.action.arm()

    print("-- Setting initial setpoint")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed \
                with error code: {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return

    #Avoidance Code
    #get own and occ info -> run algo (input: own and occ info, restore maneuver) (output: mode, avoidance manuever) -> set  manuever or relinquish control
    async for pos in drone.telemetry.position():
        phi1 = pos.latitude_deg
        lambda0 = pos.longitude_deg
        z = pos.absolute_altitude_m
        break
    x = 0
    y = 10 #i think this moves longitude "10 meters" north
    c = (x**2 + y**2)**(1/2)
    occ_pos_global = np.array([[0],[0],[0]])
    occ_pos_global[0] = np.arcsin(np.cos(c)*np.sin(phi1)+(y*np.sin(c)*np.cos(phi1)/c))
    if (phi1 != 90 and phi1 != -90):
        occ_pos_global[1,0] = lambda0 + np.arctan(x*np.sin(c)/(c*np.cos(phi1)*np.cos(c)-y*np.sin(phi1)*np.sin(c)))
    elif (phi1 == 90):
        occ_pos_global[1,0] = lambda0 + np.arctan(-x/y)
    else:
        occ_pos_global[1,0] = lambda0 + np.arctan(x/y)
    occ_pos_global[2,0] = z + 10

    x = 0
    y = 20 #i think this moves longitude "10 meters" north
    c = (x**2 + y**2)**(1/2)
    goal_global = np.array([[0],[0],[0]])
    goal_global[0,0] = np.arcsin(np.cos(c)*np.sin(phi1)+(y*np.sin(c)*np.cos(phi1)/c))
    if (phi1 != 90 and phi1 != -90):
        goal_global[1,0] = lambda0 + np.arctan(x*np.sin(c)/(c*np.cos(phi1)*np.cos(c)-y*np.sin(phi1)*np.sin(c)))
    elif (phi1 == 90):
        goal_global[1,0] = lambda0 + np.arctan(-x/y)
    else:
        goal_global[1,0] = lambda0 + np.arctan(x/y)
    goal_global[2,0] = z + 10

    while(True):
        async for pos in drone.telemetry.position():
            own_pos_global = np.array([[pos.latitude_deg],[pos.longitude_deg],[pos.absolute_altitude_m]])
            break
        occ_pos_ned = np.array([[0],[0],[0]])
        c = np.arccos(np.sin(own_pos_global[0,0])*np.sin(occ_pos_global[0,0])+np.cos(own_pos_global[0,0])*np.cos(occ_pos_global[0,0])*np.cos(occ_pos_global[1,0]-own_pos_global[1,0]))
        k = c/np.sin(c)
        occ_pos_ned[0,0] = k*(np.cos(own_pos_global[0,0])*np.sin(occ_pos_global[0,0])-np.sin(own_pos_global[0,0])*np.cos(occ_pos_global[0,0])*np.cos(occ_pos_global[1,0]-own_pos_global[1,0]))
        occ_pos_ned[1,0] = k*np.cos(occ_pos_global[0,0])*np.sin(occ_pos_global[1,0]-own_pos_global[1,0])
        occ_pos_ned[2,0] = -1*occ_pos_global[2,0]

        async for quat in drone.telemetry.attitude_quaternion():
            own_att_quat = np.array([quat.w, quat.x, quat.y, quat.z])
            break
        occ_pos_body = ned_to_body(np.array([occ_pos_ned[0,0], occ_pos_ned[1,0], occ_pos_ned[2,0]]),own_att_quat) #use reshape?
        occ_pos_body = np.array([[occ_pos_body[0]],[occ_pos_body[1]],[occ_pos_body[2]]]) #use reshape?

        goal_ned = np.array([[0],[0],[0]])
        c = np.arccos(np.sin(own_pos_global[0,0])*np.sin(goal_global[0,0])+np.cos(own_pos_global[0,0])*np.cos(goal_global[0,0])*np.cos(goal_global[1,0]-own_pos_global[1,0]))
        k = c/np.sin(c)
        goal_ned[0,0] = k*(np.cos(own_pos_global[0,0])*np.sin(goal_global[0,0])-np.sin(own_pos_global[0,0])*np.cos(goal_global[0,0])*np.cos(goal_global[1,0]-own_pos_global[1,0]))
        goal_ned[1,0] = k*np.cos(goal_global[0,0])*np.sin(goal_global[1,0]-own_pos_global[1,0])
        goal_ned[2,0] = -1*goal_global[2,0]

        goal_body = ned_to_body(np.array([goal_ned[0,0], goal_ned[1,0], goal_ned[2,0]]),own_att_quat) #use reshape?
        goal_body = np.array([[goal_body[0]],[goal_body[1]],[goal_body[2]]]) #use reshape?

        #ALGO
        occ_rad = np.array([1.5])
        own_pos = np.array([[0],[0],[0]])
        async for v in drone.telemetry.odometry():
            own_velo = np.array([[v.velocity_body.x_m_s],[v.velocity_body.y_m_s],[v.velocity_body.z_m_s]])
            break
        occ_pos_all = occ_pos_body
        occ_velo_all = np.array([[0],[0],[0]])
        rpz_all = occ_rad*d_lidar_correction + d_safety + d_buffer + own_rad
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
            test_velo_restore = get_velo_restore(own_pos, own_velo, goal_body)
            #Forecast Positions
            test_own_pos = own_pos + (test_velo_restore*t_delta_oscill)
            test_occ_pos_all = occ_pos_all + (occ_velo_all*t_delta_oscill)

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

                #Select Velocity
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

            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(velo_avoid[0,0],velo_avoid[1,0],velo_avoid[2,0],0))

        #MAINTAIN MODE
        elif (mode == 1):
            async for v in drone.telemetry.odometry():
                vx = v.velocity_body.x_m_s
                vy = v.velocity_body.y_m_s
                vz = v.velocity_body.z_m_s
                wy = v.angular_velocity_body.yaw_rad_s
                break
            await drone.offbard.set_velocity_body(VelocityBodyYawspeed(vx, vy, vz, wy))
        
        #RESTORE MODE #would give control to path planner irl
        else:
            velo_restore = get_velo_restore(own_pos, own_velo, goal_body)

            await drone.offboard.set_position_global(PositionGlobalYaw(goal_global[0,0],goal_global[1,0],goal_global[2,0],0,PositionGlobalYaw.AltitudeType(1)))


    #End Avoidance Code






    print("-- Stopping offboard")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed \
                with error code: {error._result.result}")



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

def get_velo_restore(own_pos, own_velo, goal):
    vec = goal - own_pos
    vec = (vec/np.linalg.norm(vec)) * accel_max * t_delta
    velo_restore = vec + own_velo
    return velo_restore

def ned_to_body(ned_coord, attitude_quaternion):
    # Convert attitude quaternion to rotation matrix
    r = Rotation.from_quat(attitude_quaternion)
    rotation_matrix = r.as_matrix()
    
    # Coordinate transformation from NED to body frame
    body_coord = np.dot(rotation_matrix, ned_coord)
    
    return body_coord



if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())