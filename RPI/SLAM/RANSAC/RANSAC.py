#Created by: Nicholas O'Brien
#Project Sentinel; RANSAC Functions
#Created: December 7th, 2019
import numpy as np
# import scipy.ndimage as spim
import time
import matplotlib.pyplot as plt
from numpy import array
from mpl_toolkits.mplot3d import Axes3D

###################################################################################################################################################################
#Function: calculateQ
#Purpose: calculate the 'q-angle' for a specified increment of the lidar at angle phi
#Inputs: 
    #theta, a float representing the mechanism motor's current angle
    #phi, a float representing the current increment's angle of measurement
#Outputs:
    #q, the q-angle for the specific increment.
def calculateQ(theta, phi):
    h = 118.636 #mm, physical meaning described in other documents
    r = 59.3179 #mm, physical meaning described in other documents
    q = np.arctan(r*np.cos(theta+phi)/h)
    return(q) #in radians


###################################################################################################################################################################
#Function: SampleUnassociatedPointsCartesian
#Purpose: generate a sample of keys from the Unassociated_Points to use to generate a sample of points for LSRP calculation, using angle increments as selection criteria.
#Inputs:
    #values, a list of values to the dictionary Unassociated_Points. The keys are tuples of the format (X, Y, Z)
#Outputs:
    #sample, a list of points for RANSAC. is a length between 4 and S points.
    #SKIP, a boolean used to tell RANSAC whether or not to skip the current sample of points. Only used if the sample is not large enough to calculate a plane.
def SampleUnassociatedPointsCartesian(values, S_LIM, S):
    SKIP = False
    
    valuelen = len(values)
    randIndex = int(np.random.randint(0, valuelen, 1)) #This selects a tuple from the keys list, holding the Cartesian coordinates for a point
    randPoint = values[randIndex]
##    print("Point with following coordinates has been chosen: " +str(randPoint))
    upperboundX = randPoint[0] + S_LIM
    lowerboundX = randPoint[0] - S_LIM
    upperboundY = randPoint[1] + S_LIM    
    lowerboundY = randPoint[1] - S_LIM
    upperboundZ = randPoint[2] + S_LIM
    lowerboundZ = randPoint[2] - S_LIM
        
    sample = []
    trycount=0
    while len(sample)<S and trycount<100*S:
        indexattempt = int(np.random.randint(0,valuelen, 1)) #pick a random index for keys
        #print(keyattempt)
        valueattempt = tuple(values[indexattempt])
        #Check that the point is within the phi-angle bounds
        Xcheck = (valueattempt[0]<upperboundX and valueattempt[0]>lowerboundX)
        
        #Check that the point is within the q-angle bounds and is not on the same scan-plane as the random point
        Ycheck = (valueattempt[1]<upperboundY and valueattempt[1]>lowerboundY)
        #Check that the point is within the radius bounds
        Zcheck = (valueattempt[2]<upperboundZ and valueattempt[2]>lowerboundZ)
        boundcheck = (Xcheck and Ycheck and Zcheck)
        if (valueattempt not in sample) and boundcheck:
            sample.append(valueattempt)
        trycount+=1
    if len(sample)<4:
        SKIP = True
    #print(len(sample), trycount)
    return(sample, SKIP)

###################################################################################################################################################################
#Function: RemovePoints
#Purpose: Remove points that are now associated with an LSRP from the Unassociated_Points Dictionary
#Inputs:
    #Unassociated, the dictionary containing all points currently unassociated with an LSRP
    #Associated, the dictionary containing all points currently associated with an LSRP
#Outputs:
    #None, this function operates directly on the Unassociated dictionary.
def RemovePoints(Unassociated, Associated):
    associatedkeys = Associated.keys()
    for asskey in associatedkeys: #This might take long, maybe there is a more efficient way to save some time.
        try:
            del(Unassociated[asskey])
        except:
            continue

###################################################################################################################################################################            
#Function: ExtractLSRP
#Purpose: calculate parameters to define a plane from a sample of points. Solves the equation beta1 + beta2*x + beta3*y = z
#Inputs:
    #Sample, a list of points of the format (rho, x, y, z)
#Outputs:
    #Betahat, a 3x1 numpy array containing the parameters to the plane, format [[beta1],[beta2],[beta3]]
    #SingularMatrix, a boolean specifying whether or not the Singular Matrix error appeared. Used for deciding whether or not to skip the current sample of points in RANSAC.
def ExtractLSRP(Sample):
    A = [[]]
    z = [[]]

    for point in Sample:
        x = np.float64(point[0])
        y = np.float64(point[1])
        zp = np.float64(point[2])
        if point is Sample[0]:
            A = [[1,x,y]]
            z = [[zp]]
        else:
            A = np.append(A,[[1,x,y]],0)
            z = np.append(z,[[zp]],0)
    Atrans = np.transpose(A)
    AtransA = np.matmul(Atrans,A)

    try:
        Betahat = np.linalg.solve(AtransA, np.matmul(Atrans, z)) #It seems to be somewhat common to get a singular matrix error at this step. It may be just with incorrect test data.
        SingularMatrix=False
        
    except:
        SingularMatrix = True #A singular matrix likely appears from points being co-linear, preventing a plane from being generated.
        Betahat = []
        #print(AtransA)
        print("A singular matrix has appeared! Skipping to another set of points...")
    return(Betahat, SingularMatrix)

###################################################################################################################################################################
#Function: TestTolerance
#Purpose: generate an array of the size of the dict of unassociated points, telling whether or not the points fall within tolerance (X) of the LSRP
#Inputs:
    #Unassociated_Points, the dictionary of all unassociated points
    #LSRP, a 3x1 numpy array containing the parameters for a plane, format [[beta1], [beta2], [beta3]]
#Outputs:
    #Tolerance_Bool, a dictionary with the same keys as Unassociated_Points, whose values are a boolean showing whether or not that point passed tolerance.
    #x, an int specifying how many points from Unassociated_Points passed tolerance.
    
def TestTolerance(Unassociated_Points, LSRP, X):
        beta1 = LSRP[0][0]
        beta2 = LSRP[1][0]
        beta3 = LSRP[2][0]
        beta = [[1,beta2,beta3]]
        betanorm = np.linalg.norm(beta)
        denominator = betanorm**2
        Tolerance_Bool = {}
        x = 0
        for key in Unassociated_Points:
            point = Unassociated_Points[key]
            x0 = point[0]
            y0 = point[1]
            z0 = point[2]
            vector = [[beta1], [x0], [y0]]
            numerator = np.dot(vector, beta) - z0
            t = numerator/denominator
            distance = (t*betanorm)[0][0] ##np.sqrt((t*beta2)**2 + (t*beta3)**2 + (t)**2)
            IN_TOLERANCE = abs(distance)<X
##            if distance<0: print("Found a negative distance value")
            Tolerance_Bool[key] = IN_TOLERANCE
            if IN_TOLERANCE:
                x += 1
        return(Tolerance_Bool, x)
    
###################################################################################################################################################################
#Function: LSRPtoLandmark
#Purpose: Convert the Parameters of the LSRP to a point in space that we can use for the EKF
#Inputs:
    #LSRP, a 3x1 numpy array containing the parameters for an LSRP, format [[beta1], [beta2], [beta3]]
#Outputs:
    #Landmark, a numpy array of the coordinates of the point used to represent the LSRP. format [[x_k],[y_k],[z_k]]

def LSRPtoLandmark(LSRP):
    fixed_point = [[0.0],[0.0],[0.0]]
    b1 = LSRP[0][0]
    b2 = LSRP[1][0]
    b3 = LSRP[2][0]
    b = np.array(b1/(1 + b2**2 + b3**2))
    normalvec = [[-b2],[-b3],[1]]
    Landmark = fixed_point + b*normalvec
    return(Landmark)

###################################################################################################################################################################
#Function: ConvertToCartesian
#Purpose: to convert a single scan's values into the global csys' cartesian coordinate system, taking into account the robots current pose
#Inputs:
    #res, a dict from the parser containing scan data
    #x, a nx1 numpy array, the state vector for our system
#Outputs:
    #scan, a dictionary containing all the points for a single scan, in the global coordinate system.
#NOTE: this function will have to be restructured when we start to understand how the dataflow
# from the LIDAR will look on the raspberry pi. For now, we assume we get a list of dictionaries for a frame.
# In reality, we expect to get a steady stream of new dicts every few hundredths of a second.
def ConvertToCartesian(res, x=[[0.0],[0.0],[0.0]]):
    scan = {}
    offset = 0
##    omega = np.pi/3 #radians per second, this variable is the angular frequency of the motor from the video. for testing purposes only.
    for index in range(0, len(res)):
        range_data = res[index]['Measurement']
        if type(range_data) is not list:
            try:
                range_data=eval(range_data)
            except:
                print("Error, measurements were not list or evaluatable by eval(), continuing to next scan.")
        if len(range_data)!=0:
##            if index==0:
##                start = res[index]['Time of transmission']
##            message_time = res[index]['Time of transmission']
##            time_since_start = message_time-start
            message_count = int(res[index]['Quantity'])
            angle_increment = np.radians(float(res[index]['Angular Increment']))
            start_angle = np.radians(float(res[index]['Start Angle']))
            end_angle = start_angle + (message_count-1)*angle_increment
##            theta = GetSensorEncoderData() #for when we have the arduino hooked up to the encoder, make sure it is in radians
##            theta = omega*time_since_start #the assignment here is just for visualization
            theta = np.radians(float(res[index]['Motor encoder']))
            q_0 = calculateQ(theta, 0)
            q_90 = calculateQ(theta, np.pi/2)
##            print("Index is "+str(index))
            for inc in range(0, message_count):
                angle = start_angle + inc*angle_increment
                rho = range_data[inc]
##                if rho<1000: #This is an attempt to remove datapoints that are too close to the origin.
##                    continue
                q = calculateQ(theta, angle)
                x_lidar = rho*np.cos(angle)*np.cos(q_0)
                y_lidar = rho*np.sin(angle)*np.cos(q_90)
                z_lidar = rho*(np.cos(angle)*np.sin(q_0) + np.sin(angle)*np.sin(q_90))
                x_global = x_lidar + x[0][0]
                y_global = y_lidar + x[1][0]
                z_global = z_lidar #For now, we assume sentinel does not change altitude. This is also making the global origin at the same height as the LIDAR
                scan[(angle, q, rho)] = (x_global,y_global,z_global)
            offset += message_count
        else:
            continue
    return(scan)
###############################################################################################################################################################
#Function: ConvertToCartesianEulerAngles
#Purpose: to convert a single scan's values into the global csys' cartesian coordinate system, taking into account the robots current pose
#Inputs:
    #res, a dict from the parser containing scan data
    #x, a nx1 numpy array, the state vector for our system
#Outputs:
    #scan, a dictionary containing all the points for a single scan, in the global coordinate system.
#NOTE: this function will have to be restructured when we start to understand how the dataflow
# from the LIDAR will look on the raspberry pi. For now, we assume we get a list of dictionaries for a frame.
# In reality, we expect to get a steady stream of new dicts every few hundredths of a second.
def ConvertToCartesianEulerAngles(res, x=[[0.0],[0.0],[0.0]]):
    scan = {}
    offset = 0
##    omega = np.pi/3 #radians per second, this variable is the angular frequency of the motor from the video. for testing purposes only.
    for index in range(0, len(res)):
        if len(res[index]['Measurement'])!=0:
##            if index==0:
##                start = res[index]['Time of transmission']
##            message_time = res[index]['Time of transmission']
##            time_since_start = message_time-start
            message_count = int(res[index]['Quantity'])
            angle_increment = np.radians(float(res[index]['Angular Increment']))
            start_angle = np.radians(float(res[index]['Start Angle']))
            range_data = eval(res[index]['Measurement'])
            end_angle = start_angle + (message_count-1)*angle_increment
##            theta = GetSensorEncoderData() #for when we have the arduino hooked up to the encoder, make sure it is in radians
##            theta = omega*time_since_start #the assignment here is just for visualization
##            theta = np.radians(res[index]['Motor encoder'])
##            print("Index is "+str(index))
            euler = eval(res[index]['euler'])
            phi = euler[0][0]
            theta = euler[1][0]
            psi = euler[2][0]

            Rlg = np.transpose([[np.cos(psi)*np.cos(theta), np.sin(psi)*np.cos(phi)+np.cos(psi)*np.sin(theta)*np.sin(phi), np.sin(psi)*np.sin(phi)-np.cos(psi)*np.sin(theta)*np.cos(phi)],
                   [-np.sin(psi)*np.cos(theta), np.cos(psi)*np.cos(phi)-np.sin(psi)*np.sin(theta)*np.sin(phi), np.cos(psi)*np.sin(phi)+np.sin(psi)*np.sin(theta)*np.cos(phi)],
                   [np.sin(theta), -np.cos(theta)*np.sin(phi), np.cos(theta)*np.cos(phi)]])


            for inc in range(0, message_count):
                angle = start_angle + inc*angle_increment
                rho = range_data[inc]
                
                q = calculateQ(theta, angle)
                x_lidar = rho*np.cos(angle)
                y_lidar = rho*np.sin(angle)
                z_lidar = 0
                point = np.asarray([[x_lidar], [y_lidar], [z_lidar]])
                newpoint = np.matmul(Rlg,point)+x

##                x_global = x_lidar + x[0][0]
##                y_global = y_lidar + x[1][0]
##                z_global = z_lidar #For now, we assume sentinel does not change altitude. This is also making the global origin at the same height as the LIDAR
                scan[(phi,theta,psi,rho,angle)] = newpoint
        else:
            continue
    return(scan)
###################################################################################################################################################################
#Function: ConvertToCartesianMedianFilter
#Purpose: to convert a single scan's values into the global csys' cartesian coordinate system, taking into account the robots current pose
#Inputs:
    #res, a dict from the parser containing scan data
    #x, a nx1 numpy array, the state vector for our system
    #size, a scalar that dictates the size of the sample taken from the array when filtering it. 
#Outputs:
    #scan, a dictionary containing all the points for a single scan, in the global coordinate system.
#NOTE: this function will have to be restructured when we start to understand how the dataflow
# from the LIDAR will look on the raspberry pi. For now, we assume we get a list of dictionaries for a frame.
# In reality, we expect to get a steady stream of new dicts every few hundredths of a second.
def ConvertToCartesianMedianFilter(res, x=[[0.0],[0.0],[0.0]], size=3):
    scan = {}
    offset = 0
##    omega = np.pi/3 #radians per second, this variable is the angular frequency of the motor from the video. for testing purposes only.
    for index in range(0, len(res)):
        if len(res[index]['Measurement'])!=0:
##            if index==0:
##                start = res[index]['Time of transmission']
##            message_time = res[index]['Time of transmission']
##            time_since_start = message_time-start
            message_count = res[index]['Quantity']
            angle_increment = np.radians(res[index]['Angular Increment'])
            start_angle = np.radians(res[index]['Start Angle'])
            range_data = spim.median_filter(res[index]['Measurement'], size=size, mode='reflect')
            end_angle = start_angle + (message_count-1)*angle_increment
##            theta = GetSensorEncoderData() #for when we have the arduino hooked up to the encoder, make sure it is in radians
##            theta = omega*time_since_start #the assignment here is just for visualization
            theta = np.radians(res[index]['Motor encoder'])
            q_0 = calculateQ(theta, 0)
            q_90 = calculateQ(theta, np.pi/2)
##            print("Index is "+str(index))
            for inc in range(0, message_count):
                angle = start_angle + inc*angle_increment
                rho = range_data[inc]
                if rho<1000: #This is an attempt to remove datapoints that are too close to the origin.
                    continue
                q = calculateQ(theta, angle)
                x_lidar = rho*np.cos(angle)*np.cos(q_0)
                y_lidar = rho*np.sin(angle)*np.cos(q_90)
                z_lidar = rho*(np.cos(angle)*np.sin(q_0) + np.sin(angle)*np.sin(q_90))
                x_global = x_lidar + x[0][0]
                y_global = y_lidar + x[1][0]
                z_global = z_lidar #For now, we assume sentinel does not change altitude. This is also making the global origin at the same height as the LIDAR
                scan[(angle, q, rho)] = (x_global,y_global,z_global)
            offset += message_count
        else:
            continue
    return(scan)
###################################################################################################################################################################
#Function: PairLandmarks
#Purpose: pair a newly calculated Landmark with the nearest Approved Landmark, that has been seen more than N_obsmin times
#Inputs:
    #Landmarks_New, a dict of 3x1 numpy arrays, the point landmarks we have extracted from the previous step
    #Landmark_Positions, a dict containing the number of times each landmark has been observed and its position in the state vector
    #x, the state vector
    #P, the covariance matrix. Only included for updating size.
#Outputs:
    #Minimum_Keys, a dict that has its keys being the same as Landmarks_Approved and its values being the associated point from Landmarks_New. This way, there is always a reference between the two points during the EKF.
def PairLandmarks(Landmarks_New, Landmark_Positions, x, P):
    multipliers = Landmark_Positions.keys()
    Landmarks_Approved = {}
    for multiplier in multipliers:
        landmark = x[(multiplier*3):((multiplier+1)*3)]
        Landmarks_Approved[multiplier] = landmark
    Minimum_Keys = {}
    Minimum_Distances = {}
    #This block is supposed to calculate the distance of each new landmark from each existing landmark and store it in a dict
    for appKey in Landmarks_Approved.keys():
        Approved = Landmarks_Approved[appKey]
        Distances_sub = []
        count = 0
        for newKey in Landmarks_New.keys():
            d = np.linalg.norm(Approved-Landmarks_New[newKey])
            if count==0:
                minimum_distance = d
                min_key = newKey
            elif d<minimum_distance:
                minimum_distance = d
                min_key = newKey
            count += 1
                
        Minimum_Distances[appKey] = minimum_distance
        Minimum_Keys[appKey] = min_key
        print("Observed Landmark "+str(appKey)+" has been paired with new landmark "+str(min_key))
    if len(Landmark_Positions)==0:
        print("No previous Landmarks in the database. Adding all new landmarks to the Current State")
        for New_Landmark in Landmarks_New.values():
            np.append(x, New_Landmark, 0)
            x_len = len(x)
            multiplier = int(x_len/3-1)
            Landmark_Positions[multiplier] = 1
            Minimum_Keys[multiplier] = None #This is made None to tell the EKF to skip this landmark. It has already been observed once and has no further value to the algorithm for this frame.
            P.resize((x_len, x_len)) #This should add rows and columns of zeroes to the matrix while preserving the original matrix contents
    return(Minimum_Keys)
#Some problems with the Above:
    #1. how to handle when a new landmark does not get paired with an approved landmark. It should be appended to the state vector and the Landmark_Positions dict
    #2. how to handle when there are too many approved landmarks for new landmarks. This algorithm will
    #   assign each approved landmark with a new landmark, even if that new landmark is shared with another approved landmark.
    #   Even so, is this a bad thing? Possibly not, the validation gate should in theory take care of that. But, what if it does not?
    #   Then a new landmark will be interpreted as two different approved landmarks at the same time. this could happen in the case of planes that
    #   intersect near the fixed point for calculating the point Landmark's coordinates. Even then again: is that a bad thing? More testing needed.


    #This block is for testing if any new landmarks were simultaneously the closest to multiple approved landmarks.
##    keyslist = []
##    for key1 in Minimum_Indexes.keys():
##        keyslist.append(key1)
##        identical_keys = []
##        for key2 in Minimum_Indexes.keys():
##            if (key1 != key2) and (key2 not in keyslist):
##                if Minimum_Indexes[key1] == Minimum_Indexes[key2]:
##                    if key1 not in identical_keys: identical_keys.append(key1)
##                    if key2 not in identical_keys: identical_keys.append(key2)
##        count = 0
##        while len(identical_keys)>1:
##            for index in range(0, len(identical_keys), 1):
##                d = Minimum_Distance[key]
##                if index==0:
##                    minimum = d
##                    min_index=index
##                elif minimum>d:
##                    minimum=d
##                    min_index=index
##                    #This might be a job for Harris.

    #Given a 2 sets of points, pair the points together based on closest distance. However
    #Every point can only have one pair. Not every point will have a pairing.
###################################################################################################################################################################
#Function: plotLSRPs
#Purpose: Plot any LSRP's found by RANSAC on an existing plot of the point cloud. This is only for verification testing.
#Inputs:
    #figure, the plt.figure() that holds the point cloud
    #LSRP_list, the list of all LSRP's found by RANSAC. These LSRP's are 1x3 numpy arrays holding the parameters for the plane's equation
    #xmin, the minimum bound for the plane to span in the x-direction.
    #xmax, the maximum bound for the plane to span in the x-direction. 
    #ymin and ymax are analagous to xmin and xmax for the y-direction.
    #alpha, the transparency of the plane. This is between 1 and 0, 1 being opaque, 0 being invisible.
#Outputs:
    #Nothing is returned. The function only plots the plane.
    
def plotLSRPs(figure, LSRP_list, xmin=-3000, xmax=3000, ymin=-3000, ymax=3000, alpha=0.5):
    xx, yy = np.meshgrid(range(xmin, xmax, 10), range(ymin, ymax, 10))
    for LSRP in LSRP_list:
        beta1 = LSRP[0][0]
        beta2 = LSRP[1][0]
        beta3 = LSRP[2][0]
        zz = beta1 + beta2*xx + beta3*yy
        #plt.hold #This is a deprecated function in more recent releases of matplotlib.pyplot. If the plane overwrites the points, try this.
        figure.plot_surface(xx, yy, zz, alpha=alpha)
###################################################################################################################################################################
#Function: RANSAC
#Purpose: run the RANSAC algorithm on a point cloud to extract LSRP's
#Inputs:
    #scan, the dictionary holding the point cloud.
    #start_angle, the starting measurement angle for the lidar
    #end_angle, the ending measurement angle for the lidar.
#Outputs:
    #Landmark_LSRPS, a list containing the coordinates of each LSRP.
    #LSRP_List, a list containing the parameters of the LSRP's. This is only for visualisation.
    
def RANSAC(scan, X, C, N, S, S_LIM):
    
    Landmarks_New = {}
    LSRP_list = []
    Associated_Points = {}
##    fig = plt.figure()
##    ax = Axes3D(fig)
##    ax.set_xlabel('X')
##    ax.set_ylabel('Y')
##    ax.set_zlabel('Z')
##    ax.set_xlim3d(-2000, 2000)
##    ax.set_ylim3d(-2000, 2000)
##    ax.set_zlim3d(-2000, 2000)
##    ax.view_init(45, 45)
    Unassociated_Points = scan #for now, we are black-boxing the function to get the frame data.
    n=0
    c = len(Unassociated_Points)
    print("#####################################################")
    while c>C and n<N:
        print("SAMPLE "+str(n))
        UnassociatedKeys = list(Unassociated_Points.keys())
        UnassociatedValues = list(Unassociated_Points.values())
        samplestart = time.time()
        (Sample_points, SKIP) = SampleUnassociatedPointsCartesian(UnassociatedValues, S_LIM, S) #Function to return indexes of the sample of points to calculate LSRP
        sampleend = time.time()
        print("It took "+str(sampleend-samplestart)+" seconds to generate a sample of "+str(len(Sample_points))+" points")
        if not SKIP:
##            Sample_points = [] #collector for Sample points
##            for key in Sample_Keys:
##                Sample_points.append(Unassociated_Points[key])
##            xs = []
##            ys = []
##            zs = []
##            for point in Sample_points:
##                xs.append(point[0])
##                ys.append(point[1])
##                zs.append(point[2])
##            ax.scatter(xs, ys, zs, s=1, marker='o', color='r')
##            plt.pause(1)
##            plt.show(False)
            start = time.time()
            (LSRP, Error) = ExtractLSRP(Sample_points) #Function to calculate the LSRP from the sampled points
            end = time.time()
            if not Error:
                print("Sample LSRP Calculated after "+str(end-start)+" seconds.")
                start = time.time()
                (Tolerance_Bool, x) = TestTolerance(Unassociated_Points, LSRP, X)
                end = time.time()
                print(str(x)+" Points were found in tolerance with the Sample LSRP after "+str(end-start)+" seconds.")
                
                if x>C:
                    print("CONSENSUS PASSED: CALCULATING LANDMARK LSRP")
                    InTolerance_Points = []
                    for key in UnassociatedKeys:
                        IN_TOLERANCE = Tolerance_Bool[key]
                        
                        if IN_TOLERANCE:
                            intolerance_point = Unassociated_Points[key]
                            InTolerance_Points.append(intolerance_point)
                            Associated_Points[key] = intolerance_point
                    start = time.time()
                    (LSRP, Error) = ExtractLSRP(InTolerance_Points)
                    end = time.time()
                    print("It took "+str(end-start)+" seconds to extract the Landmark LSRP.")
                    if not Error:
                        LSRP_list.append(LSRP) #This line should be removed in the instantiation of SENTINEL, this is only for visualization
                        Landmark = LSRPtoLandmark(LSRP)
                        Landmarks_New[n] = Landmark #n is assigned as the key because this dict is erased from each RANSAC run
                        RemovePoints(Unassociated_Points, Associated_Points)
                        print("NEW LANDMARK LSRP SUCCESSFULLY STORED. CONTINUING TO NEXT SAMPLE")
                else:
                    print("CONSENSUS FAILED: CONTINUING TO NEXT SAMPLE")
        
        c = len(Unassociated_Points)
        n += 1
        print("#####################################################")
    
    if n==N: print("Trial Limit of "+str(N)+" Reached")
    else: print(str(c) + " Unassociated Points are left. This is less than Consensus, "+str(C))
    return(Landmarks_New, LSRP_list, Unassociated_Points)
###################################################################################################################################################################
