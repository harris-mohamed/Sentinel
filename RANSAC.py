import numpy as np

#Constants for the RANSAC Process
X = 10.0 #The tolerance, how close points must be to the LSRP to be accurately represented by the LSRP, units are mm
C = 30 #The consensus, how many points must be within tolerance of the LSRP to pass, units are points
N = 200 #Max number of trials, units are trials
S = 10 #The number of points sampled around the initially chosen point
D = 10 #The number of degrees (angle) from S we are allowing ourselves to sample from, the search area, units are degrees
Drad = np.radians(D) #convert D to radians

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
#Function: UpdatePosition
#Purpose: Update the state vector of Sentinel, using information from the wheel encoders and the accelerometer
#The accelerometer has not been included yet.
#Inputs:
    #x, 1xn numpy array representing the state vector of Sentinel. The first three entries are the pose of Sentinel
    #dt1, a float representing the change in angle (in radians) of wheel 1
    #dt2, a float representing the change in angle (in radians) of wheel 2
#Outputs:
    #x, the updated state vector from Inputs.
    
def UpdatePosition(x,dt1,dt2):
    R = 1 #Radius of the wheels
    L = 1 #distance between the wheels
    dt = (R/L)*(dt2-dt1)
    if dt>=2*np.pi:
        dt += -2*np.pi
    elif dt<0:
        dt += 2*np.pi
    if dt==0:
        dx = -R*dt1*np.sin(x[2][0])
        dy = R*dt1*np.cos(x[2][0])
    else:
        dx = (L/2)*(dt2+dt1)/(dt2-dt1)*(np.cos(x[2][0]+dt)-np.cos(x[2][0]))
        dy = (L/2)*(dt2+dt1)/(dt2-dt1)*(np.sin(x[2][0]+dt)-np.sin(x[2][0]))
    x[0] += dx
    x[1] += dy
    x[2] += dt
    return(x)

###################################################################################################################################################################
#Function: SampleUnassociatedPoints
#Purpose: generate a sample of keys from the Unassociated_Points to use to generate a sample of points for LSRP calculation.
#Inputs:
    #keys, a list of keys to the dictionary Unassociated_Points. The keys are tuples of the format (phi, q)
    #start_angle, the angle that the scans begin at for the lidar's coordinate system, in radians
    #end_angle, the angle that the scans end at for the lidar's coordinate system, in radians.
#Outputs:
    #sample, a list of keys for RANSAC. is the length of S or fewer.
    #SKIP, a boolean used to tell RANSAC whether or not to skip the current sample of points. Only used if the sample is not large enough to calculate a plane.
def SampleUnassociatedPoints(keys, start_angle, end_angle):
    SKIP = False
    keylen = len(keys)
    start_angleq = np.radians(-26.565)
    end_angleq = np.radians(26.565) #this is the value of the max q angle that the mechanism can achieve with the values of h and r 
    randIndex = int(np.random.randint(0, keylen, 1)) #This selects a tuple from the keys list, holding the angle for phi and q for a specific point
    randPoint = keys[randIndex]
    upperboundphi = randPoint[0] + Drad
    lowerboundphi = randPoint[0] - Drad
    upperboundq = randPoint[1] + Drad
    lowerboundq = randPoint[1] - Drad
   
    if lowerboundphi < start_angle:
        lowerboundphi = start_angle
    if upperboundphi > end_angle:
        upperboundphi = end_angle
    if upperboundq > end_angleq:
        upperboundq = end_angleq
    if lowerboundq < start_angleq:
        lowerboundq = start_angleq
        
    sample = []
    trycount=0
    while len(sample)<S and trycount<10*S:
        indexattempt = int(np.random.randint(0,keylen, 1)) #pick a random index for keys
        #print(keyattempt)
        keyattempt = keys[indexattempt]
        phicheck = (keyattempt[0]<upperboundphi and keyattempt[0]>lowerboundphi)
        qcheck = (keyattempt[1]<upperboundq and keyattempt[1]>lowerboundq)
        boundcheck = (qcheck and phicheck)
        if (keyattempt not in sample) and boundcheck:
            sample.append(keyattempt)
        trycount+=1
    if len(sample)<3:
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
    #Betahat, a 1x3 numpy array containing the parameters to the plane, format [[beta1],[beta2],[beta3]]
    #SingularMatrix, a boolean specifying whether or not the Singular Matrix error appeared. Used for deciding whether or not to skip the current sample of points in RANSAC.
def ExtractLSRP(Sample):
    A = [[]]
    z = [[]]

    for point in Sample:
        x = np.float64(point[1])
        y = np.float64(point[2])
        zp = np.float64(point[3])
        if point is Sample[0]:
            A = [[1,x,y]]
            z = [[zp]]
        else:
            A = np.append(A,[[1,x,y]],0)
            z = np.append(z,[[zp]],0)
    Atrans = np.transpose(A)
    AtransA = np.matmul(Atrans,A)

    try:
        AtransAinv = np.linalg.inv(AtransA) #It seems to be somewhat common to get a singular matrix error at this step. It may be just with incorrect test data.
        Betahat = np.matmul(np.matmul(AtransAinv,Atrans),z)
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
    #LSRP, a 1x3 numpy array containing the parameters for a plane, format [[beta1], [beta2], [beta3]]
#Outputs:
    #Tolerance_Bool, a dictionary with the same keys as Unassociated_Points, whose values are a boolean showing whether or not that point passed tolerance.
    #x, an int specifying how many points from Unassociated_Points passed tolerance.
    
def TestTolerance(Unassociated_Points, LSRP):
        beta1 = LSRP[0]
        beta2 = LSRP[1]
        beta3 = LSRP[2]
        Tolerance_Bool = {}
        x = 0
        for key in Unassociated_Points:
            point = Unassociated_Points[key]
            x0 = point[1]
            y0 = point[2]
            z0 = point[3]
            numerator = beta1 + beta2*x0 + beta3*y0 - z0
            denominator = 1 + beta2**2 + beta3**2
            t = numerator/denominator
            distance = np.sqrt((t*beta2)**2 + (t*beta3)**2 + (t)**2)
            IN_TOLERANCE = distance<X
            Tolerance_Bool[key] = IN_TOLERANCE
            if IN_TOLERANCE:
                x += 1
        return(Tolerance_Bool, x)

###################################################################################################################################################################
#Function: LSRPtoLandmark
#Purpose: Convert the Parameters of the LSRP to a point in space that we can use for the EKF
#Inputs:
    #LSRP, a 1x3 numpy array containing the parameters for an LSRP, format [[beta1], [beta2], [beta3]]
#Outputs:
    #Landmark, a numpy array of the coordinates of the point used to represent the LSRP. format [[x_k],[y_k],[z_k]]

def LSRPtoLandmark(LSRP):
    fixed_point = np.array([[0],[0],[0]])
    b1 = np.float64(LSRP[0])
    b2 = np.float64(LSRP[1])
    b3 = np.float64(LSRP[2])
    b = b1/(1 + b2**2 + b3**2)
    normalvec = np.array([[-b2],[-b3],[1]])
    Landmark = fixed_point + b*normalvec
    return(Landmark)

###################################################################################################################################################################
#Function: ConvertToCartesian
#Purpose: to convert a single scan's values into the global csys' cartesian coordinate system, taking into account the robots current pose
#Inputs:
    #res, a dict from the parser containing scan data
    #x, a 1xn numpy array, the state vector for our system
#Outputs:
    #scan, a dictionary containing all the points for a single scan, in the global coordinate system.
#NOTE: this function will have to be restructured when we start to understand how the dataflow
# from the LIDAR will look on the raspberry pi. For now, we assume we get a list of dictionaries for a frame.
# In reality, we expect to get a steady stream of new dicts every few hundredths of a second.
def ConvertToCartesian(res, x):
    scan = {}
    offset = 0
    for index in range(0, len(res)):
        
        if len(res[index]['Measurement'])!=0:
            message_count = res[index]['Message Count']
            #angle_increment = np.radians(res[index]['Angular Increment']) #for when the angle increment value is returned correctly by the parser
            angle_increment = np.radians(270/811)
            start_angle = np.radians(res[index]['Start Angle'])
            range_data = res[index]['Measurement']
            end_angle = start_angle + (message_count-1)*angle_increment
            #theta = GetSensorEncoderData() #for when we have the arduino hooked up to the encoder, make sure it is in radians
            theta = index #the assignment here is just for visualization
            q_0 = calculateQ(theta, 0)
            q_90 = calculateQ(theta, np.pi/2)
            
            for inc in range(0, message_count):
                angle = start_angle + inc*angle_increment
                rho = range_data[inc]
                q = calculateQ(theta, angle)
                x_lidar = rho*np.cos(angle)*np.cos(q_0)
                y_lidar = rho*np.sin(angle)*np.cos(q_90)
                z_lidar = rho*(np.cos(angle)*np.sin(q_0) + np.sin(angle)*np.sin(q_90))
                x_global = x_lidar + x[0]
                y_global = y_lidar + x[1]
                z_global = z_lidar #For now, we assume sentinel does not change altitude. This is also making the global origin at the same height as the LIDAR
                scan[(angle, q)] = (rho,x_global,y_global,z_global)
            offset += message_count
        else:
            continue
    return(scan)

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
    
def RANSAC(scan, start_angle, end_angle):
    
    Landmark_LSRPS = []
    LSRP_list = []
    Associated_Points = {}
    
    Unassociated_Points = scan #for now, we are black-boxing the function to get the frame data.
    n=0
    c = len(Unassociated_Points)
    while c>C and n<N:
        UnassociatedKeys = list(Unassociated_Points.keys())

        (Sample_Keys, SKIP) = SampleUnassociatedPoints(UnassociatedKeys, start_angle, end_angle) #Function to return indexes of the sample of points to calculate LSRP
        
        if not SKIP:
            Sample_points = [] #collector for Sample points
            for key in Sample_Keys:
                Sample_points.append(Unassociated_Points[key])
            (LSRP, Error) = ExtractLSRP(Sample_points) #Function to calculate the LSRP from the sampled points

            if not Error:
                print("LSRP Calculated, now testing Unassociated Points against tolerance...")
                (Tolerance_Bool, x) = TestTolerance(Unassociated_Points, LSRP)
                print(str(x)+" Points were found in tolerance!")
                
                if x>C:
                    print("This Passes the consensus! Now Calculating new LSRP from in tolerance points...")
                    InTolerance_Points = []
                    for key in UnassociatedKeys:
                        IN_TOLERANCE = Tolerance_Bool[key]
                        
                        if IN_TOLERANCE:
                            intolerance_point = Unassociated_Points[key]
                            InTolerance_Points.append(intolerance_point)
                            Associated_Points[key] = intolerance_point
                    (LSRP, Error) = ExtractLSRP(InTolerance_Points)
                        
                    if not Error:
                        LSRP_list.append(LSRP) #This line should be removed in the instantiation of SENTINEL, this is only for visualization
                        Landmark = LSRPtoLandmark(LSRP)
                        Landmark_LSRPS.append(Landmark)
                        RemovePoints(Unassociated_Points, Associated_Points)
       
        c = len(Unassociated_Points)
        n += 1
    
    if n==N: print("Trial Limit of "+str(N)+" Reached")
    else: print(str(c) + " Unassociated Points are left. This is less than Consensus, "+str(C))
    return(Landmark_LSRPS, LSRP_list)
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
        beta1 = LSRP[0]
        beta2 = LSRP[1]
        beta3 = LSRP[2]
        zz = beta1 + beta2*xx + beta3*yy
        #plt.hold #This is a deprecated function in more recent releases of matplotlib.pyplot. If the plane overwrites the points, try this.
        figure.plot_surface(xx, yy, zz, alpha=alpha)
###################################################################################################################################################################
