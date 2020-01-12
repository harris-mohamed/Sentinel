#Created by: Nicholas O'Brien
#Project Sentinel; EKF Functions
#Created: December 8th, 2019
import numpy as np
#Global Variables
L = 1 #distance between two propulsion wheels, mm
R = 1 #Radius of wheel, mm
threshold = 1 #The validation gate's threshold value
N_obsmin = 2 #minimum number of times a landmark must be observed to be used in the EKF
v_rho = 1 #variance of landmark's range value from robot
v_phi = 1 #variance of landmark's phi angle from robot
v_q = 1 #variance of landmark's q-value

Q = [[v_rho, 0, 0],
     [0, v_phi, 0],
     [0, 0, v_q]] #matrix for representing the error in measurement
###################################################################################################################################################################
#Function: UpdatePosition
#Purpose: Update the state vector of Sentinel, using information from the wheel encoders and the accelerometer
#The accelerometer has not been included yet.
#Inputs:
    #x, nx1 numpy array representing the state vector of Sentinel. The first three entries are the pose of Sentinel
    #dx_sum, 3x1 numpy array that is for tracking the total change in pose of Sentinel from the beginning of a new frame
    #dt1, a float representing the change in angle (in radians) of wheel 1
    #dt2, a float representing the change in angle (in radians) of wheel 2
#Outputs:
    #x, the updated state vector from Inputs.
    #dx_sum, the updated pose-change array
def UpdatePosition(x,dx_sum,dt1,dt2):
    theta = x[2][0]
    dt = (R/L)*(dt2-dt1)
    if dt>=2*np.pi:
        dt += -2*np.pi
    elif dt<0:
        dt += 2*np.pi
    if dt==0:
        dx = -R*dt1*np.sin(theta)
        dy = R*dt1*np.cos(theta)
    else:
        dx = (L/2)*(dt2+dt1)/(dt2-dt1)*(np.cos(theta+dt)-np.cos(theta))
        dy = (L/2)*(dt2+dt1)/(dt2-dt1)*(np.sin(theta+dt)-np.sin(theta))
    f = [[dx],[dy],[dt]]
    f_adjusted = np.append(f, np.zeros((len(x)-3,1)),0)
    x += f_adjusted
    dx_sum += f
    return(x, dx_sum)

###################################################################################################################################################################
#Function: Update_PredictionModel
#Purpose: Update the covariance matrix, P, with information from the prediction model
#Inputs:
    #P, the covariance matrix from the previous frame
    #dx_sum, 3x1 numpy array that holds the displacement of Sentinel with respect to its pose at the beginning of the current frame
#Outputs:
    #none, this function updates the covariance matrix, P.
def Update_PredictionModel(P, dx_sum):
    c = 0.1 #c is a dimensionless scaling parameter for calculating the prediction model's error matrix, R
    dxr = dx_sum[0][0]
    dyr = dx_sum[1][0]
    dtr = dx_sum[2][0]
    
    A = np.eye(len(P))
    A[0][2] = -dyr
    A[1][2] = dxr
    print(A)

    R = np.multiply(c,[[dxr**2, dxr*dyr, dxr*dtr],
         [dyr*dxr, dyr**2, dyr*dtr],
         [dtr*dxr, dtr*dyr, dtr**2]])
    
    R.resize((6,6))
    Atrans = np.transpose(A)
    P = np.matmul(A,np.matmul(P,Atrans)) + R
    return(P)
###################################################################################################################################################################
#Function: CalculatehandH
#Purpose: calculate the measurement prediction model array and its jacobian matrix for a specific landmark
#Inputs:
    #Point, a 3x1 numpy array holding the x, y, z coordinates of a calculated point landmark.
    #x, an nx1 numpy array with the first 3 values being the current pose of Sentinel
    #CALCULATEJACOBIAN, a boolean that tells the function whether or not to calculate H
#Outputs:
    #h, a 3x1 numpy array holding the range, bearing, and angle from sentinel of a specific landmark
    #H, a 3x6 numpy array representing the jacobian of the measurement prediction model
def CalculatehandH(Point, x, CALCULATEJACOBIAN=True):
    xk = Point[0][0]
    yk = Point[1][0]
    zk = Point[2][0]
    xr = x[0][0]
    yr = x[1][0]
    tr = x[2][0]
    zr = 0 #for now, we are assuming the height of sentinel to not change. If we stop assuming that, this must be updated.

    dx = xk-xr
    dy = yk-yr
    dz = zk-zr
    ak = np.sqrt(dx**2 + dy**2)
    
    rhok = np.sqrt(dx**2 + dy**2 + dz**2)
    phik = np.arctan(dy/dx) - tr
    qk = np.arctan(dz/ak)
    h = [[rhok],[phik],[qk]]
    if CALCULATEJACOBIAN:
        aksq = ak**2
        rhoksq = rhok**2
        
        Hk11 = -dx/rhok
        Hk12 = -dy/rhok
        Hk13 = 0.0
        Hk14 = -Hk11
        Hk15 = -Hk12
        Hk16 = dz/rhok

        Hk21 = dy/(aksq)
        Hk22 = -dx/(aksq)
        Hk23 = -1.0
        Hk24 = -Hk21
        Hk25 = -Hk22
        Hk26 = 0.0

        Hk31 = (dx*dz)/(rhoksq*ak)
        Hk32 = (dy*dz)/(rhoksq*ak)
        Hk33 = 0.0
        Hk34 = -Hk31
        Hk35 = -Hk32
        Hk36 = ak/rhoksq

        H = [[Hk11, Hk12, Hk13, Hk14, Hk15, Hk16],
             [Hk21, Hk22, Hk23, Hk24, Hk25, Hk26],
             [Hk31, Hk32, Hk33, Hk34, Hk35, Hk36]]
    else: H=None    
    return(h,H)
###################################################################################################################################################################
#Function: ValidationGate
#Purpose: determine whether or not a pair of landmarks passes our validation gate based on the innovation vector and matrix
#Inputs:
    #nuk, a 3x1 numpy array that is the innovation vector
    #Skinv, a 3x3 numpy array that is the innovation covariance matrix
#Outputs:
    #boolean that is true if they pass the gate, false if they do not.
def ValidationGate(nuk, Skinv):
    nuktrans = np.transpose(nuk)
    value = np.matmul(nuktrans,np.matmul(Skinv, nuk))
    return(threshold>=value)
###################################################################################################################################################################
#Function: CalculateSInverse
#Purpose: Calculate the innovation covariance matrix for the current landmark
#Inputs:
    #P, the current covariance matrix, 3n+1 by 3n+1
    #Hk, the measurement model's jacobian matrix, scaled up to the higher dimension of P
#Outputs:
    #Skinv, the inverse of the innovation covariance matrix. The inverse is returned because that is the only form we use. should be 3x3
def CalculateSInverse(P, Hk):
    Hktrans = np.transpose(Hk)
    product = np.matmul(Hk, np.matmul(P, Hktrans))
    Sk = product+Q #Q is the global matrix variable for the measurement model's variation
    Skinv = np.linalg.inv(Sk)
    return(Skinv)
###################################################################################################################################################################
#Function: Hk_LowtoHigh
#Purpose: bring the measurement jacobian matrix up to the size of the covariance matrix
#Inputs:
    #Hk_Low, the jacobian matrix as it comes from the CalculatehandH function
    #multiplier, the current landmark's position multiplier
    #x_len, the current length of the state vector
#Outputs:
    #Hk_High, the jacobian matrix of the measurement model in the dimension of the covariance matrix. Should be 6 by x_len
def Hk_LowtoHigh(Hk_Low, multiplier, x_len):
    Omega = np.zeros((6, x_len))

    Omega[0][0] = 1
    Omega[1][1] = 1
    Omega[2][2] = 1

    Omega[3][3*multiplier] = 1
    Omega[4][3*multiplier+1] = 1
    Omega[5][3*multiplier+2] = 1

    Hk_High = np.matmul(Hk_Low, Omega)
    return(Hk_High)
###################################################################################################################################################################
#Function: Update_MeasurementModel
#Purpose: Calculate the Kalman gain for the current landmark.
#Inputs:
    #P, the Covariance Matrix x_len by x_len
    #Hk, the jacobian of the measurement model in the dimension of the covariance matrix, 6 by x_len
    #Skinv, the inverse of the innovation covariance matrix 3x3
    #x, the state vector, x_len x 1
    #nuk, the innovation vector 3x1
#Outputs:
    #x_prime, updated state vector
    #P_prime, updated covariance matrix
def Update_MeasurementModel(P, x, nuk, Skinv, Hk):
    Hktrans = np.transpose(Hk)
    Kk = np.matmul(P, np.matmul(Hktrans, Skinv))
    x_prime = x + np.matmul(Kk, nuk)
    print(Kk)
    print(Hk)
    P_prime = np.matmul((np.eye(len(x))-np.matmul(Kk, Hk)),P)
    return(x_prime, P_prime)
###################################################################################################################################################################
#Function: EKF
#Purpose: Act as the master function for the Extended Kalman Filter calculations
#Inputs:
    #x, the state vector
    #P, the covariance matrix
    #Landmark_Positions, a dict containing each landmark's positions in the state vector
    #Landmarks_New, a dict containing 3x1 numpy arrays for each new landmark from the current fram
    #Landmark_Pairs, a dict containing the link between the keys of the Landmark_Positions and Landmarks_New dicts
#Outputs:
    #x, the updated state vector
    #P, the updated covariance matrix
def EKF(x, dx_sum, P, Landmark_Positions, Landmarks_New, Landmark_Pairs):
    print("Updating the Covariance Matrix according to Sentinel's Position now...")
    P = Update_PredictionModel(P, dx_sum)
    multipliers = Landmark_Positions.keys()
    x_len = len(x)
    for multiplier in multipliers:
        if Landmark_Pairs[multiplier]==None:
            continue
        print("Updating Landmark "+str(multiplier)+" in the Covariance Matrix and State Vector")
        N_obs = Landmark_Positions[multiplier]
        Landmark_New = Landmarks_New[Landmark_Pairs[multiplier]]
        Landmark_Old = x[(multiplier*3):((multiplier+1)*3)] #Pick the landmark from the state
        (hk, Hk_Low) = CalculatehandH(Landmark_Old, x)
        Hk = Hk_LowtoHigh(Hk_Low, multiplier, x_len)
        Skinv = CalculateSInverse(P, Hk)
        nuk = Landmark_New - hk
        SAMELANDMARK = ValidationGate(nuk, Skinv)
        if SAMELANDMARK: #If they were the same landmark, add 1 to N_obs
            N_obs += 1
            Landmark_Positions[multiplier] = N_obs
        else: #If not, it is a new landmark. Set N_obs to 1, assign to the Landmarks dicitonary, and append to state vector
            N_obs = 1
            np.append(x, Landmark_New, 0)
            x_len = len(x)
            multiplier = int(x_len/3-1)
            Landmark_Positions[multiplier] = N_obs

            #These next lines are adding 3 columns and 3 rows of zeros to the covariance matrix. This is likely not the most efficient way.
            P.resize((x_len, x_len))
        VALID = (N_obs>N_obsmin)
        if VALID: 
            (x, P) = Update_MeasurementModel(P, x, nuk, Skinv, Hk)
    return(x, P)
            
        
