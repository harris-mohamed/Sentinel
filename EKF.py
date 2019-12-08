#Created by: Nicholas O'Brien
#Project Sentinel; EKF Functions
#Created: December 8th, 2019
import numpy as np
#Global Variables
L = 1 #distance between two propulsion wheels, mm
R = 1 #Radius of wheel, mm
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
#Function: CalculateAandR
#Purpose: Create the Jacobian matrix and the error matrix of the prediction model
#Inputs:
    #dx_sum, 3x1 numpy array that holds the displacement of Sentinel with respect to its pose at the beginning of the current frame
#Outputs:
    #A, a 3x3 numpy array that represents the Jacobian matrix of the prediction model.
    #R, a 3x3 numpy array that represents the error of the prediction model
def CalculateAandR(dx_sum):
    c = 0.1 #c is a dimensionless scaling parameter for calculating the prediction model's error matrix, R
    dxr = dx_sum[0][0]
    dyr = dx_sum[1][0]
    dtr = dx_sum[2][0]
    
    A = [[1.0, 0.0, -dyr],
         [0.0, 1.0, dxr],
         [0.0, 0.0, 1.0]]

    R = c*[[dxr**2, dxr*dyr, dxr*dtr],
         [dyr*dxr, dyr**2, dyr*dtr],
         [dtr*dxr, dtr*dyr, dtr**2]]
    return(A, R)
###################################################################################################################################################################
#Function: UpdateP_PredictionModel
#Purpose: Update the covariance matrix, P, with information from the prediction model
#Inputs:
    #P, the covariance matrix from the previous frame
    #A, a 3x3 numpy array, Jacobian matrix of the Prediction model
    #R, 3x3 numpy array, error of the prediction model
#Outputs:
    #none, this function updates the covariance matrix, P.
def UpdateP_PredictionModel(P, A, R):
    Atrans = np.transpose(A)
    P_position = P[0:3][0:3]
    P_prime = np.matmul(A,np.matmul(P_position,Atrans)) + R
    P[0:3][0:3] = P_prime
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
def EKF(x, dx_sum, P, Landmarks):
    (A, R) = CalculateAandR(dx_sum)
    UpdateP_PredictionModel(P, A, R)
    for Landmark in Landmarks.values():
        (hk, Hk) = CalculatehandH(Landmark, x)
        Sk = CalculateS(P, Hk)
        nuk = Calculatenu(hk, Landmark)
        
        
