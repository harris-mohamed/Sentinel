#Created by: Nicholas O'Brien
#Project Sentinel; Kalman Filter Library
from RANSAC.RANSAC import calculateQ
import numpy as np
#import scipy.linalg as slp

###################################################################################################################################################################
#Function: Predict
#Purpose: use readings from the gyroscope and the mechanism encoder to estimate the current orientation of the sensor.
#Inputs: 
    #xk_1, a 3x1 numpy array that holds the last accepted orientation of the sensor
    #Pk_1, a 3x3 numpy array that holds the covariances for the last accepted orientation
    #omega, a 3x1 numpy array that is the format [[wx], [wy],[wz]] from the gyroscope
    #dt, the time lapse between this calculation and the time at which xk_1 was last calculated.
    #Qk, a 3x3 numpy array that is the error for the prediction step

#Outputs:
    #xk_new, a 3x1 numpy array that is the new accepted orientation of the sensor
    #Pk_new, a 3x3 numpy array that is the new covariance matrix

#NOTES:
    #The orientation of the gyroscope's axes is the following: {x' is aligned with -x_lidar, y' is aligned with y_lidar, z' is aligned with -z_lidar}
def Predict(xk_1, Pk_1, omega, dt, Qk):

    phi =xk_1[0][0]
    theta = xk_1[1][0]
    Dk_1 = [[0, np.sin(phi)*np.tan(theta), np.cos(phi)*np.tan(theta)],
            [1, np.cos(phi), -np.sin(phi)],
            [0, np.sin(phi), np.cos(phi)/np.cos(theta)]]

    Phik = np.diag([np.exp(dt),np.exp(dt),np.exp(dt)])
##    Phik = slp.expm(np.multiply(dt,Dk_1))
    
    xk_newpre = np.matmul(Dk_1,omega)*dt+xk_1
    Pk_newpre = np.matmul(np.matmul(Phik,Pk_1),np.linalg.inv(Phik))+Qk
    
    return(xk_newpre,Pk_newpre)
###################################################################################################################################################################
#Function: Gravity
#Purpose: use readings from the gyroscope and the mechanism encoder to estimate the current orientation of the sensor.
#Inputs: 
    #acc, the acceleration of gravity in the accelerometer's coordinate system.

#Outputs:
    #xk_estimate, the euler angles of the lidar's csys from the global.

#NOTES:
    #The orientation of the gyroscope's axes is the following: {x' is aligned with -x_lidar, y' is aligned with y_lidar, z' is aligned with -z_lidar}
def Gravity(acc):
    ax = acc[0][0]
    ay = acc[1][0]
    az = acc[2][0]
    if az!=0:        
        phi = -np.arctan(ay/az)
    elif ay>0: phi=-np.pi/2
    else: phi=np.pi/2
    
    if ay!=0:
        theta = np.arctan(ax*np.sin(phi)/ay)
    elif ax>0: theta=np.pi/2
    else: theta = -np.pi/2
    psi = 0
    xk_estimate = np.asarray([[phi],[theta],[psi]])
    return(xk_estimate)

if __name__=="__main__":
    xk = [[0],[0],[0]]
    Pk = [[0,0,0],[0,0,0],[0,0,0]]
    omega = [[0.4],[0.4],[0.4]]
    dt = 0.1
    Gravity(xk)
###################################################################################################################################################################
#Function: Correct
#Purpose: use readings from the motor to correct the orientation of the sensor, estimated by the gyroscope.
#Inputs: 
    #xk_newpre, a 3x1 numpy array that holds the predicted covariance from the gyroscope
    #Pk_newpre, a 3x3 numpy array that holds the covariances from the prediction step
    #theta_m, a float specifying the encoder-motor's current position, should be in radians
    #Rk, a 3x3 numpy array that is the error matrix for the measurement

#Outputs:
    #xk_new, a 3x1 numpy array that is the new accepted orientation of the sensor
    #Pk_new, a 3x3 numpy array that is the new covariance matrix
def Correct(xk_newpre, Pk_newpre, acc, Rk):
    yk = Gravity(acc)
    ek = yk-xk_newpre
    Sk = Pk_newpre + Rk
    Lk = np.matmul(Pk_newpre,np.linalg.inv(Sk))
    xk_new = xk_newpre + np.matmul(Lk,ek)
##    Pk_new = Pk_newpre + np.matmul(np.matmul(Lk,Sk),np.transpose(Lk))
    #As of Feb 29, 2020, the Pk matrix in the above explodes due to the matrix effectively experiencing a 5th power growth in its determinant, with no added error.
    Pk_new = np.matmul((np.eye(3)-Lk),Pk_newpre)
    return(xk_new,Pk_new)

