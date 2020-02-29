#Created by: Nicholas O'Brien
#Project Sentinel; Kalman Filter Library
from RANSAC.RANSAC import calculateQ
import numpy as np
import scipy.linalg as slp

###################################################################################################################################################################
#Function: KALMAN
#Purpose: use readings from the gyroscope and the mechanism encoder to estimate the current orientation of the sensor.
#Inputs: 
    #xk_1, a 3x1 numpy array that holds the last accepted orientation of the sensor
    #Pk_1, a 3x3 numpy array that holds the covariances for the last accepted orientation
    #theta_m, a float specifying the encoder-motor's current position, should be in radians
    #omega, a 3x1 numpy array that is the format [[wx], [wy],[wz]] from the gyroscope
    #dt, the time lapse between this calculation and the time at which xk_1 was last calculated.

#Outputs:
    #xk_new, a 3x1 numpy array that is the new accepted orientation of the sensor
    #Pk_new, a 3x3 numpy array that is the new covariance matrix

#NOTES:
    #The orientation of the gyroscope's axes is the following: {x' is aligned with -x_lidar, y' is aligned with y_lidar, z' is aligned with -z_lidar}
def KALMAN(xk_1, Pk_1, theta_m, omega, dt, Qk, Rk):
    yk = [[calculateQ(theta_m,np.pi/2)],[calculateQ(theta_m,0)],[0]]

    phi =xk_1[0][0]
    theta = xk_1[1][0]
    Dk_1 = [[1, np.sin(phi)*np.tan(theta), np.cos(phi)*np.tan(theta)],
            [0, np.cos(phi), -np.sin(phi)],
            [0, np.sin(phi)/np.cos(theta), np.cos(phi)/np.cos(theta)]]

    Phik = np.diag([np.exp(dt),np.exp(dt),np.exp(dt)])
##    Phik = slp.expm(np.multiply(dt,Dk_1))
    
    xk_newpre = np.matmul(Dk_1,omega)*dt+xk_1
    Pk_newpre = np.matmul(np.matmul(Phik,Pk_1),np.linalg.inv(Phik))+Qk

    ek = yk-xk_newpre
    Sk = Pk_newpre + Rk
    Lk = np.matmul(Pk_newpre,np.linalg.inv(Sk))
    xk_new = xk_newpre + np.matmul(Lk,ek)
##    Pk_new = Pk_newpre + np.matmul(np.matmul(Lk,Sk),np.transpose(Lk))
    Pk_new = np.matmul((np.eye(3)-Lk),Pk_newpre)
    #As of Feb 29, 2020, the Pk matrix explodes due to the matrix effectively experiencing a 5th power growth in its determinant, with no added error.
    return(xk_new,Pk_new)
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
    phi = np.arctan(-ay/az)
    theta = np.arctan(ax*np.sin(phi)/ay)
    psi = 0
    xk_estimate = np.asarray([[phi],[theta],[psi]])
    return(xk_estimate)

if __name__=="__main__":
    xk = [[0],[0],[0]]
    Pk = [[0,0,0],[0,0,0],[0,0,0]]
    omega = [[0.4],[0.4],[0.4]]
    dt = 0.1
    for angle in range(0,10):
        theta_m = angle*0.1
        (xk, Pk) = KALMAN(xk, Pk, theta_m, omega, dt)
        print(xk)
        print(Pk)
