#Created by: Nicholas O'Brien
#Project Sentinel; main script
#Created: December 7th, 2019
#Last Edit: December 8th, 2019
import sys
import ast
sys.path.append("..\PARSER")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
import RANSAC.RANSAC as RANSAC
from numpy import array
##import PARSER
import EKF.EKF as EKF
import numpy as np
import KALMAN
import quick
import FILTER
##import ANGLEPARSE
X = 10 #mm, the maximum distance a point must be from an LSRP in RANSAC to be considered in tolerance.
C = 800 #Consensus for RANSAC, number of points that must pass the tolerance check of the LSRP
N = 100 #Max number of trials in RANSAC before ending
S = 50 #Number of points to sample for RANSAC
S_LIM = 100 #mm, half the length of a side of the cube to draw around the randomly sampled point in RANSAC

DATABASE = "room_contbutnotQk8_1"

res = quick.readFromAWS(DATABASE)
LSRP_list = []

x = [[0.0], [0.0], [0.0], [6.0], [3.0], [1.0]]
dx_sum = np.zeros((3,1))
P = np.zeros((6,6))
Landmark_Positions = {1:3}
dt1 = 0.0
dt2 = 0.0 #In reality, these would be grabbed from the Arduino.
##
##frame = []
##for i in range(0, len(res), 1):
##    frame.append(res[i])
##    
error = []
errorphi = []
errortheta = []
errorpsi = []
A_psi = []
A_phi = []
A_theta = []
Ax = []
Ay = []
Az = []
x_phi = []
x_theta = []
x_psi = []
Pnorm = []
time = []
Qk = eval(res[0]['Qk'])
Rk = eval(res[0]['Rk'])
start_time = float(res[0]['Time of transmission'])
for i in range(0,len(res),1):
    x = eval(res[i]['euler'])
    t = float(res[i]['Time of transmission'])-start_time
    time.append(t)
    x_phi.append(x[0][0])
    x_theta.append(x[1][0])
    x_psi.append(x[2][0])
    P = eval(res[i]['P'])
    acc = [[float(res[i]['Ax'])],[float(res[i]['Ay'])],[float(res[i]['Az'])]]
    A = KALMAN.Gravity(acc)
##    print(A,acc)
    Ax.append(acc[0][0])
    Ay.append(acc[1][0])
    Az.append(acc[2][0])
    A_psi.append(A[2])
    A_phi.append(A[0])
    A_theta.append(A[1])
    errorvec = x-A
    errorphi.append(errorvec[0])
    errortheta.append(errorvec[1])
    errorpsi.append(errorvec[2])
    error.append(np.linalg.norm(errorvec))
    Pnorm.append(np.linalg.norm(P))
##    errorphi.append(x[0])
##    errortheta.append(x[1])
##    errorpsi.append(x[2])
##    error.append(np.linalg.norm(x))
##    Pnorm.append(np.linalg.norm(P))
##    print(P)
##    print(dt)

plt.figure()
plt.plot(time,error, time, errorphi, time, errortheta, time, errorpsi, time, Pnorm)
plt.legend(("norm", "phi", "theta", "psi", "P_Norm"))
plt.title(DATABASE+" Orientation Error vs. Time")
plt.xlabel("Time since first scan (s)")
plt.ylabel("Angle [radians]")
plt.ylim(-1,1.5)
plt.savefig("..\\..\\..\\"+DATABASE+"Error.png", quality=100)

plt.figure()
plt.plot(time,A_phi, time, x_phi, time, A_theta, time, x_theta)
plt.legend(("A_phi", "euler_phi", "A_theta", "euler_theta"))
plt.title(DATABASE+" Orientations from Accelerometer Vs. Kalman Filter")
plt.xlabel("Time since First scan (s)")
plt.ylabel("Angle [radians]")
plt.ylim(-1,1)
plt.savefig("..\\..\\..\\"+DATABASE+"Orientations.png", quality=100)

plt.show()
