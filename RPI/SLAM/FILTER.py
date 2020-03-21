#FILTER Library
#Project Sentinel
#Created by: Nicholas O'Brien
from scipy import ndimage
import numpy as np
import matplotlib.pyplot as plt

#Function: ButterworthAcceleration
#Goal: Take accelerations from the scans of a test and return their filtered versions in an attempt to give smoother curves and thus more accurate orientation predictions.

#Inputs:
    #scans, a list of dictionaries, each dictionary is a scan from the lidar

#Outputs:
    #updatedscans, a list of dictionaries, now with their acceleration values updated from the butterworth filter.
def ButterworthAcceleration(scans,c=3):
    Ax = []
    Ay = []
    Az = []
    for scan in scans:
        Ax.append([float(scan['Ax'])])
        Ay.append([float(scan['Ay'])])
        Az.append([float(scan['Az'])])
    Ax = np.asarray(Ax)
    Ay = np.asarray(Ay)
    Az = np.asarray(Az)
##    fs = 10 #scans per revolution, the sampling frequency
####    fc = 5 #cutoff frequency
##    w = fc/ (fs/2)
##    b, a = signal.butter(5, w, 'low')
    Ayf = ndimage.median_filter(Ay,c, mode='reflect')
    Azf = ndimage.median_filter(Az,c, mode='reflect')
##    Ayf = signal.filtfilt(b,a,Ay,padlen=0)
##    Azf = signal.filtfilt(b,a,Az,padlen=0)
    indices = np.arange(0,len(Ax),1)
    Axf = 0.5*np.cos(indices*np.pi/10-7*np.pi/6)
    
    plt.plot(indices,Axf, indices, Ax)
