import numpy as np
import time 
import cv2

start = time.time()
cam_port = 0 
cap = cv2.VideoCapture(cam_port)
cap.read()

img_name = "test.png"
ret, img = cap.read()
data = np.reshape(img, (-1, 3))
data = data[153590:153610][:]

data = np.float32(data)
print(np.shape(data))
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
flags = cv2.KMEANS_RANDOM_CENTERS
compactness, labels, centers = cv2.kmeans(data, 1, None, criteria, 10, flags)
print("Dominant color: ",centers[0].astype(np.int32))
# cv2.imwrite(img_name, img)
print(time.time() - start)
cap.release()
