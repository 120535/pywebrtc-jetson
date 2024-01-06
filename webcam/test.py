import cv2
import numpy
import glob
from natsort import natsorted


fpSamples = "/home/pg/repos/sample/*.png"
fpSamples = "/repos/sample/*.png"

# Get a sorted list of all files in the directory
files = natsorted(glob.glob(fpSamples))

# Iterate through the sorted list of files
for file in files:
    print(file)
    img = cv2.imread(file)
    img = cv2.resize(img, (640, 480))  # Resize the image to 640x480

    print("shape",img.shape)

    cv2.imshow("img", img)
    cv2.waitKey(5000)
    break

