#short python script to display ppm files on windows.
#taken from https://stackoverflow.com/questions/4101576/importing-ppm-images-with-python-and-pil-module.
import time
from PIL import Image

# im = Image.open("C:\\Users\\suprm\\Documents\College Work\\CMSC 435\\Proj1\\mschuck1\\proj1\\hide.ppm")
# #im2 = Image.open("C:\\Users\\suprm\\Documents\College Work\\CMSC 435\\Proj1\\mschuck1\\proj1\\output.ppm")

# im.show()
#im2.show()


import cv2

#im = Image.open("C:\\Users\\suprm\\source\\repos\\RayTracer/hide.ppm")
#im.show()
#im.close()

img = cv2.imread("C:\\Users\\suprm\\Documents\College Work\\CMSC 435\\Proj1\\mschuck1\\proj2\\hide.ppm")
#height, width, layers = img.shape
#print(height, width, layers)
cv2.imshow('image', img)
cv2.waitKey()