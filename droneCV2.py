#Testing drone camera with opencv pose estimation and aruco detection

#import serial
import imutils
import argparse
import time
import socket,os,struct, time
import numpy as np
import cv2
import sys
import pickle
from imutils.video import VideoStream

"""COMPUTER VISION ALGORITHM"""

"""
Camera loop will be in autoFlight as it needs to be all in one code
This file will handle all the processes that come with the camera
ONLY FUNCTIONS IN THIS FILE

No Need to print anything in the camera output, but still print the info in the terminal for debugging

REMEMBER:
Two arucos, one for target, one for RTL

*THIS MIGHT HAVE TO BE A CLASS SO WE CAN HAVE ATTRIBUES LIKE self.arucoDetected = False that makes it easier to 
call in the main file


"""


###FUNCTIONS TO MAKE

#Drone cam matrices
with open("cameraMatrix.pkl", "rb") as file:
    cMatrix = pickle.load(file)
with open("distortionMatrix.pkl", "rb") as file:
    dMatrix = pickle.load(file)


ap = argparse.ArgumentParser()
ap.add_argument("-t","--type",type=str,default="DICT_ARUCO_ORIGINAL",help="type of aruco tag to detect")
args = vars(ap.parse_args())

# load the ArUCo dictionary and grab the ArUCo parameters
print("[INFO] detecting '{}' tags...".format(args["type"]))
arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
arucoParams = cv2.aruco.DetectorParameters()
arucoDetector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)

#VARIABLES
frameCenterColor = (0,255,0)
#offset values
downOff = 0
upOff = 0
leftOff = 0
rightOff = 0
horizontalFix = 0
verticalFix = 0
horizontalFixText = ""
verticalFixText = ""
landedText = ""



def detect_aruco(frame):
    #detect aruco and returns the important info abt aruco
    #this is the main function call and the other functions will be subfunctions for this
    (corners, ids, rejected) = arucoDetector.detectMarkers(frame)
    Xframe = frame.shape[1]
    Yframe = frame.shape[0]
    centerRectCoordX1 = (Xframe//2) - 20
    centerRectCoordX2 = (Xframe//2) + 20
    centerRectCoordY1 = (Yframe//2) - 20
    centerRectCoordY2 = (Yframe//2) + 20
    globalCorners = corners
    returnVal = None

    #if ids != None:
    #    ret = cv2.aruco.estimatePoseSingleMarkers(corners, 10)
    #    cv2.aruco.drawDetectedMarkers(frame, corners)

    frameCenter = (Xframe//2,Yframe//2)
    if len(corners) > 0:
        ids = ids.flatten()
        for (markerCorner, markerId) in zip(corners, ids):
            corners = markerCorner.reshape((4,2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))
            arucoCenterX = int((topLeft[0] + bottomRight[0]) / 2.0)
            arucoCenterY = int((topLeft[1] + bottomRight[1]) / 2.0)
    
        returnVal = (globalCorners, topRight, bottomRight, bottomLeft, topLeft, frameCenter)

    return returnVal

    

def arucoFound(globalCorners):
    return len(globalCorners) > 0

def get_offset_from_center(topRight, bottomRight, bottomLeft, topLeft, frameCenter):
    #values that this returns will be given to centerAruco function in autoFlight.py
    # negative verticalOffset means down and negative horizontalOffset means left
    # just get the absolute value but use the pos/neg sign as an indication

    verticalOffset = 0
    horizontalOffset = 0

    trX = topRight[0]
    trY = topRight[1]
    tlX = topLeft[0]
    tlY = topLeft[1]
    brX = bottomRight[0]
    brY = bottomRight[1]
    blX = bottomLeft[0]
    blY = bottomLeft[1]

    #Determine if aruco is in withing center
    if tlX < frameCenter[0] and tlY < frameCenter[1] and trX > frameCenter[0] and trY < frameCenter[1] and blX < frameCenter[0] and blY > frameCenter[1] and brX > frameCenter[0] and brY > frameCenter[1]:
        verticalOffset = 0
        horizontalOffset = 0
        
    #if to the right of center
    if tlX > frameCenter[0] and blX > frameCenter[0]:
        horizontalOffset = blX-frameCenter[0] if blX>tlX else tlX-frameCenter[0]
        #print(f"offset to the right {offsetValue} units...")

    #if to the left of center
    if trX < frameCenter[0] and brX < frameCenter[0]:
        horizontalOffset = -(frameCenter[0]-trX if brX>trX else frameCenter[0]-brX)
        #print(f"offset to the left {offsetValue} units...")

    #if below center
    if trY > frameCenter[1] and tlY > frameCenter[1]:
        verticalOffset = -(trY-frameCenter[1] if trY>tlY else tlY-frameCenter[1])
        #print(f"offset down {offsetValue} units...")

    #if above center
    if brY < frameCenter[1] and blY < frameCenter[1]:
        verticalOffset = frameCenter[1]-blY if brY>blY else frameCenter[1]-brY
        #print(f"offset upwards {offsetValue} units...")


    return verticalOffset, horizontalOffset


def poseEstimate(corners):
    #This should be where it returns the tvec average, to signal latch activation
    marker_size = 0.01
    marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                    [marker_size / 2, marker_size / 2, 0],
                    [marker_size / 2, -marker_size / 2, 0],
                    [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)

    ret, rvec, tvec = cv2.solvePnP(marker_points, corners, cMatrix, dMatrix)

    tvecTotal = 0
    for t in tvec:
        tvecTotal += t
    tvecAve = tvecTotal / len(tvec)
    #return original tvec and average (which will be used for determining distance to aruco)
    return tvecAve


