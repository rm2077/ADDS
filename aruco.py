import cv2
import matplotlib.pyplot as plt
import time
import glob
import numpy as np
import pickle
import argparse
ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}




def arucoMarkerDetection(tag, cam_mat, dst):
    cap = cv2.VideoCapture(1)
    
    while cap.isOpened():
        
        ret, img = cap.read()
        aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[tag])
        params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, params)

        markerCorners, ids, rejectedCandidates = detector.detectMarkers(img)
        if not markerCorners:
            print("No markers detected...")

        else:
            for corner, id in zip(markerCorners, ids):
                corner = corner.reshape(4,2)
                top_left, top_right, bottom_right, bottom_left = tuple(corner[0]), tuple(corner[1]), tuple(corner[2]), tuple(corner[3])
                top_left = (int(top_left[0]), int(top_left[1]))
                top_right = (int(top_right[0]), int(top_right[1]))
                bottom_left = (int(bottom_left[0]), int(bottom_left[1]))
                bottom_right = (int(bottom_right[0]), int(bottom_right[1]))
                cv2.line(img, top_left, top_right, color=(0, 255, 0))
                cv2.line(img, top_right, bottom_right, color=(0,255,0))
                cv2.line(img, bottom_right, bottom_left, color=(0, 255, 0))
                cv2.line(img, bottom_left, top_left, color=(0,255,0))
                cv2.putText(img, str(int(id)), org=(top_left[0], top_left[1]-10), fontFace=cv2.FONT_HERSHEY_COMPLEX, fontScale=1, color=(255,0,0), thickness=3)
                cv2.circle(img, (int((bottom_right[0] + bottom_left[0]) / 2), int((bottom_left[1] + top_left[1]) / 2)), radius=3, color=(255, 0, 0), thickness=3)
                marker_size = 0.01
                marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, -marker_size / 2, 0],
                              [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)

                ret, rvec, tvec = cv2.solvePnP(marker_points, corner, cam_mat, dst)
                if ret:
                    cv2.drawFrameAxes(img, cam_mat, dst, rvec, tvec, 0.1)

                #rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corner, 0.05, cam_mat, dst)
                #cv2.drawFrameAxes(img, cam_mat, dst, rvec, tvec, 0.1)
            print("Marker #", id, "detected")
        cv2.imshow('Aruco Computer Vision', img)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
        

    cap.release()
    cv2.destroyAllWindows()

    #TOP LEFT, TOP RIGHT, BOTTOM LEFT, BOTTOM RIGHT
    #cv2.aruco.ArucoDetector(img, ARUCO_DICT).detectMarkers()


def takeChessboardImgs():
    cap = cv2.VideoCapture(1)
    if not cap.isOpened():
        print("Error: Camera unable to open..")
        return ''
    count = 0
    while cap.isOpened():
        ret, frame = cap.read()
        print("Image being taken...")
        cv2.waitKey(5000)
        cv2.imshow('frame', frame)
        cv2.imwrite(f"calibration_imgs/img{count+1}.png", frame)
        print("Image ", (count + 1), "has been taken!")
        count += 1
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
        if count == 9:
            break
        
    cap.release()
    cv2.destroyAllWindows()
    
def retrieve_matrices():
    FRAME_SIZE = (1080, 1920)
    CHESSBOARD_SIZE = (7, 7)

    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((1, (CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1]), 3), np.float32)
    objp[0,:,:2] = np.mgrid[0:CHESSBOARD_SIZE[0],0:CHESSBOARD_SIZE[1]].T.reshape(-1,2)
    
    size_of_chessboard_squares_mm = 50
    objp = objp * size_of_chessboard_squares_mm


    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.

    images = glob.glob('calibration_imgs/*.png')

    for image in images:

        img = cv2.imread(image)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        ret, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
        print(ret)
        if ret == True:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,1), criteria)
            imgpoints.append(corners2)

            cv2.drawChessboardCorners(img, CHESSBOARD_SIZE, corners2, ret)
            cv2.imshow('img', img)
            cv2.waitKey(1000)

    ret, cameraMatrix, distortion_coefs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, FRAME_SIZE, None, None)
    print(cameraMatrix, " Shape: ", cameraMatrix.shape)
    print(distortion_coefs, " Shape: ", distortion_coefs.shape)
    with open('matrices/cameraMatrix.pkl', 'wb') as file:
        pickle.dump(cameraMatrix, file)
    with open('matrices/distortionMatrix.pkl', 'wb') as file:
        pickle.dump(distortion_coefs, file)
   


with open("matrices/cameraMatrix.pkl", "rb") as file:
    cMatrix = pickle.load(file)

with open("matrices/distortionMatrix.pkl", "rb") as file:
    dMatrix = pickle.load(file)

arucoMarkerDetection("DICT_6X6_250", cMatrix, dMatrix)
