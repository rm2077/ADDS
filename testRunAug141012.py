
import logging
import sys
import time
from threading import Event
import threading
import socket
import struct
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper
import argparse
import droneCV2
import numpy as np
import cv2
import serial
import os
import gui 

count = 0
imgsArraySize = 5

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

deck_attached_event = Event()

logging.basicConfig(level=logging.ERROR)

#position_estimate = [0,0,0]
global thread_frame
global thread_data
droneXPos = 0
droneYPos = 0
droneZPos = 0
droneRoll = 0
dronePitch = 0
droneYaw = 0
DEFAULT_HEIGHT = 0.5 #meters
BOX_LIMIT = 0.5
droneCurrentDegree = 0
distanceToSite = 0.5
tvecThreshold = 0.0025
parser = argparse.ArgumentParser(description='Connect to AI-deck JPEG streamer example')
parser.add_argument("-n",  default="192.168.4.1", metavar="ip", help="AI-deck IP")
parser.add_argument("-p", type=int, default='5000', metavar="port", help="AI-deck port")
parser.add_argument('--save', action='store_true', help="Save streamed images")
args = parser.parse_args()

deck_port = args.p
deck_ip = args.n

print("Connecting to socket on {}:{}...".format(deck_ip, deck_port))
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((deck_ip, deck_port))
print("Socket connected")

imgdata = None
data_buffer = bytearray()

"""THREAD VARIABLES  (shared by both threads, thread_data has to be a list)"""
thread_frame = None
thread_data = None


def rx_bytes(size):
    data = bytearray()
    while len(data) < size:
        packet_chunk = client_socket.recv(size-len(data))
        if not packet_chunk:
            raise ConnectionError("Socket connection lost")
        data.extend(packet_chunk)
    return data

def log_pos_callback(timestamp, data, logconf):

    global droneXPos, droneYPos, droneZPos
    droneXPos = data['stateEstimate.x']
    droneYPos = data['stateEstimate.y']
    droneZPos = data['stateEstimate.z']

def param_deck_flow(_, value_str):
    value = int(value_str)
    print(value)
    if value:
        deck_attached_event.set()
        print('Deck is attached!')
    else:
        print('Deck is NOT attached!')


#Asynchronous logging

def log_stab_callback(timestamp, data, logconf):
    #print('[%d][%s]: %s' % (timestamp, logconf.name, data))
    global droneRoll, dronePitch, droneYaw
    droneRoll = data['stabilizer.roll']
    dronePitch = data['stabilizer.pitch']
    droneYaw = data['stabilizer.yaw']

#add logging config to the logging framework of the cf
def simple_log_async(scf, logconf):
    cf = scf.cf
    cf.log.add_config(logconf)
    #logconf.data_received_cb.add_callback(log_stab_callback)
    logconf.start()
    time.sleep(5)
    logconf.stop()




'''
#RTL concept
#Go up 0.2 meters, go down 0.1 meters, go left 0.1 meters, go right 0.2 meters
d = {"up":0.2, "down":0.1}
for key, value in d.items():
    if key == "up":
        mc.down(value)
'''

def returnToLaunch():
    #will run this after dropping down and releasing latch
    time.sleep(2)
    print("Up 0.3 meters")
    #mc.up(.3) #drone dropped to .2m to drop latch (not fully landed)
    time.sleep(2)
    print("Turn right 180 degrees")
    #mc.turn_right(180)
    time.sleep(2)
    print("Forward 0.5 meters")
    #mc.forward(.5)
    #Adjust meters as needed
    time.sleep(2)
    print("Stopped")
    #mc.stop()


#Had to split them up because well do one at a time while checking the frame in between
def centerArucoHorizontal(horizontalOffset):
    #get values from get_offset_from_center in droneCV
    isCenteredX = False
    #check horiz offsets
    if horizontalOffset > 0:
        #to the left, move right
        print("Move right 0.1")
    elif horizontalOffset < 0:
        #to the right, move left
        print("Move left 0.1")
    else:
        print("Centered horizontally")
        isCenteredX = True
    return isCenteredX
    
def centerArucoVertical(verticalOffset):
    isCenteredY = False
    #check vert offsets
    if verticalOffset > 0:
        #below center, move up
        print("move up 0.1")
    elif verticalOffset < 0:
        #above center, move down
        print("move down 0.1 meters")
        #mc.down(0.1)
    else:
        print("Centered vertically")
        isCenteredY = True
    return isCenteredY


def correctHeightOffset():
    if droneZPos > 0.5:
        #higher than marker
        offset = droneZPos - 0.5
        print("Move down", str(offset))
    if droneZPos < 0.5:
        #lower than marker
        offset = 0.5 - droneZPos
        print("Move up", str(offset))
        #mc.up(offset)
        
def markerTooClose(frame, mc, tr, br, bl, tl):
    #droneZPos
    #this is wrong, tr has two values
    frameHeight = frame.shape[0]
    if tr < 20 or tl < 20 or br > frameHeight - 20 or bl > frameHeight - 20:
        #if any of the corners are reaching the edge of frame, its too close and move back
        mc.back(0.1)


def printLogs():
    #print important drone information
    print(f"Drone X Position: {droneXPos}\nDrone Y Position: {droneYPos}\nDrone Z Position: {droneZPos}\nDrone Roll: {droneRoll}\nDrone Pitch: {dronePitch}\nDrone Yaw: {droneYaw}\n")

def return_frame():
    #ONE FRAME
    packetInfoRaw = rx_bytes(4)
    [length, routing, function] = struct.unpack('<HBB', packetInfoRaw)
    imgHeader = rx_bytes(length - 2)
    [magic, width, height, depth, format, size] = struct.unpack('<BHHBBI', imgHeader)
    if magic == 0xBC:
        img_stream = bytearray()
        #print("Magic is good")
        #print("Resolution is {}x{} with depth of {} byte(s)".format(width, height, depth))
        #print("Image format is {}".format(format))
        #print("Image size is {} bytes".format(size))
        while len(img_stream) < size:
            packet_info_raw = rx_bytes(4)
            length, dst, src = struct.unpack('<HBB', packet_info_raw)
            chunk = rx_bytes(length - 2)
            img_stream.extend(chunk)
                    # mean_time_per_image = (time.time() - start) / count
                    # print("Mean time per image: {:.2f}s".format(mean_time_per_image))

    if format == 0:
                        # Assuming this is a raw Bayer image
        bayer_img = np.frombuffer(img_stream, dtype=np.uint8)
        bayer_img.shape = (244, 324)  # Adjust this if needed
        color_img = cv2.cvtColor(bayer_img, cv2.COLOR_BayerBG2BGRA)
        cv2.waitKey(1)
    elif format == 1:
                        
        nparr = np.frombuffer(img_stream, np.uint8)
        decoded = cv2.cvtColor(nparr, cv2.COLOR_BayerBG2BGRA)
        decoded = cv2.imdecode(nparr,cv2.IMREAD_UNCHANGED)
        #decoded = cv2.cvtColor(decoded, cv2.COLOR_BGR2RGB) #this might not be necessary
        
    else:
        print("Frame not received")
        
    frame = decoded

    return frame

def return_drone_data(x, y, z, roll, pitch, yaw):
    data = [x, y, z, roll, pitch, yaw]
    return data





def mainCode():
    count = 0
    cflib.crtp.init_drivers()


    #main_app = gui.MainApp(frame=None, data=None)
    #main_app.mainloop()

    #Load arduino board
    #arduino = serial.Serial('/dev/cu.usbmodem1301', 9600)

    #reset servo position to 0
    #arduino.write(str(180).encode("utf-8"))
    
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:

        
        scf.cf.param.add_update_callback(group='deck', name='bcFlow2',
                                        cb=param_deck_flow)
        time.sleep(1)

        if not deck_attached_event.wait(timeout=5):
            print('No flow deck detected!')
            sys.exit(1)

        #loggings
        lg_stab = LogConfig(name='Stabilizer', period_in_ms=10)
        lg_stab.add_variable('stabilizer.roll', 'float')
        lg_stab.add_variable('stabilizer.pitch', 'float')
        lg_stab.add_variable('stabilizer.yaw', 'float')
        scf.cf.log.add_config(lg_stab)
        lg_stab.data_received_cb.add_callback(log_stab_callback)
        #logging while flying
        logconf = LogConfig(name='Position', period_in_ms=10)
        logconf.add_variable('stateEstimate.x', 'float')
        logconf.add_variable('stateEstimate.y', 'float')
        logconf.add_variable('stateEstimate.z', 'float')
        scf.cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(log_pos_callback)
        #simple_log_async(scf, lg_stab)

        with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
            #while loop is for continuous viewing of camera while processing for aruco
            print("starting flight")
            takeoffToAruco = True
            returnToLaunchSite = False
            logconf.start()
            lg_stab.start()
            while True:
                
                if takeoffToAruco:
                    #mc.forward(1.2)
                    #time.sleep(1)
                    print("Taking off to marker")
                    takeoffToAruco = False

                if returnToLaunchSite:
                    print("Landing and release latch.")
                    #will be made True after released latch
                    #returnToLaunch()
                    #arduino.write(str(0).encode("utf-8"))
                    #time.sleep(2)
                    #arduino.write(str(180).encode("utf-8"))
                    #time.sleep(2)
                    returnToLaunchSite = False
                    break

                """CONTROL"""

                #starting process (takeoff)
                #this is where move forward will be
                
                packetInfoRaw = rx_bytes(4)
                [length, routing, function] = struct.unpack('<HBB', packetInfoRaw)
                imgHeader = rx_bytes(length - 2)
                [magic, width, height, depth, format, size] = struct.unpack('<BHHBBI', imgHeader)
                if magic == 0xBC:
                    img_stream = bytearray()
                    #print("Magic is good")
                    #print("Resolution is {}x{} with depth of {} byte(s)".format(width, height, depth))
                    #print("Image format is {}".format(format))
                    #print("Image size is {} bytes".format(size))
                    while len(img_stream) < size:
                        packet_info_raw = rx_bytes(4)
                        length, dst, src = struct.unpack('<HBB', packet_info_raw)
                        chunk = rx_bytes(length - 2)
                        img_stream.extend(chunk)
                                # mean_time_per_image = (time.time() - start) / count
                                # print("Mean time per image: {:.2f}s".format(mean_time_per_image))

                if format == 0:
                                    # Assuming this is a raw Bayer image
                    bayer_img = np.frombuffer(img_stream, dtype=np.uint8)
                    bayer_img.shape = (244, 324)  # Adjust this if needed
                    color_img = cv2.cvtColor(bayer_img, cv2.COLOR_BayerBG2BGRA)
                    cv2.waitKey(1)
                elif format == 1:
                                    
                    nparr = np.frombuffer(img_stream, np.uint8)
                    decoded = cv2.cvtColor(nparr, cv2.COLOR_BayerBG2BGRA)
                    decoded = cv2.imdecode(nparr,cv2.IMREAD_UNCHANGED)
                    print("Returned frame")
                    #decoded = cv2.cvtColor(decoded, cv2.COLOR_BGR2RGB) #this might not be necessary
                    
                else:
                    print("Frame not received")
                    
                frame = decoded
                thread_frame = frame
                thread_data = [droneXPos, droneYPos, droneZPos, droneRoll, dronePitch, droneYaw]
                print("frame returned")

                #print(frame.shape)
                #cv2.imshow('frame', frame)
                #cv2.destroyAllWindows()

                retVal = droneCV2.detect_aruco(frame)
                if retVal == None:
                    print("Aruco not detected")
                    mc.forward(0.01)
                else:
                    print("Aruco detected")
                    corners, tr, br, bl, tl, frameCenter = retVal
                    verticalOffset, horizontalOffset = droneCV2.get_offset_from_center(tr, br, bl, tl, frameCenter)
                    print(f"Frame center: {frameCenter}")
                    print(f"Vert: {verticalOffset}, Hori: {horizontalOffset}")
                    if verticalOffset == 0 and horizontalOffset == 0:
                        #centered, move forward till tvec is within threshold
                        #release latch, stop
                    
                        print("Centered (2)")
                        #cv2.imshow('Centered Frame', frame)
                        #cv2.waitKey(1)
                        thread_frame = frame
                        thread_data = [droneXPos, droneYPos, droneZPos, droneRoll, dronePitch, droneYaw]
                        newCorners = corners[0]
                        
                        tvec = droneCV2.poseEstimate(newCorners)
                        print("Tvec Average: ", tvec)
                        cv2.destroyWindow('Centered Frame')
                        while (tvec > tvecThreshold):
                            frame = return_frame()
                            #cv2.imshow('tvecTracking', frame)
                            #cv2.waitKey(1)
                            thread_frame = frame
                            thread_data = [droneXPos, droneYPos, droneZPos, droneRoll, dronePitch, droneYaw]
                            retVal = droneCV2.detect_aruco(frame)
                            if retVal is not None:
                                corners, _, _, _, _, _, = retVal
                                if corners is not None:
                                    newCorners = corners[0]
                                    tvec = droneCV2.poseEstimate(newCorners)
                                    print("Tvec Average: ", tvec)
                                    #mc.forward(0.01)

                                     
                        cv2.destroyWindow('tvecTracking')
                        returnToLaunchSite = True
                        print("Releasing latch")
                        #return 
                        #Return to launch function
                        mc.stop()

                    else:
                        horiCentered = False
                        vertCentered = False
                        cv2.destroyAllWindows()
                        while (not horiCentered) and (not vertCentered):
                            
                            #keeps centering the aruco
                            horiCentered = centerArucoHorizontal(horizontalOffset)
                            vertCentered = centerArucoVertical(verticalOffset)
                            frame = return_frame()
                            #frame = cv2.circle(frame, frameCenter, 3, (0, 255, 0), 1)
                            #cv2.imshow("frame", frame)
                            #cv2.waitKey(1)
                            thread_frame = frame
                            thread_data = [droneXPos, droneYPos, droneZPos, droneRoll, dronePitch, droneYaw]
                            
                            
                            retval = droneCV2.detect_aruco(frame)
                            if retval == None:
                                print("Aruco not detected")
                                #maybe move back at this point to refind the aruco
                                mc.forward(0.01)
                            else:
                                corners, tr, br, bl, tl, frameCenter = retval
                                verticalOffset, horizontalOffset = droneCV2.get_offset_from_center(tr, br, bl, tl, frameCenter)
                                print(f"Offsets | Vertical: {verticalOffset}, Horizontal: {horizontalOffset}")
                            
                            #os.system('clear')

                #testRun(mc)
                
                
                #print important info
                printLogs()

        logconf.stop()
        lg_stab.stop()


    client_socket.close()
    cv2.destroyAllWindows()

def guiCode():
    
    main_app = gui.MainApp(thread_frame, thread_data)
    main_app.mainloop()
    



if __name__ == "__main__":
    t1 = threading.Thread(target = mainCode)
    t2 = threading.Thread(target = guiCode)
    t1.start()
    t2.start()
    t1.join()
    t2.join()