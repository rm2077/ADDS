import logging
import sys
import time
from threading import Event
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
#Load arduino board
#arduino = serial.Serial('/dev/cu.usbmodem101', 9600)
#Drone able to detect aruco marker 1.33 meters away
count = 0
imgsArraySize = 5

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

deck_attached_event = Event()

logging.basicConfig(level=logging.ERROR)

#position_estimate = [0,0,0]
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
tvecThreshold = 0.05
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




#fly up, wait 3 secs, fly back down
def takeoffAndSearch():

    time.sleep(3)
    print("Moving forward 0.5 meters")
    #mc.forward(distanceToSite)
    time.sleep(2)
    frames = return_multiple_frames(10)
    time.sleep(1)
    #think about failsafes if it doesnt see it first try, like use 10 frames and one of them should have detected it, use that one
    global_corners, tr, br, bl, tl, frameCenter  = detectArucoFromFrames(frames)
    time.sleep(1)
    #also maybe do if all four corners found, continue, else, move back a bit (since some corners alrdy found, moving back will put the whole thing in frame)
    #If found
    print("Aruco Detected")
    
    #Center the aruco
    horiCentered = False
    vertiCentered = False

    while (not horiCentered) or (not vertiCentered):
       
        frames = return_multiple_frames(10)
        time.sleep(1)
        global_corners, tr, br, bl, tl, frameCenter = detectArucoFromFrames(frames)
        time.sleep(1)
        vertOff, horizOff = droneCV2.get_offset_from_center(tr, br, bl, tl, frameCenter)
        time.sleep(1)
        horiCentered = centerArucoHorizontal(horizOff)
        time.sleep(1)
        vertiCentered = centerArucoVertical(vertOff)
        time.sleep(1)
        if horiCentered and vertiCentered:
            break
            #now the marker should be centered
    
    time.sleep(1)

    #Get closer until tvec is within threshold
    tvecAve = 100 #starting position
    tvecThreshold = .22 #fake threshold
    while tvecAve > tvecThreshold:
        time.sleep(.5)
        frames = return_multiple_frames(10)
        time.sleep(.5)
        corners, tr, br, bl, tl, fc = detectArucoFromFrames(frames)
        time.sleep(.5)
        tvecAve = droneCV2.poseEstimate(corners)
        time.sleep(.5)
        print("Forward 0.1 meters")
        

    #Now the tvec is within our threshold, time to drop down
    #Go down to 0.2m above ground
    offsetToGoDown = droneZPos - 0.2
    print("Down", str(offsetToGoDown))
    #mc.down(offsetToGoDown)
    time.sleep(1)

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
    if horizontalOffset < 0:
        #to the left, move right
        print("Move right 0.1")
    elif horizontalOffset > 0:
        #to the right, move left
        print("Move left 0.1")
    else:
        print("Centered horizontally")
        isCenteredX = True
    return isCenteredX
    
def centerArucoVertical(verticalOffset):
    isCenteredY = False
    #check vert offsets
    if verticalOffset < 0:
        #below center, move up
        print("move up 0.1")
    elif verticalOffset > 0:
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

"""
*main difference between mc.forward() and mc.start_forward() etc. is that mc.forward and mc.back wont continue
the code until the distance has been reached
mc.start_...() will not stop until the mc.stop() is given, which is done automatically when the mc instance is exited
"""
 

if __name__ == '__main__':
    count = 0
    cflib.crtp.init_drivers()

    #Load arduino board
    #arduino = serial.Serial('/dev/cu.usbmodem11401', 9600)

    #reset servo position to 0
    #arduino.write(str(0).encode("utf-8"))
    
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
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

        
        scf.cf.param.add_update_callback(group='deck', name='bcFlow2',
                                        cb=param_deck_flow)
        time.sleep(1)

        if not deck_attached_event.wait(timeout=5):
            print('No flow deck detected!')
            sys.exit(1)


        with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
            #while loop is for continuous viewing of camera while processing for aruco
            while True:

                #simple_log_async(scf, lg_stab)



                """CONTROL"""

                #starting process (takeoff)
                #this is where move forward will be
                print("starting flight")


                
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
                print("frame returned")
                #print(frame.shape)
                #cv2.imshow('frame', frame)
                #cv2.destroyAllWindows()

                retVal = droneCV2.detect_aruco(frame)
                if retVal == None:
                    print("Aruco not detected")
                else:
                    print("Aruco detected")
                    corners, tr, br, bl, tl, frameCenter = retVal
                    verticalOffset, horizontalOffset = droneCV2.get_offset_from_center(tr, br, bl, tl, frameCenter)
                    print(f"Frame center: {frameCenter}")
                    print(f"Vert: {verticalOffset}, Hori: {horizontalOffset}")
                    if verticalOffset == 0 and horizontalOffset == 0:
                        #centered, move forward till tvec is within threshold
                        #release latch, stop
                        print("Centered 1")
                        #Call Tvec function
                        tvec = droneCV2.poseEstimate(corners)
                        print("Average of tvec: ", tvec)
                        while tvec > tvecThreshold:
                            mc.forward(0.01)
                            time.sleep(1)

                        print("Landing site in correct position")
                        #Releasing latch, activate arduino

                        
                            
                            
                        #mc.stop()



                    else:
                        horiCentered = False
                        vertCentered = False
                        while (not horiCentered) and (not vertCentered):
                            #keeps centering the aruco
                            horiCentered = centerArucoHorizontal(horizontalOffset)
                            vertCentered = centerArucoVertical(verticalOffset)
                            frame = return_frame()
                            #frame = cv2.circle(frame, frameCenter, 3, (0, 255, 0), 1)
                            cv2.imshow("frame", frame)
                            cv2.waitKey(1)
                             
                            retval = droneCV2.detect_aruco(frame)
                            if retval == None:
                                print("Aruco not detected")
                                #maybe move back at this point to refind the aruco
                            else:
                                corners, tr, br, bl, tl, frameCenter = retval
                                verticalOffset, horizontalOffset = droneCV2.get_offset_from_center(tr, br, bl, tl, frameCenter)
                                print(f"Offsets | Vertical: {verticalOffset}, Horizontal: {horizontalOffset}")
                            if horiCentered and vertCentered:
                                print("Centered (2)")
                                mc.stop()
                                break
                            #os.system('clear')





                #testRun(mc)
                
                
                #print important info
                printLogs()





                

                """
                while(1):
                    frame = return_frame()
                    cv2.imshow('frame', frame)
                    cv2.waitKey(0)
                """
                #takeoffAndSearch()
                #returnToLaunch()
                #logconf.start()

                #logconf.stop()

    client_socket.close()
    cv2.destroyAllWindows()