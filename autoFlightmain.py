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

#Load arduino board
arduino = serial.Serial('/dev/cu.usbmodem101', 9600)

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
DEFAULT_HEIGHT = .5 #meters
BOX_LIMIT = 0.5
droneCurrentDegree = 0
distanceToSite = 0.5

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
def takeoffAndSearch(mc):
    
    """
    - Go up DEFAULT_HEIGHT meters and wait 2 sec
    - Move forward .5 meters (this is where the aruco should be)
    - Keep turning until aruco is found
    - If turned 360 and still not found, meaning its probably too high or too low
        -get height offset and move accordingly
    - If found marker, move drone so marker is centered
    - If marker centered, go down .2m above ground and release latch
    """
    time.sleep(3)
    mc.forward(distanceToSite)
    time.sleep(2)
    frame = return_frame()
    time.sleep(1)
    #think about failsafes if it doesnt see it first try, like use 10 frames and one of them should have detected it, use that one
    global_corners, tr, br, bl, tl, frameCenter = droneCV2.detect_aruco(frame)
    time.sleep(1)
    #also maybe do if all four corners found, continue, else, move back a bit (since some corners alrdy found, moving back will put the whole thing in frame)
    #If found
    print("Aruco Detected")
    
    #Center the aruco
    horiCentered = False
    vertiCentered = False

    while (not horiCentered) or (not vertiCentered):
       
        frame = return_frame()
        time.sleep(1)
        vertOff, horizOff = droneCV2.get_offset_from_center(tr, br, bl, tl, frameCenter)
        time.sleep(1)
        horiCentered = centerArucoHorizontal(mc, horizOff)
        time.sleep(1)
        vertiCentered = centerArucoVertical(mc, vertOff)
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
        frames = return_multiple_frames(30)
        time.sleep(.5)
        corners, tr, br, bl, tl, fc = detectArucoFromFrames(frames)
        time.sleep(.5)
        tvecAve = droneCV2.poseEstimate(corners)
        time.sleep(.5)
        mc.forward(0.1)

    #Now the tvec is within our threshold, time to drop down
    #Go down to 0.2m above ground
    offsetToGoDown = droneZPos - 0.2
    mc.down(offsetToGoDown)
    time.sleep(1)

'''

#Go up 0.2 meters, go down 0.1 meters, go left 0.1 meters, go right 0.2 meters
d = {"up":0.2, "down":0.1}
for key, value in d.items():
    if key == "up":
        mc.down(value)
'''

def returnToLaunch(mc):
    #will run this after dropping down and releasing latch
    time.sleep(2)
    mc.up(.3) #drone dropped to .2m to drop latch (not fully landed)
    time.sleep(2)
    mc.turn_right(180)
    time.sleep(2)
    mc.forward(.5)
    #Adjust meters as needed
    time.sleep(2)
    mc.stop()


#Had to split them up because well do one at a time while checking the frame in between
def centerArucoHorizontal(mc, horizontalOffset):
    #get values from get_offset_from_center in droneCV
    isCenteredX = False
    #check horiz offsets
    if horizontalOffset < 0:
        #to the left, move right
        mc.right(0.1)
    elif horizontalOffset > 0:
        #to the right, move left
        mc.left(0.1)
    else:
        isCenteredX = True
    return isCenteredX
    
def centerArucoVertical(mc, verticalOffset):
    isCenteredY = False
    #check vert offsets
    if verticalOffset < 0:
        #below center, move up
        mc.up(0.1)
    elif verticalOffset > 0:
        #above center, move down
        mc.down(0.1)
    else:
        isCenteredY = True
    return isCenteredY


def correctHeightOffset(mc):
    if droneZPos > 0.5:
        #higher than marker
        offset = droneZPos - 0.5
        mc.down(offset)
    if droneZPos < 0.5:
        #lower than marker
        offset = 0.5 - droneZPos
        mc.up(offset)
        

def return_multiple_frames(numFrames): #function: droneCV.poseEstimate()
    arr = []
    #store frames in array here
    for i in range(numFrames):
        frame = return_frame()
        time.sleep(0.025)
        arr.append(frame)
        #store frames in array
        
    return arr

def detectArucoFromFrames(frames):
    detectArr = []
    for frame in frames:
        returnVal = droneCV2.detectAruco(frame)
        if not returnVal:
            continue
        else:
            detectArr.append(returnVal)

    return detectArr[len(detectArr)//2] #returns a frame that is sure to have a detected aruco

"""
def getTvecFromFrames(corners):
    #corners = one set of corners, br, tp, tl
    tvecArr = []
    for cornerSet in corners:
        tvecAvg = droneCV2.poseEstimate(cornerSet)
        tvecArr.append(tvecAvg)

    optimalTvec = min(tvecArr)
    return optimalTvec
"""

def return_frame():
    #ONE FRAME
    packetInfoRaw = rx_bytes(4)
    [length, routing, function] = struct.unpack('<HBB', packetInfoRaw)

    imgHeader = rx_bytes(length - 2)
    [magic, width, height, depth, format, size] = struct.unpack('<BHHBBI', imgHeader)

    if magic == 0xBC:
        img_stream = bytearray()
        print("Magic is good")
        print("Resolution is {}x{} with depth of {} byte(s)".format(width, height, depth))
        print("Image format is {}".format(format))
        print("Image size is {} bytes".format(size))
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
        decoded = cv2.imdecode(nparr,cv2.IMREAD_UNCHANGED)
    else:
        print("Frame not received")
        
                
    frame = decoded
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    frame = cv2.flip(frame, 1)
    scale_factor = 1.5
    frame = cv2.resize(frame, None, fx=scale_factor, fy=scale_factor)
    
    return frame

"""
*main difference between mc.forward() and mc.start_forward() etc. is that mc.forward and mc.back wont continue
the code until the distance has been reached
mc.start_...() will not stop until the mc.stop() is given, which is done automatically when the mc instance is exited
"""


if __name__ == '__main__':
    count = 0
    cflib.crtp.init_drivers()

    #reset servo position to 0
    arduino.write(str(0).encode("utf-8"))
    
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
        simple_log_async(scf, lg_stab)
        
        
        #logconf.start()
        
        #DRONE AUTONOMOUS CONTROL
        with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
            takeoffAndSearch(mc)
            #now the drone should be .2m above ground

            #release latch here
            arduino.write(str(180).encode("utf-8"))
            time.sleep(2)
            arduino.write(str(0).encode("utf-8"))
            
            #return to launch here
            returnToLaunch(mc)

"""
        while(1):
            frame = return_frame()
            #Enter the frame in the GUI, enter yaw pitch roll attributes
            count += 1
            if count != 0 and count % imgsArraySize != 0:
                count += 1
                imgsArray.append(frame)
            else:
                #Return imgsArray to aruco marker detector function for processing
                count = 0
                imgsArray = []

            cv2.imshow("***DRONE CAMERA STREAM***", frame)
            cv2.waitKey() #25 gives best resolution
"""


        #logconf.stop()

