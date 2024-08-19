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
#from droneCV import arucoFound
import numpy as np
import cv2

count = 0
imgsArraySize = 5

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

deck_attached_event = Event()

logging.basicConfig(level=logging.ERROR)

position_estimate = [0,0,0]
DEFAULT_HEIGHT = .5 #meters
BOX_LIMIT = 0.5
droneCurrentDegree = 0

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

    global position_estimate
    position_estimate[0] = data['stateEstimate.x']
    position_estimate[1] = data['stateEstimate.y']
    position_estimate[2] = data['stateEstimate.z']

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
    print('[%d][%s]: %s' % (timestamp, logconf.name, data))

#add logging config to the logging framework of the cf
def simple_log_async(scf, logconf):
    cf = scf.cf
    cf.log.add_config(logconf)
    #logconf.data_received_cb.add_callback(log_stab_callback)
    logconf.start()
    time.sleep(5)
    logconf.stop()


#fly up, wait 3 secs, fly back down
def takeoffAndSearch(scf):
    totalDegrees = 0
    degreeAmount = 3
    """
    - Go up DEFAULT_HEIGHT meters and wait 2 sec
    - Move forward .5 meters (this is where the aruco should be)
    - Keep turning until aruco is found
    - If turned 360 and still not found, meaning its probably too high or too low
        -get height offset and move accordingly
    - If found marker, move drone so marker is centered
    - If marker centered, go down .2m above ground and release latch
    """
    '''with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(2)
        mc.forward(.5)
        time.sleep(3)
        while not arucoFound():
            mc.turn_right(degreeAmount)
            totalDegrees+=degreeAmount
            droneCurrentDegree = totalDegrees
            time.sleep(0.3)
            if totalDegrees >= 360:
                correctHeightOffset(mc) #level drone to be in same height as marker
                continue

        #If found
        print("Aruco Detected")
        
        #Center the aruco

        #Go down to 0.2m above ground
        offsetToGoDown = position_estimate[2] - 0.2
        mc.down(offsetToGoDown)

        #release latch using Arduino servo

        #Returning to Launch and landing drone'''


def returnToLaunch(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(4)
        mc.turn_right(180)
        time.sleep(2)
        mc.forward(.5)
        #Adjust meters as needed
        time.sleep(2)
        mc.stop()


def centerAruco(mc):
    
    #get return values from camWithPose getting the offset of marker tothe center of frame
    #move drone to center it
    pass
def correctHeightOffset(mc):
    if position_estimate[2] > 0.5:
        #higher than marker
        offset = position_estimate[2] - 0.5
        mc.down(offset)
    if position_estimate[2] < 0.5:
        #lower than marker
        offset = 0.5 - position_estimate[2]
        mc.up(offset)
        

def return_frame():

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

    #loggings
    lg_stab = LogConfig(name='Stabilizer', period_in_ms=10)
    lg_stab.add_variable('stabilizer.roll', 'float')
    lg_stab.add_variable('stabilizer.pitch', 'float')
    lg_stab.add_variable('stabilizer.yaw', 'float')

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        
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
        
        while(1):
            frame = return_frame()
            count += 1
            if count != 0 and count % imgsArraySize != 0:
                count += 1
                imgsArray.append(frame)
            else:
                #Return imgsArray to aruco marker detector function for processing
                count = 0
                imgsArray = []
                

                
            
                
            cv2.imshow("***DRONE CAMERA STREAM***", frame)
            cv2.waitKey(25)
        
        ## DRONE COMMANDS
        #



        #logconf.stop()

