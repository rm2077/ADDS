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
import threading
import queue
from threading import Event


#setup the queue for thread-safe communication
frame_queue = queue.Queue()


flag = 0


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

def centerArucoHorizontal(horizontalOffset):
    global flag
    #get values from get_offset_from_center in droneCV
    isCenteredX = False
    #check horiz offsets
    if horizontalOffset < 0:
        #to the left, move right
        print("Move right 0.1")
        flag = 2
    elif horizontalOffset > 0:
        #to the right, move left
        print("Move left 0.1")
        flag = 1
    else:
        print("Centered horizontally")
        isCenteredX = True
    return isCenteredX
    
def centerArucoVertical(verticalOffset):
    global flag
    isCenteredY = False
    #check vert offsets
    if verticalOffset < 0:
        #below center, move up
        print("move up 0.1")
        flag = 3
    elif verticalOffset > 0:
        #above center, move down
        print("move down 0.1 meters")
        flag = 4
        #mc.down(0.1)
    else:
        print("Centered vertically") 
        isCenteredY = True
    return isCenteredY

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









"""MAIN FUNCTIONS"""
 



def detectionSystem(frame_queue):
    global flag
    tvecThreshold = 0.0025
    while True:
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
        
        #add frame to queue to be printed
        frame_queue.put(frame)


        retVal = droneCV2.detect_aruco(frame)
        if retVal == None:
            print("Aruco not detected")
        else:
            print("Aruco detected")
            corners, tr, br, bl, tl, frameCenter = retVal
            verticalOffset, horizontalOffset = droneCV2.get_offset_from_center(tr, br, bl, tl, frameCenter)
            if verticalOffset == 0 and horizontalOffset == 0:
                #centered, move forward till tvec is within threshold
                #release latch, stop 

                #add frame to queue to be printed
                print("Aruco Marker Centered")
                frame_queue.put(frame)

                newCorners = corners[0]
                
                tvec = droneCV2.poseEstimate(newCorners)
                print("Tvec Average: ", tvec)
                while tvec > tvecThreshold:
                    tvec = droneCV2.poseEstimate(newCorners)
                    print("Tvec Average: ", tvec)

                
                print("Releasing latch")
                
                #Wait 5 seconds and Add Return To Launch Function
                
            else:
                #not centered
                horiCentered = False
                vertCentered = False
                while (not horiCentered) and (not vertCentered):
                    #keeps centering the aruco
                    horiCentered = centerArucoHorizontal(horizontalOffset)
                    vertCentered = centerArucoVertical(verticalOffset)
                    frame = return_frame()
                    #frame = cv2.circle(frame, frameCenter, 3, (0, 255, 0), 1)
                    frame_queue.put(frame)
                    #cv2.imshow("frame", frame)
                    #cv2.waitKey(1)
                        
                    retval = droneCV2.detect_aruco(frame)
                    if retval == None:
                        print("Aruco not detected")
                        #maybe move back at this point to refind the aruco
                    else:
                        corners, tr, br, bl, tl, frameCenter = retval
                        verticalOffset, horizontalOffset = droneCV2.get_offset_from_center(tr, br, bl, tl, frameCenter)
                        print(f"Offsets | Vertical: {verticalOffset}, Horizontal: {horizontalOffset}")
                    
                    #os.system('clear')

    
    
        




def flightSystem():

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

    cflib.crtp.init_drivers()

    global flag

    URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
    deck_attached_event = Event()
    logging.basicConfig(level=logging.ERROR)


    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        if not deck_attached_event.wait(timeout=5):
            print('No flow deck detected!')
            sys.exit(1)
        
        with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
            while True:
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








'''

Takeoff to aruco = 0
Move left = 1
Move right = 2
Move up = 3
Move down = 4
Move forward = 5
Move back = 6

'''
if __name__ == '__main__':
    camera_thread = threading.Thread(target=detectionSystem)
    drone_thread = threading.Thread(target=flightSystem)

    camera_thread.start()
    drone_thread.start()


    while True:
        if not frame_queue.empty():
            frame = frame_queue.get()
            cv2.imshow("Output", frame)
        if cv2.waitKey(1) == 27:
            break

    camera_thread.join()
    drone_thread.join()

    cv2.destroyAllWindows()
    
    







