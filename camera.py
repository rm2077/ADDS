import cv2
import cv2 as cv
import mediapipe as mp
import numpy as np
import argparse
import socket
import struct
import time
import copy
import math

######################################################################## DRONE
import logging
import sys
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper
from cflib.utils.multiranger import Multiranger
URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

if len(sys.argv) > 1:
    URI = sys.argv[1]

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)
######################################################################## DRONE
parser = argparse.ArgumentParser(description='Connect to AI-deck JPEG streamer example')
parser.add_argument("-n", default="192.168.4.1", metavar="ip", help="AI-deck IP")
parser.add_argument("-p", type=int, default=5000, metavar="port", help="AI-deck port")
parser.add_argument('--save', action='store_true', help="Save streamed images")
args = parser.parse_args()

deck_ip = args.n
deck_port = args.p

print("Connecting to socket on {}:{}".format(deck_ip, deck_port))
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


def main():

    readings_avg = []
    readings_count = 0
    shoulder_count = 0
    
    cflib.crtp.init_drivers()

    global count
    mp_drawing = mp.solutions.drawing_utils
    mp_drawing_styles = mp.solutions.drawing_styles
    mp_pose = mp.solutions.pose

    # Initialize mediapipe pose
    pose = mp_pose.Pose(
        static_image_mode=True,
        model_complexity=2,
        enable_segmentation=True,
        min_detection_confidence=0.5)

    cf = Crazyflie(rw_cache='./cache')
    with SyncCrazyflie(URI, cf=cf) as scf:
        with MotionCommander(scf, default_height = 0.8) as motion_commander:
            with Multiranger(scf) as multiranger:
                keep_flying = True

                while keep_flying:
                    
                    VELOCITY = 0.5
                    velocity_x = 0.0
                    velocity_y = 0.0

                    if is_close(multiranger.front):
                        velocity_x -= VELOCITY
                        # print("Doing action F")
                    if is_close(multiranger.back):
                        velocity_x += VELOCITY
                        # print("Doing action B")
                    if is_close(multiranger.left):
                        velocity_y -= VELOCITY
                        # print("Doing action L")
                    if is_close(multiranger.right):
                        velocity_y += VELOCITY
                        # print("Doing action R")

                    if is_close(multiranger.up):
                        keep_flying = False
                    # time.sleep(0.1)
                    motion_commander.start_linear_motion(
                        velocity_x, velocity_y, 0)


                    # Get the image packet from the AI-deck
                    packetInfoRaw = rx_bytes(4)
                    [length, routing, function] = struct.unpack('<HBB', packetInfoRaw)

                    imgHeader = rx_bytes(length - 2)
                    [magic, width, height, depth, format, size] = struct.unpack('<BHHBBI', imgHeader)

                    if magic == 0xBC:
                        img_stream = bytearray()
                        while len(img_stream) < size:
                            packet_info_raw = rx_bytes(4)
                            length, dst, src = struct.unpack('<HBB', packet_info_raw)
                            chunk = rx_bytes(length - 2)
                            img_stream.extend(chunk)

                        count = count + 1
                        # mean_time_per_image = (time.time() - start) / count
                        # print("Mean time per image: {:.2f}s".format(mean_time_per_image))

                        if format == 0:
                            # Assuming this is a raw Bayer image
                            bayer_img = np.frombuffer(img_stream, dtype=np.uint8)
                            bayer_img.shape = (244, 324)  # Adjust this if needed
                            color_img = cv2.cvtColor(bayer_img, cv2.COLOR_BayerBG2BGRA)
                            cv2.imshow('AI-Deck Stream - Raw', bayer_img)
                            cv2.imshow('AI-Deck Stream - Color', color_img)
                            cv2.waitKey(1)
                        
                            nparr = np.frombuffer(img_stream, np.uint8)
                            decoded = cv2.imdecode(nparr,cv2.IMREAD_UNCHANGED)
                    
                    image = decoded
                    image = cv.cvtColor(image, cv.COLOR_BGR2RGB)
                    image = cv.flip(image, 1)
                    scale_factor = 1.5
                    image = cv.resize(image, None, fx=scale_factor, fy=scale_factor)
                    