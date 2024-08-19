import serial
import time
PORT = "/dev/cu"

def release_latch(angle):

    arduino = serial.Serial(PORT, 115200)
    command = str(angle)
    arduino.write(command.encode())
    reachedPos = str(arduino.readline())
    print("Reached position: ", reachedPos)
    

release_latch(45)
