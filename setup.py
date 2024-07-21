import collections
import collections.abc
import dronekit_sitl
collections.MutableMapping = collections.abc.MutableMapping
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import socket
import math


sitl = dronekit_sitl.start_default()
connection_string = sitl.connection_string()
print("Connecting to vehicle on: ", connection_string)
vehicle = connect(connection_string, wait_ready=True)
print("Type: ", vehicle._vehicle_type)
vehicle.close()