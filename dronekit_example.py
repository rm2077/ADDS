import collections
import collections.abc
import dronekit_sitl
collections.MutableMapping = collections.abc.MutableMapping
from dronekit import connect, LocationGlobalRelative, LocationGlobal, VehicleMode
import time
sitl = dronekit_sitl.start_default()
connection_string = sitl.connection_string()
print(connection_string)
vehicle = connect(connection_string, wait_ready=True)

print("Get some vehicle attribute values:")
print(" GPS: %s" % vehicle.gps_0)
print(" Battery: %s" % vehicle.battery)
print(" Last Heartbeat: %s" % vehicle.last_heartbeat)
print(" Is Armable?: %s" % vehicle.is_armable)
print(" System status: %s" % vehicle.system_status.state)
print(" Mode: %s" % vehicle.mode.name)
print(" Longitude: ", vehicle.location._lon)
print(" Latitude: ", vehicle.location._lat)

def arm_and_takeoff(targetAltitude):
    reachingAltitude = True
    print("Basic arm checks")
    while not vehicle.is_armable:
        print("Waiting for vehicle to initalize")
        time.sleep(2)

    print("Arming drone")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print("Vehicle arming..")
        time.sleep(2)
    
    print("Drone taking off..")
    vehicle.simple_takeoff(targetAltitude)

    while reachingAltitude:
        print("Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= targetAltitude * 0.95:
            print("Reaching altitude")
            break
        time.sleep(1)

    

arm_and_takeoff(5)



