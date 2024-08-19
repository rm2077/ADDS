import asyncio
from mavsdk import System

async def run():

    drone = System()
    await drone.connect()
    await drone.action.arm()
    await drone.action.takeoff()

    print("Succeeded")
    
run()