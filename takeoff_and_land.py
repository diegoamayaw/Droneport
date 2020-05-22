#!/usr/bin/env python3

import asyncio

from mavsdk import start_mavlink
from mavsdk import connect as mavsdk_connect

start_mavlink(connection_url="serial:///dev/ttyUSB0:57600") #Connection to drone
drone = mavsdk_connect(host="127.0.0.1") #Connect drone to host


async def run():

    print("Waiting for drone...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Drone discovered with UUID: {state.uuid}")
            break

    # print("-- Arming")
    # await drone.action.arm()
    #
    # print("-- Taking off")
    # await drone.action.takeoff()
    #
    # await asyncio.sleep(5)
    #
    # print("-- Landing")
    # await drone.action.land()


loop = asyncio.get_event_loop()
loop.run_until_complete(run())
