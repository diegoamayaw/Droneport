#!/usr/bin/env python3
import asyncio

from mavsdk import start_mavlink
from mavsdk import connect as mavsdk_connect

start_mavlink(connection_url="serial:///dev/ttyUSB0:57600")
drone = mavsdk_connect(host="127.0.0.1")

battery_level = 0.0


async def getBattery():
    global battery_level

    async for battery in drone.telemetry.battery():
        battery_level = battery.remaining_percent
        return


asyncio.ensure_future(getBattery())
loop = asyncio.get_event_loop()
loop.run_until_complete(getBattery())
print(battery_level)
