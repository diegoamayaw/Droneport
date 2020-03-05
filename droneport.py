#!/usr/bin/env python3
import asyncio
import RPi.GPIO as GPIO

from mavsdk import start_mavlink
from mavsdk import connect as mavsdk_connect
import paho.mqtt.publish as publish
import paho.mqtt.client as mqtt
import time

#########################################################################

# Motor configuration

#########################################################################

rising_ticks1 = 0
rising_ticks2 = 0
ticks_for_opening = 100#510 # Equals ~90°
encoder1 = 13 # GPIO13 for encoder (CH1) of motor 1
encoder2 = 19 # GPIO19 for encoder (CH1) of motor 2
mot1_pwm = 20 # Motor PWM in GPIO20
#mot2_pwm = 21 # Motor PWM in GPIO21
dir1 = 12 # Digital pin for direction of motor1
dir2 = 16 #Digital pin for direction of motor2

stop_motor1 = False # Control flag
stop_motor2 = False # Control flag
#Mode
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
#Encoder
GPIO.setup(encoder1, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(encoder2, GPIO.IN, pull_up_down=GPIO.PUD_UP)
#PWM
GPIO.setup(mot1_pwm, GPIO.OUT)
#GPIO.setup(mot2_pwm, GPIO.OUT)
motor_control = GPIO.PWM(mot1_pwm, 15000) #1.5KHz Frequency
#motor2 = GPIO.PWM(mot2_pwm, 15000) #1.5KHz Frequency
#Direction
GPIO.setup(dir1,GPIO.OUT)
GPIO.setup(dir2,GPIO.OUT)

# Function that counts ticks of each motor via interrupts
def interrupt_handler_rising1(channel):
    global rising_ticks1
    global stop_motor1
    rising_ticks1 += 1
    #print(rising_ticks1)
    if(rising_ticks1>=ticks_for_opening):
        stop_motor1 = True;

def interrupt_handler_rising2(channel):
    global rising_ticks2
    global stop
    rising_ticks2 += 1
    #print(rising_ticks2)
    if(rising_ticks2>=ticks_for_opening):
        stop_motor2 = True

# Function that moves motors depending on direction (open/close)
def move_motors(direction):
    global stop_motor1
    global stop_motor2

    if direction == 'open':
        GPIO.output(dir1,GPIO.LOW)
        GPIO.output(dir2,GPIO.LOW)
        motor_control.start(50) #Duty Cycle

        while(not stop_motor1 and not stop_motor2): ##Change to AND?
            continue

        motor_control.stop()
        stop_motor1 = False
        stop_motor2 = False

    elif direction == 'close':
        print("closing")
        GPIO.output(dir1,GPIO.HIGH)
        GPIO.output(dir2,GPIO.HIGH)
        motor_control.start(50) #Duty Cycle

        while(not stop_motor1 and not stop_motor2): ##Change to AND?
            continue

        motor_control.stop()
        stop_motor1 = False
        stop_motor2 = False

# Definition of interrupts
GPIO.add_event_detect(encoder1, GPIO.RISING,
                      callback=interrupt_handler_rising1,
                      bouncetime=20)

GPIO.add_event_detect(encoder2, GPIO.RISING,
                      callback=interrupt_handler_rising2,
                      bouncetime=20)

#########################################################################

# Charge configuration

#########################################################################
drone_relay = 23
charge_relay = 24
battery_level = 0.0

#Setup
GPIO.setup(drone_relay,GPIO.OUT,initial = GPIO.LOW)
GPIO.setup(charge_relay,GPIO.OUT,initial = GPIO.LOW)

GPIO.output(drone_relay,GPIO.LOW)
GPIO.output(charge_relay,GPIO.LOW)

async def getBattery():
    global battery_level

    async for battery in drone.telemetry.battery():
        battery_level = battery.remaining_percent
        return

#########################################################################

# MAVLink configuration

#########################################################################

start_mavlink(connection_url="serial:///dev/ttyUSB0:57600") #Connection to drone
drone = mavsdk_connect(host="127.0.0.1") #Connect drone to host

async def run():

    conn_state = drone.core.connection_state()
    async for state in conn_state:
        if state.is_connected:
            return state.uuid


async def get_fw():

    info = await drone.info.get_version()
    return info

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

    # Subscribing in on_connect() - if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe("/cloud/connection",0) # Connection establishment topic
    client.subscribe("/cloud/data",0) # Exchange data topic
    client.subscribe("/device/next-action",0)

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    ##client.loop_stop()

    print('Received message')
    ##msg.topic
    if "cloud/connection" in str(msg.topic):

        if "dp-connection-request" in str(msg.payload):
            print("connection request")
            loop = asyncio.get_event_loop()
            try:
                x,info = loop.run_until_complete(asyncio.gather(run(),get_fw()))
                client.publish("/device/connection","successful")
                ack = "Connection successful to drone with UUID: " + str(x)
                client.publish("/device/data", ack)

            except Exception as exc:
                print('The coroutine raised an exception: {!r}'.format(exc))
                ack="failed"
                client.publish("/device/connection",ack)

    if "cloud/data" in str(msg.topic):
        if "start-mission" in str(msg.payload):
            #publish next action start_motors
            print("Starting mission")
            msg = "open_motors"
            client.publish("/device/next-action",msg)

    if "device/next-action" in str(msg.topic):
        if "open_motors" in str(msg.payload):
            open_motors()
        elif "arm_drone" in str(msg.payload):
            loop = asyncio.get_event_loop()
            loop.run_until_complete(arm_drone())
            # arm_drone()
        elif "kill_drone" in str(msg.payload):
            loop = asyncio.get_event_loop()
            loop.run_until_complete(disarm_drone())
            # disarm_drone()
        elif "close_motors" in str(msg.payload):
            close_motors()


def on_publish(client,userdata,result):
    print("Data published \n")
    pass

def open_motors():
    global stop_motor1
    global stop_motor2
    global rising_ticks1
    global rising_ticks2

    stop_motor1 = False
    stop_motor2 = False
    rising_ticks1 = 0
    rising_ticks2 = 0
    move_motors('open')
    print("Droneport open")
    client.publish("/device/data","Droneport open")
    msg = "arm_drone"
    #msg = "close_motors"
    client.publish("/device/next-action",msg)
    time.sleep(1)

async def arm_drone():
    print("arm drone")
    try:
        await drone.action.arm()
        print("Drone armed")
        client.publish("/device/data","Drone armed")

    except Exception as exc:
        print('The coroutine raised an exception: {!r}'.format(exc))
        client.publish("/device/data","Drone couldn't be armed")

    print("Sleeping 5 seconds")
    time.sleep(5)
    msg = "kill_drone"
    client.publish("/device/next-action",msg)
    return

async def disarm_drone():
    print("disarm drone")
    try:
        await drone.action.disarm()
        print("Drone disarmed")
        client.publish("/device/data","Drone disarmed")

    except Exception as exc:
        print('The coroutine raised an exception: {!r}'.format(exc))
        client.publish("/device/data","Drone couldn't be disarmed")

    time.sleep(1)
    msg = "close_motors"
    client.publish("/device/next-action",msg)
    return


def close_motors():
    global stop_motor1
    global stop_motor2
    global rising_ticks1
    global rising_ticks2

    print("Closing")
    stop_motor1 = False
    stop_motor2 = False
    rising_ticks1 = 0
    rising_ticks2 = 0
    move_motors('close')
    client.publish("/device/data","Droneport closed")

## Check battery management in telemetry.py

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.on_publish = on_publish

client.username_pw_set("********", "********")
client.connect("*********************", 00000)

client.loop_forever()
