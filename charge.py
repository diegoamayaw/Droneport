#!/usr/bin/env python3
import RPi.GPIO as GPIO
import paho.mqtt.publish as publish
import paho.mqtt.client as mqtt
import time

#########################################################################
# Pin/relay configuration

#########################################################################
drone_relay = 23
charge_relay = 24
is_charging = False
battery_level = 0.0
#Mode
GPIO.setmode(GPIO.BCM)

#Setup
GPIO.setup(drone_relay,GPIO.OUT,initial = GPIO.LOW)
GPIO.setup(charge_relay,GPIO.OUT,initial = GPIO.LOW)

#Initial output
GPIO.output(drone_relay,GPIO.LOW)
GPIO.output(charge_relay,GPIO.LOW)

def start_charge():
    print("Enabling Charging")
    GPIO.output(drone_relay,GPIO.HIGH)
    GPIO.output(charge_relay,GPIO.HIGH)
    print("...")

def end_charge():
    print("disabling Charging")
    GPIO.output(drone_relay,GPIO.LOW)
    GPIO.output(charge_relay,GPIO.LOW)
    print("...")

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

    # Subscribing in on_connect() - if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe("/cloud/connection",0) # Connection establishment topic
    client.subscribe("/cloud/data",0) # Exchange data topic

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    ##client.loop_stop()

    print('Received message')
    ##msg.topic
    if "cloud/data" in str(msg.topic):
        if "start-mission" in str(msg.payload):
            print("Charging")
            start_charge()
            client.publish("device/data","Drone charging")
            print("Sleeping 30 seconds")

            time.sleep(30)
            end_charge()
            print("Charged")
            client.publish("device/data","Drone charged")
            #GPIO.cleanup()


def on_publish(client,userdata,result):
    print("Data published \n")
    pass

## Check battery management in telemetry.py

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.on_publish = on_publish

client.username_pw_set("fjzobzzq", "yvU8HXpyeaRv")
client.connect("soldier.cloudmqtt.com", 17132)

client.loop_forever()

GPIO.cleanup()
