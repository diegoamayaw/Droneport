#!/usr/bin/env python3
import RPi.GPIO as GPIO
import paho.mqtt.publish as publish
import paho.mqtt.client as mqtt
import time


#########################################################################
# Motor configuration

#########################################################################

rising_ticks1 = 0
rising_ticks2 = 0
ticks_for_opening = 510 # Equals ~90°
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
#Encoder
GPIO.setup(encoder1, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(encoder2, GPIO.IN, pull_up_down=GPIO.PUD_UP)
#PWM
GPIO.setup(mot1_pwm, GPIO.OUT)
#GPIO.setup(mot1_pwm, GPIO.OUT)
motor_control = GPIO.PWM(mot1_pwm, 15000) #1.5KHz Frequency
#motor2 = GPIO.PWM(mot1_pwm, 15000) #1.5KHz Frequency
#Direction
GPIO.setup(dir1,GPIO.OUT)
GPIO.setup(dir2,GPIO.OUT)

# Function that counts ticks of each motor via interrupts
def interrupt_handler_rising1(channel):
    global rising_ticks1
    global stop_motor1
    rising_ticks1 += 1
    print(rising_ticks1)

    if(rising_ticks1>=ticks_for_opening):
        stop_motor1 = True;
        print("Stop motor 1")

def interrupt_handler_rising2(channel):
    global rising_ticks2
    global stop
    rising_ticks2 += 1
    print(rising_ticks2)

    if(rising_ticks2>=ticks_for_opening):
        stop_motor2 = True
        print("Stop motor 2")

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

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

    # Subscribing in on_connect() - if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe("/cloud/connection",0) # Connection establishment topic
    client.subscribe("/cloud/data",0) # Exchange data topic

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    ##client.loop_stop()
    global stop_motor1
    global stop_motor2
    global rising_ticks1
    global rising_ticks2

    print('Received message')
    ##msg.topic
    if "cloud/data" in str(msg.topic):
        if "start-mission" in str(msg.payload):
            print("Opening")
            stop_motor1 = False
            stop_motor2 = False
            rising_ticks1 = 0
            rising_ticks2 = 0
            move_motors('open')
            client.publish("device/data","Droneport open")
            print("Sleeping 5 seconds")

            time.sleep(5)
            print("Closing")
            stop_motor1 = False
            stop_motor2 = False
            rising_ticks1 = 0
            rising_ticks2 = 0
            move_motors('close')
            client.publish("device/data","Droneport closed")


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
print("hello")

GPIO.cleanup()
