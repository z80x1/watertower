#!/usr/bin/env python 

import signal, os, sys
from time import sleep
import RPi.GPIO as GPIO
import random
import paho.mqtt.client as paho

SERVER = "10.10.10.70"
OBJECT = "kns1"
TOPIC_PRESSURE = OBJECT+"/pressure"
TOPIC_PUMPS_STATUS = OBJECT+"/pumps"
TOPIC_RELAYS = OBJECT+"/relay"

GPIO_SW_CNTL = {'RR1': 25}
GPIO_SW_CNTL['RR3'] = 23
GPIO_SW_CNTL['RR4'] = 24

GPIO_PUMPS = {'H1': 17}
GPIO_PUMPS['H2'] = 27

def signal_handler(signum, frame):
    print 'Signal handler called with signal', signum , '. Cleaning up and exiting'
    GPIO.cleanup(25)
    sys.exit(0)

signal.signal(signal.SIGTERM, signal_handler)
signal.signal(signal.SIGINT, signal_handler)

def gpio_setup():
    # use P1 header pin numbering convention
    GPIO.setmode(GPIO.BCM)

    for k, v in GPIO_SW_CNTL.iteritems():
        # Set up the GPIO channels
        print k, ': setting GPIO', v, 'as OUT'
        GPIO.setup(v, GPIO.OUT)

    for k, v in GPIO_PUMPS.iteritems():
        # Set up the GPIO channels
        print k, ': setting GPIO', v, 'as IN'
        GPIO.setup(v, GPIO.OUT)

def read_from_pressure_sensor():
    return random.randint(10, 30)

def read_pumps_status():
    H1_status = GPIO.input(GPIO_PUMPS['H1'])
    H2_status = GPIO.input(GPIO_PUMPS['H2'])

    return (H1_status, H2_status)

def on_connect(client, userdata, flags, rc):
    print("CONNACK received with code %d." % (rc))

def on_publish(client, userdata, mid):
    print("mid: "+str(mid))

def on_subscribe(client, userdata, mid, granted_qos):
    print("Subscribed: "+str(mid)+" "+str(granted_qos))
 
def on_message(client, userdata, msg):
    print(msg.topic+" "+str(msg.qos)+" "+str(msg.payload))
    GPIO.output(25, GPIO.HIGH)
    sleep(2)
    GPIO.output(25, GPIO.LOW)

def mqtt_setup():
    client = paho.Client()
    client.on_connect = on_connect
    client.on_publish = on_publish
    client.on_subscribe = on_subscribe
    client.on_message = on_message

    #client.username_pw_set("username", "password")

    client.connect(SERVER, 1883)

    client.subscribe(TOPIC_RELAYS, qos=1)
 
    client.loop_start()

    return client

def main():

    gpio_setup()

    client = mqtt_setup()

    while True:
        pressure = read_from_pressure_sensor()
        print 'pressure:', pressure
        (rc, mid) = client.publish(TOPIC_PRESSURE, str(pressure), qos=1)

        pumps = read_pumps_status()
        print 'pumps status:', pumps
        (rc, mid) = client.publish(TOPIC_PUMPS_STATUS, str(pumps), qos=1)

        sleep(10)
 
if __name__=="__main__":
   main()
