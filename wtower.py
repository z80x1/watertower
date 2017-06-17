#!/usr/bin/env python 

import signal, os, sys
from time import sleep
import RPi.GPIO as GPIO
import random
import paho.mqtt.client as paho
import pdb

#BROKER_SERVER = "10.10.10.70"
BROKER_SERVER = "10.10.10.64"
BROKER_PORT = 1883
OBJECT = "kns1"
TOPIC_PRESSURE = OBJECT+"/pressure"
TOPIC_PUMPS_STATUS = OBJECT+"/pumps"
TOPIC_RELAYS = OBJECT+"/relay"

SW_CNTL_OFF = GPIO.HIGH
SW_CNTL_ON = GPIO.LOW

#output GPIOs for comtrolling relays module
#ground - pin 20, +5V - pin 4
GPIO_SW_CNTL = {'RR1': 25} #pin 22
GPIO_SW_CNTL['RR3'] = 23 #pin 16
GPIO_SW_CNTL['RR4'] = 24 #pin 18

#input GPIOs for reading status of H1, H2
GPIO_PUMPS = {'H1': 17} #pin 11
GPIO_PUMPS['H2'] = 27 #pin 13

def signal_handler(signum, frame):
    print 'Signal handler called with signal', signum , '. Cleaning up and exiting'
    gpio_cleanup()
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
        GPIO.output(v, SW_CNTL_OFF)

    for k, v in GPIO_PUMPS.iteritems():
        # Set up the GPIO channels
        print k, ': setting GPIO', v, 'as IN'
        GPIO.setup(v, GPIO.IN)

def gpio_cleanup():
    for k, v in GPIO_SW_CNTL.iteritems():
        print k, ': cleaning up GPIO', v
        GPIO.cleanup(v)

    for k, v in GPIO_PUMPS.iteritems():
        print k, ': cleaning up GPIO', v
        GPIO.cleanup(v)

def read_from_pressure_sensor():
    #TODO implement reading pressure sensor via I2C bus 
    #see example http://www.raspberrypi-spy.co.uk/2015/04/bmp180-i2c-digital-barometric-pressure-sensor/
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
#  pdb.set_trace()
    try:
        gpio = GPIO_SW_CNTL[msg.payload]

        GPIO.output(gpio, SW_CNTL_ON)
        sleep(2)
        GPIO.output(gpio, SW_CNTL_OFF)
    except KeyError:
        pass
    except Exception:
        pass

def mqtt_setup():
    client = paho.Client()
    client.on_connect = on_connect
    client.on_publish = on_publish
    client.on_subscribe = on_subscribe
    client.on_message = on_message

    #TODO add authorization
    #client.username_pw_set("username", "password")

    #TODO add SSl support. See http://www.hivemq.com/blog/mqtt-client-library-paho-python and https://gist.github.com/sharonbn/4104301
    #client.tls_set()

    client.connect(BROKER_SERVER, BROKER_PORT)

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
