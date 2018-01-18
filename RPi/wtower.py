#!/usr/bin/env python3

import signal, os, sys
from time import sleep
import RPi.GPIO as GPIO
import ads1115
import paho.mqtt.client as paho
import pdb

#BROKER_SERVER = "10.10.10.70"
BROKER_SERVER = "54.93.165.93"
BROKER_PORT = 1883
OBJECT = "kns1"
TOPIC_PRESSURE = OBJECT+"/pressure"
TOPIC_PUMPS_STATUS = OBJECT+"/pumps"
TOPIC_RELAYS = OBJECT+"/relay"

SW_STATES = {'on': GPIO.LOW}
SW_STATES['off'] = GPIO.HIGH

#output GPIOs for comtrolling relays module
#ground - pin 20, +5V - pin 4
GPIO_SW_CNTL = {'RR1': 25} #pin 22
GPIO_SW_CNTL['RR3'] = 23 #pin 16
GPIO_SW_CNTL['RR4'] = 24 #pin 18

#input GPIOs for reading status of pumps N1, N2
GPIO_PUMPS = {'N1': 17} #pin 11
GPIO_PUMPS['N2'] = 27   #pin 13
GPIO_PUMPS['N3'] = 22   #pin 15
GPIO_PUMPS['N4'] = 5    #pin 29
GPIO_PUMPS['N5'] = 6    #pin 31
GPIO_PUMPS['N6'] = 13   #pin 33
GPIO_PUMPS['N7'] = 19   #pin 35
GPIO_PUMPS['N8'] = 26   #pin 37
GPIO_PUMPS['N9'] = 16   #pin 36
GPIO_PUMPS['N10'] = 20  #pin 38
GPIO_PUMPS['N11'] = 21  #pin 40


def signal_handler(signum, frame):
    print("Signal handler called with signal %d. Cleaning up and exiting" % signum)
    gpio_cleanup()
    sys.exit(0)

signal.signal(signal.SIGTERM, signal_handler)
signal.signal(signal.SIGINT, signal_handler)

def gpio_setup():
    # use P1 header pin numbering convention
    GPIO.setmode(GPIO.BCM)

    for k, v in GPIO_SW_CNTL.items():
        # Set up the GPIO channels
        print("%s: setting GPIO %d as OUT" % (k, v))
        GPIO.setup(v, GPIO.OUT)
        GPIO.output(v, SW_STATES['off'])

    for k, v in GPIO_PUMPS.items():
        # Set up the GPIO channels
        print("%s: setting GPIO %d as IN" % (k, v))
        GPIO.setup(v, GPIO.IN)

def gpio_cleanup():
    for k, v in GPIO_SW_CNTL.items():
        print("%s: cleaning up GPIO%d" % (k, v))
        GPIO.cleanup(v)

    for k, v in GPIO_PUMPS.items():
        print("%s: cleaning up GPIO%d" % (k, v))
        GPIO.cleanup(v)

def read_from_pressure_sensor():
    #TODO move to using https://github.com/adafruit/Adafruit_Python_ADS1x15
    #TODO add 4-20mA range overflow detection
    U = 1024*ads1115.ads_read()/32768 #mV
    #calibration coefficients for ADZ-SML-10.0 4-20mA pressure sensor
    # 51 Ohm resistor is used as load
    a = 82.5 #got from calibration
    b = 196  # p=0.0Atm -> U=196mV
             # p=2.0Atm -> U=361mV 
    pressure = (U-b)/a
    return "{:.2f}".format(pressure)+" Atm"

def read_pumps_status():
    N1_status = GPIO.input(GPIO_PUMPS['N1'])
    N2_status = GPIO.input(GPIO_PUMPS['N2'])
    N3_status = GPIO.input(GPIO_PUMPS['N3'])

    return (N1_status, N2_status, N3_status)

def on_connect(client, userdata, flags, rc):
    print("CONNACK received with code %d. Adding subscriptions" % (rc))
    client.subscribe(TOPIC_RELAYS, qos=1)
#TODO next can be used to separate messages between differert callbacks
#    message_callback_add(sub, callback)

def on_publish(client, userdata, mid):
    print("mid: "+str(mid))

def on_subscribe(client, userdata, mid, granted_qos):
    print("Subscribed: "+str(mid)+" "+str(granted_qos))
 
def on_message(client, userdata, msg):
    print(msg.topic+" "+str(msg.qos)+" "+str(msg.payload))
    try:
        (relay, action) = msg.payload.split(":")
    except ValueError:
        print("Incorrent message payload received. Ignoring")
        return

    try:
        gpio = GPIO_SW_CNTL[relay]
    except KeyError:
        print("Received message to unknown target '"+relay+"'. Ignoring")
        return

    state = SW_STATES.get(action, SW_STATES['off'])

    print("Switching relay "+relay+"/GPIO"+str(gpio)+" action "+action+"("+str(state)+")")
    GPIO.output(gpio, state)

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

#  pdb.set_trace()
    while True:
        try:
            client.connect(BROKER_SERVER, BROKER_PORT)
            break
        except Exception as e:
            print("Broker connnection error(%s): %s" % (e.errno, e.strerror))
            sleep(10)
            continue

    client.loop_start()

    return client

def main():

    gpio_setup()
    #FIXME crash if no ADS on I2C is detected
    ads1115.ads_setup()

    client = mqtt_setup()

    while True:
        pressure = read_from_pressure_sensor()
        print("pressure:%s" % pressure)
        (rc, mid) = client.publish(TOPIC_PRESSURE, str(pressure), qos=1)

        pumps = read_pumps_status()
        print("pumps status:%s" % str(pumps))
        (rc, mid) = client.publish(TOPIC_PUMPS_STATUS, str(pumps), qos=1)

        sleep(10)
 
if __name__=="__main__":
   main()
