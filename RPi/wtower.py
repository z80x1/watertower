#!/usr/bin/env python3

import signal, os, sys
from time import sleep
import RPi.GPIO as GPIO
import ads1115
import paho.mqtt.client as paho
import pdb
import configparser

CONF_FILE = "/etc/wtower.conf"
gconf = {}

gtopics = {}

SW_STATES = {'on': GPIO.LOW}
SW_STATES['off'] = GPIO.HIGH

#output GPIOs for comtrolling relays module
#ground - pin 20, +5V - pin 4
#RR1 - remote control
#RR3 - control Nasos1
#RR4 - control Nasos2
GPIO_SW_CNTL = {'RR1': 25} #pin 22
GPIO_SW_CNTL['RR2'] = 12   #pin 32
GPIO_SW_CNTL['RR3'] = 23   #pin 16
GPIO_SW_CNTL['RR4'] = 24   #pin 18

#input GPIOs for reading status of pumps and other sensors
# connect next PINs to GROUND pin #25 
GPIO_IN = {'Nasos1': 17} #pin 11
GPIO_IN['Nasos2'] = 27   #pin 13
GPIO_IN['N3'] = 22   #pin 15
GPIO_IN['N4'] = 5    #pin 29
GPIO_IN['N5'] = 6    #pin 31
GPIO_IN['N6'] = 13   #pin 33
GPIO_IN['N7'] = 19   #pin 35
GPIO_IN['N8'] = 26   #pin 37
GPIO_IN['N9'] = 16   #pin 36
GPIO_IN['N10'] = 20  #pin 38
GPIO_IN['N11'] = 21  #pin 40


def signal_handler(signum, frame):
    print("Signal handler called with signal %d. Cleaning up and exiting" % signum)
    gpio_cleanup()
    sys.exit(0)

signal.signal(signal.SIGTERM, signal_handler)
signal.signal(signal.SIGINT, signal_handler)

def load_config():
    parser = configparser.ConfigParser()
    parser.read(CONF_FILE)
    section = parser['default']
    gconf['type'] = section.get('Nodetype', 'wtower')
    gconf['name'] = section.get('Nodename', 'xxx')
    gconf['broker_ip'] = section.get('BrokerIp', '10.10.10.213')
    gconf['broker_port'] = section.getint('BrokerPort', 1883)
    print("Loaded config: %s" % str(gconf)) 

    gtopics['status'] = gconf['name']+"/status"
    gtopics['pressure'] = gconf['name']+"/pressure"
    gtopics['relay'] = gconf['name']+"/relay"
    gtopics['system'] = gconf['name']+"/system"

def gpio_setup():
    # use P1 header pin numbering convention
    GPIO.setmode(GPIO.BCM)

    for k, v in GPIO_SW_CNTL.items():
        # Set up the GPIO channels
        print("%s: setting GPIO %d as OUT" % (k, v))
        GPIO.setup(v, GPIO.OUT)
        GPIO.output(v, SW_STATES['off'])

    for k, v in GPIO_IN.items():
        # Set up the GPIO channels
        print("%s: setting GPIO %d as IN" % (k, v))
        GPIO.setup(v, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def gpio_cleanup():
    for k, v in GPIO_SW_CNTL.items():
        print("%s: cleaning up GPIO%d" % (k, v))
        GPIO.cleanup(v)

    for k, v in GPIO_IN.items():
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

def read_inputs_status():
    N1_status = GPIO.input(GPIO_IN['Nasos1'])
    N2_status = GPIO.input(GPIO_IN['Nasos2'])
    N3_status = GPIO.input(GPIO_IN['N3'])

    return (N1_status, N2_status, N3_status)

def on_connect(client, userdata, flags, rc):
    print("CONNACK received with code %d" % (rc))
    print("Adding subscriptions to '%s'" % gtopics['relay'])
    (rc, mid) = client.subscribe(gtopics['relay'], qos=1)
    if rc == paho.MQTT_ERR_SUCCESS:
        print("Sent subscribe request, mid:%d" % (mid))
    else:
        print("Subscription error occured: %s, mid:%d" % (rc, mid))

#TODO next can be used to separate messages between differert callbacks
#    message_callback_add(sub, callback)

def on_publish(client, userdata, mid):
    print("mid: "+str(mid))

def on_subscribe(client, userdata, mid, granted_qos):
    print("Subscribed OK:  mid:"+str(mid)+", qos:"+str(granted_qos))
 
def on_message(client, userdata, msg):
    print("Got:"+msg.topic+" qos:"+str(msg.qos)+" "+str(msg.payload))
    try:
        (relay, action) = msg.payload.decode().split(":")
    except ValueError as e:
        print("Ignoring incorrect message: %s" % (e))
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
    print("Connecting to broker at address %s:%d" % (gconf['broker_ip'], gconf['broker_port']))
    while True:
        try:
            client.connect(gconf['broker_ip'], gconf['broker_port'])
            break
        except Exception as e:
            print("Broker connnection error(%s): %s" % (e.errno, e.strerror))
            sleep(10)
            continue

    #MQTT will message seems to be not working
    lwm = "Unexpectedly gone offline"
    client.will_set(gtopics['system'], lwm, qos=1, retain=False)

    client.loop_start()

    return client

def main():

    load_config()

    gpio_setup()
    #FIXME crash if no ADS on I2C is detected
    ads1115.ads_setup()

    client = mqtt_setup()
    (rc, mid) = client.publish(gtopics['system'], "Starting work", qos=1)

    old_pressure = 0

    while True:
        pressure = read_from_pressure_sensor()
        if pressure != old_pressure:
            print("pressure:%s" % pressure)
            (rc, mid) = client.publish(gtopics['pressure'], str(pressure), qos=0)
            old_pressure = pressure

        #add comparing current status with previous and send msg only if different
        status = read_inputs_status()
        print("inputs status:%s" % str(status))
        (rc, mid) = client.publish(gtopics['status'], str(status), qos=1)

        sleep(10)
 
if __name__=="__main__":
   main()
