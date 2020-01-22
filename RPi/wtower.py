#!/usr/bin/env python3

class Unbuffered(object):
   def __init__(self, stream):
       self.stream = stream
   def write(self, data):
       self.stream.write(data)
       self.stream.flush()
   def writelines(self, datas):
       self.stream.writelines(datas)
       self.stream.flush()
   def __getattr__(self, attr):
       return getattr(self.stream, attr)

import sys
sys.stdout = Unbuffered(sys.stdout) #making stdout unbuffered

import signal, os
from time import sleep
import time
import RPi.GPIO as GPIO
import ads1115
import paho.mqtt.client as paho
import pdb
import configparser

CONF_FILE = "/etc/wtower/wtower.conf"
CERT_FILE = "/etc/wtower/ca.crt"
#global variables
gconf = {}
gtopics = {}
glist_statuses = []
glist_alarms = []
mqtt = {}

SW_STATES = {}
SW_STATES['on']  = GPIO.LOW
SW_STATES['off'] = GPIO.HIGH

#output GPIOs for controlling relays module
#ground - pin 20, +5V - pin 4
GPIO_SW_CNTL = {}
GPIO_SW_CNTL['RemCtrl'] = 25 #relays IN1 - pin 22
GPIO_SW_CNTL['Pump3'] = 12 #relays IN2 - pin 32
GPIO_SW_CNTL['Pump1'] = 23 #relays IN3 - pin 16
GPIO_SW_CNTL['Pump2'] = 24 #relays IN4 - pin 18

#input GPIOs for reading status of pumps and other sensors
# connect next PINs to GROUND pin #25 
GPIO_IN = {}
GPIO_IN['Pump1'] = 17 #pin 11
GPIO_IN['Pump2'] = 27   #pin 13
GPIO_IN['ExtPwr'] = 22   #pin 15
GPIO_IN['Pump3'] = 5    #pin 29
GPIO_IN['DoorOpen'] = 6    #pin 31
GPIO_IN['Rake'] = 13   #pin 33
GPIO_IN['Level1'] = 19   #pin 35
GPIO_IN['Level2'] = 26   #pin 37
GPIO_IN['Level3'] = 16   #pin 36
GPIO_IN['DrainOn'] = 20  #pin 38
GPIO_IN['Flood'] = 21  #pin 40


def signal_handler(signum, frame):
    print("Signal handler called with signal %d. Cleaning up and exiting" % signum)
    mqtt.publish(gtopics['online'], "false", qos=1, retain=True)
    mqtt.disconnect()
    gpio_cleanup()
    sys.exit(0)

signal.signal(signal.SIGTERM, signal_handler)
signal.signal(signal.SIGINT, signal_handler)

def load_config():
    global gconf
    global gtopics

    parser = configparser.ConfigParser()
    parser.read(CONF_FILE)
    section = parser['default']
    gconf['type'] = section.get('Nodetype', 'tower') # tower/kns
    gconf['name'] = section.get('Nodename', 'xxx') # wtXX/knsXX
    gconf['broker_host'] = section.get('BrokerHost', 'localhost')
    gconf['broker_port'] = section.getint('BrokerPort', 1883) # 8883 for TLS enabled

    #default values are calibration coefficients for ADZ-SML-10.0 4-20mA pressure sensor
    # with 51 Ohm resistor used as load got from 2 points:
    # p=0.0Atm -> U=196mV
    # p=2.0Atm -> U=361mV
    gconf['ads1115_mnk_a'] = section.getfloat('ADS1115_a', 82.5)
    gconf['ads1115_mnk_b'] = section.getfloat('ADS1115_b', 196)

    #TODO: verify loaded values
    print("Loaded config: %s" % str(gconf)) 

    gtopics['online'] = gconf['name']+"/online"
    gtopics['alarm'] = gconf['name']+"/alarm"
    gtopics['status'] = gconf['name']+"/status"
    gtopics['pressure'] = gconf['name']+"/pressure"
    gtopics['set'] = gconf['name']+"/set"

def classify_gpio():
    global glist_statuses
    global glist_alarms

    if gconf['type'] == 'tower':
        glist_statuses = ["Pump1", "Pump2"]
        glist_alarms = ["ExtPwr"]

    elif gconf['type'] == 'kns':
        glist_statuses = ["Pump1", "Pump2", "Pump3", "Level1", "Level2", "Level3", "DoorOpen", "Rake"]
        glist_alarms = ["ExtPwr", "DrainOn", "Flood"]

    else:
        print("Unsupported system type: %s" % gconf['type'])
        return False
        
    return True

def gpio_setup():
    # use P1 header pin numbering convention
    GPIO.setmode(GPIO.BCM)

    print("Setting relay outputs:")
    for k, v in GPIO_SW_CNTL.items():
        # Set up the GPIO channels
        print(" %s: setting GPIO %d as OUT" % (k, v))
        GPIO.setup(v, GPIO.OUT)
        GPIO.output(v, SW_STATES['off'])

    print("Setting status inputs:")
    for n in glist_statuses:
        v = GPIO_IN[n]
        print(" %s: setting GPIO %d as IN" % (n, GPIO_IN[n]))
        GPIO.setup(GPIO_IN[n], GPIO.IN, pull_up_down=GPIO.PUD_UP)

    print("Setting ALARM inputs:")
    for n in glist_alarms:
        v = GPIO_IN[n]
        print(" %s: setting GPIO %d as IN" % (n, GPIO_IN[n]))
        GPIO.setup(GPIO_IN[n], GPIO.IN, pull_up_down=GPIO.PUD_UP)

def gpio_cleanup():
    print("cleaning up GPIOs")
    GPIO.cleanup()

def read_from_pressure_sensor():
    #TODO move to using https://github.com/adafruit/Adafruit_Python_ADS1x15
    #TODO add 4-20mA range overflow detection
    U = 4*1024*ads1115.ads_read()/32768 #mV
    if U <= 0: #shouldn't happen with correct setup
        return float('NaN')

    pressure = (U-gconf['ads1115_mnk_b'])/gconf['ads1115_mnk_a']
    return "{:.2f}".format(pressure)+" Atm"

def read_inputs_status(input_list):
    status = {}
    for n in input_list:
      level_on = GPIO.HIGH if n=="ExtPwr" else GPIO.LOW #FIXME better to save GPIO level corresponding to ON status in GPIO_IN itself
      status[n] = 'on' if GPIO.input(GPIO_IN[n])==level_on else 'off'

    return status

def on_connect(mqtt, userdata, flags, rc):
    print("CONNACK received with rc=%d" % (rc))

    sub_topic = gtopics['set']+'/#'
    print("Adding subscription to '%s'" % sub_topic)
    (rc, mid) = mqtt.subscribe(sub_topic, qos=1)
    if rc == paho.MQTT_ERR_SUCCESS:
        print("Sent subscribe request, mid:%d" % (mid))
    else:
        print("Subscription error occured: %s, mid:%d" % (rc, mid))

def on_disconnect(mqtt, userdata, rc=0):
    print("DisConnected result code "+str(rc))

#TODO next can be used to separate messages between differert callbacks
#    message_callback_add(sub, callback)

def on_publish(mqtt, userdata, mid):
    #print("mid: "+str(mid))
    return

def on_subscribe(mqtt, userdata, mid, granted_qos):
    print("Subscribed OK:  mid:"+str(mid)+", qos:"+str(granted_qos))
 
def on_message(mqtt, userdata, msg):
    print("Got:"+msg.topic+" qos:"+str(msg.qos)+" "+str(msg.payload))
    try:
        (o, s, relay) = msg.topic.split("/")
        action = msg.payload.decode()
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
    global mqtt

    mqtt = paho.Client(client_id=gconf['name'], clean_session=False, userdata=None, protocol=paho.MQTTv311)
    mqtt.on_connect = on_connect
    mqtt.on_disconnect = on_disconnect
    mqtt.on_publish = on_publish
    mqtt.on_subscribe = on_subscribe
    mqtt.on_message = on_message

    #TODO add authorization
    #mqtt.username_pw_set("username", "password")

    #TODO add SSl support. See http://www.hivemq.com/blog/mqtt-mqtt-library-paho-python and https://gist.github.com/sharonbn/4104301
    mqtt.tls_set(CERT_FILE, tls_version=2)
    mqtt.tls_insecure_set(True)

    #set will message to be displayed when connection interrupted
    mqtt.will_set(gtopics['online'], "false", qos=1, retain=True)

#  pdb.set_trace()
    print("Connecting to broker at address %s:%d" % (gconf['broker_host'], gconf['broker_port']))
    while True:
        try:
            mqtt.connect(gconf['broker_host'], gconf['broker_port'], keepalive=60)
            break
        except Exception as e:
            print("Broker connnection error: %s" % e)
            sleep(10)
            continue

    mqtt.loop_start()

    return mqtt

def main():

    load_config()

    if not classify_gpio():
        return False

    gpio_setup()
    if gconf['type'] == 'tower':
        #FIXME crash if no ADS on I2C is detected
        ads1115.ads_setup()

    mqtt = mqtt_setup()
    (rc, mid) = mqtt.publish(gtopics['online'], "true", qos=1, retain=True)

    old_pressure = 0
    old_alarms = {}
    old_statuses = {}

    while True:
        #TODO check if connection to broker exists

        if gconf['type'] == 'tower':
            pressure = read_from_pressure_sensor()
            if (pressure != float("nan")) and (pressure != old_pressure):
                print("pressure: %s" % pressure)
                (rc, mid) = mqtt.publish(gtopics['pressure'], str(pressure), qos=0, retain=True)
                old_pressure = pressure

        #add comparing current status with previous and send msg only if different
        alarms = read_inputs_status(glist_alarms)
        for k, v in alarms.items():
            if k not in old_alarms.keys() or old_alarms[k] != v:
                print("alarms status: %s %s" % (k, v))
                (rc, mid) = mqtt.publish(gtopics['alarm']+"/"+k, v, qos=1, retain=True)
        old_alarms = alarms

        statuses = read_inputs_status(glist_statuses)
        for k, v in statuses.items():
            if k not in old_statuses.keys() or old_statuses[k] != v:
                print("inputs status: %s %s" % (k, v))
                (rc, mid) = mqtt.publish(gtopics['status']+"/"+k, v, qos=1, retain=True)
        old_statuses = statuses

        sleep(5)
 
if __name__=="__main__":
   main()
