# watertower

enable SSH and I2C via raspi-config on RPi

# Install packages on RPi

apt install mosquitto-clients
apt install python-pip
apt-get install -y python-smbus i2c-tools

pip install paho-mqtt
https://github.com/eclipse/paho.mqtt.python

#Install MQTT broker Docker container on server

1) Install Docker
https://hub.docker.com/_/eclipse-mosquitto/
docker pull eclipse-mosquitto
sudo mkdir -p  /srv/mosquitto/{data,log}
sudo chown -R :docker /srv/mosquitto
sudo chmod -R g+w /srv/mosquitto
sudo chmod -R 777 /srv/mosquitto/{log,data}
/srv/mosquitto/run_broker.sh


#Working with broker 

Publishing instructions to relays
mosquitto_pub -h localhost -t kns1/relay -m RR3 -q 2
Only RR1, RR3 and RR4 messages are permitable

Subscribing to all messages coming to broker
mosquitto_sub -h 10.10.10.64 -t kns1/#


# Web

https://github.com/fabaff/mqtt-panel
