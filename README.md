# Watertower monitoring system

## System architecture

[TODO]

## Rasberry Pi node side

* Install latest Raspbian on RPi
* Enable SSH and I2C via raspi-config on RPi
* Update system
```bash
sudo rpi-update
sudo apt update && sudo apt dist-upgrade
```

### Install packages on RPi

```bash
apt install -y git python3 python3-pip 
apt-get install -y python3-smbus i2c-tools
apt install openvpn wvdial policykit-1 tmux
#for debugging install mosquitto-clients
apt install mosquitto-clients
```

Install [paho-mqtt](https://github.com/eclipse/paho.mqtt.python)
```bash
pip3 install paho-mqtt RPi.GPIO
```
### Setup monitoring

Clone repository
```bash
mkdir /opt/watertower
cd /opt/watertower
git clone https://github.com/z80x1/watertower.git
chmod +x /opt/watertower/RPi/wtower.py
```

Example config file
```bash
$ cat /etc/wtower.conf
[default]
Nodetype = tower
Nodename = wt2
BrokerIp = 10.10.10.213
BrokerPort = 1883
```

#### Systemd services

Unit file
```bash
# cat /etc/systemd/system/wtower.service
[Unit]
Description = monitoring pumps and sensors
#After = networking.target
After = wvdial.target

[Service]
ExecStart = /opt/watertower/RPi/wtower.py
StandardOutput=syslog
StandardError=syslog
SyslogIdentifier=wtower
Restart=always
RestartSec=10

[Install]
WantedBy = multi-user.target
```
Enable service
```bash
systemctl enable wtower.service
systemctl start wtower.service
```
### Setup Internet access

#### For 3G modem using wvdial 

```bash
#unneeded with Raspbian 9 and Huawei E172 modem
sudo usb_modeswitch -v 12d1 -p 14fe -J
```

```bash
# cat /etc/wvdial.conf
[Dialer life]
Modem = /dev/ttyUSB0
Baud = 460800
Init1 = ATZ
Init2 = AT+CGDCONT=1,"IP","internet"
Phone = *99***1#
Stupid mode = 1
Username = " "
Password = " "
```
Manual run wvdial
```bash
sudo wvdial life &
```

##### Setup autodialing service
```bash
# cat /opt/modem.sh
#!/bin/bash

usb_modeswitch -v 12d1 -p 14fe -J
sleep 1
wvdial life
```

```bash
sudo chmod +x /opt/modem.sh
```

wvdial service
```bash
# cat /etc/systemd/system/wvdial.service
[Unit]
Description = WvDial service
After = networking.target

[Service]
ExecStart = /opt/modem.sh
Restart=always
RestartSec=10

[Install]
WantedBy = multi-user.target
```

Enable service
```bash
systemctl enable wvdial.service
systemctl start wvdial.service
```

## Server side

### Install MQTT broker Docker container on server

* Install Docker on server
```bash
apt install mosquitto-clients docker.io
```

* Add user wtower
* Using vigr utility add wtower to groups adm,sudo,docker,netdev

* Pull and run [mosquitto container](https://hub.docker.com/_/eclipse-mosquitto/)
```bash
docker pull eclipse-mosquitto
sudo mkdir -p  /srv/mosquitto/{data,log}
sudo chown -R :docker /srv/mosquitto
sudo chmod -R g+w /srv/mosquitto
sudo chmod -R 777 /srv/mosquitto/{log,data}
/srv/mosquitto/run_broker.sh
```

### Working with broker 

Publishing instructions to relays
```bash
mosquitto_pub -h localhost -t wt2/set/Pump2 -m on -q 1 --retain
```
Only Pump1, Pump2 and RemCtrl topics are permitable.
Payload can be either 'on' or 'off'.

Subscribing to all messages coming to broker
```bash
mosquitto_sub -h localhost -t '#' -v | xargs -d$'\n' -L1 sh -c 'date "+%D %T.%3N $0"'
```

Connecting to broker with TLS enabled
```bash
mosquitto_sub -h localhost -p 8883 --cafile /srv/mosquitto/data/tls/ca.crt --insecure -t '#' -v | xargs -d$'\n' -L1 sh -c 'date "+%D %T.%3N $0"'
```

### Web

[mqtt-panel](https://github.com/fabaff/mqtt-panel)

Run HTTP server
```bash
cd web/mqtt-panel
docker run -dit --name my-apache-app -p 8080:80 -v "$PWD":/usr/local/apache2/htdocs/ httpd:2.4
```

Point your browser to http://localhost:8080


