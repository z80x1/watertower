#!/bin/bash

BASE=/srv/mosquitto
NAME=broker

docker stop $NAME
docker rm $NAME
docker run -it --detach \
        --name $NAME \
        -p 1883:1883 -p 9001:9001 \
        -v $BASE/mosquitto.conf:/mosquitto/config/mosquitto.conf \
        -v $BASE/data:/mosquitto/data \
        -v $BASE/log:/mosquitto/log \
        eclipse-mosquitto
docker ps | grep $NAME
