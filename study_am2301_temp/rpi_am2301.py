#!/usr/bin/python3

# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT
# Installation: python3 -m pip install homeassistant-mqtt-binding adafruit-circuitpython-bmp3xx adafruit-circuitpython-sgp30 adafruit-circuitpython-dht
import paho.mqtt.client as mqtt
from ha_mqtt.mqtt_thermometer import MqttThermometer
from ha_mqtt.mqtt_device_base import MqttDeviceSettings
from ha_mqtt.ha_device import HaDevice

import time
import board
import adafruit_dht
import datetime
import logging

logging.basicConfig(level=logging.DEBUG)

def on_log(mqttc, obj, level, string):
    print("[{}] {} - {}  - {}".format(datetime.datetime.now(),level,string,obj))

# instantiate an paho mqtt client and connect to the mqtt server
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,"StudyRPi")
client.connect("homeassistant.local", 1883)
# client.loop_start()
client.on_log = on_log
# instantiate an MQTTThermometer object
dev = HaDevice("Study AM2301", "StudyAM2301")
th = MqttThermometer(MqttDeviceSettings("Study AM2301 Temperature", "StudyTemp",client,dev),"°C")
hum = MqttThermometer(MqttDeviceSettings("Study AM2301 Humidity", "StudyHumidity",client,dev),"%")
#th = MqttThermometer("Study", "StudyTemp",client)

# Initial the dht device, with data pin connected to:
dhtDevice = adafruit_dht.DHT22(board.D10)

# you can pass DHT22 use_pulseio=False if you wouldn't like to use pulseio.
# This may be necessary on a Linux single board computer like the Raspberry Pi,
# but it will not work in CircuitPython.
# dhtDevice = adafruit_dht.DHT22(board.D18, use_pulseio=False)
success = False
for i in range(10):
    try:
        # Print the values to the serial port
        temperature_c = dhtDevice.temperature
        humidity = dhtDevice.humidity
        temp = f"{temperature_c:2.2f}"
        #print(f"publishing temperature: {temp} {th.unit_of_measurement}")
        th.publish_state(temp)
        hum.publish_state(humidity)
        success = True
        print(
            "[{}] Published Temp: {}    Humidity: {}".format(datetime.datetime.now(),temp, humidity)
        )
        time.sleep(2.0)
        break

    except RuntimeError as error:
        # Errors happen fairly often, DHT's are hard to read, just keep going
        print("RuntimeError: {}".format(error.args[0]))
        time.sleep(2.0)
        continue
    except Exception as error:
        dhtDevice.exit()
        raise error

print("stopping loop")
client.loop_stop()
dhtDevice.exit()
#th.close()
#hum.close()
client.disconnect()
print("closed connection")
