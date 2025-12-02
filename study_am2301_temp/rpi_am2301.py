#!/usr/bin/python3

# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT
# Installation: python3 -m pip install homeassistant-mqtt-binding adafruit-circuitpython-bmp3xx adafruit-circuitpython-sgp30 adafruit-circuitpython-dht
import paho.mqtt.client as mqtt
# potential better alternative https://pypi.org/project/ha-mqtt-discoverable/#sensor
from ha_mqtt_discoverable import Settings, DeviceInfo
from ha_mqtt_discoverable.sensors import Sensor, SensorInfo
from adafruit_blinka.microcontroller.rockchip.rk3588 import pin
import gpiod

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

# Configure MQTT settings
mqtt_settings = Settings.MQTT(host="homeassistant.local", client=client)

# Define device
device = DeviceInfo(name="Study AM2301", identifiers="StudyAM2301")

# Create temperature sensor
temp_info = SensorInfo(name="Study AM2301 Temperature", unique_id="StudyTemp", 
                      device_class="temperature", unit_of_measurement="°C", device=device, state_class="measurement")
temp_settings = Settings(mqtt=mqtt_settings, entity=temp_info)
th = Sensor(temp_settings)

# Create humidity sensor
hum_info = SensorInfo(name="Study AM2301 Humidity", unique_id="StudyHumidity", 
                     device_class="humidity", unit_of_measurement="%", device=device, state_class="measurement")
hum_settings = Settings(mqtt=mqtt_settings, entity=hum_info)
hum = Sensor(hum_settings)

# Initial the dht device, with data pin connected to:
# D15
dhtDevice = adafruit_dht.DHT22(pin.GPIO3_C1)

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
        print(f"publishing temperature: {temp}")
        th.set_state(temp)
        hum.set_state(humidity)
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
# th.stop()
# hum.stop()
client.loop_stop()
dhtDevice.exit()
#th.close()
#hum.close()
client.disconnect()
print("closed connection")
