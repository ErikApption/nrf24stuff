#!/usr/bin/python3

# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT
from paho.mqtt.client import Client
from ha_mqtt.mqtt_thermometer import MqttThermometer
from ha_mqtt.mqtt_device_base import MqttDeviceSettings

import time
import board
import adafruit_dht
import ha_mqtt

# instantiate an paho mqtt client and connect to the mqtt server
client = Client("StudyRPi")
client.connect("openhab.local", 1883)
client.loop_start()

# instantiate an MQTTThermometer object
settings = MqttDeviceSettings("Study AM2301", "StudyTemp",client)
th = MqttThermometer(settings)
#th = MqttThermometer("Study", "StudyTemp",client)

# Initial the dht device, with data pin connected to:
dhtDevice = adafruit_dht.DHT22(board.D17)

# you can pass DHT22 use_pulseio=False if you wouldn't like to use pulseio.
# This may be necessary on a Linux single board computer like the Raspberry Pi,
# but it will not work in CircuitPython.
# dhtDevice = adafruit_dht.DHT22(board.D18, use_pulseio=False)

for i in range(2):
    try:
        # Print the values to the serial port
        temperature_c = dhtDevice.temperature
        temperature_f = temperature_c * (9 / 5) + 32
        humidity = dhtDevice.humidity
        temp = f"{temperature_c:2.2f}"
        print(f"publishing temperature: {temp} {th.unit_of_measurement}")
        th.publish_state(temp)

        print(
            "Temp: {:.1f} F / {:.1f} C    Humidity: {}% ".format(
                temperature_f, temperature_c, humidity
            )
        )
        break

    except RuntimeError as error:
        # Errors happen fairly often, DHT's are hard to read, just keep going
        print(error.args[0])
        time.sleep(2.0)
        continue
    except Exception as error:
        dhtDevice.exit()
        raise error

    time.sleep(2.0)

time.sleep(2.0)
th.close()
client.loop_stop()
client.disconnect()
print("closed connection")
