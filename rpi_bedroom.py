#!/usr/bin/env python3
import busio
import adafruit_sgp30
import time
import board
import sys
import argparse
import time
import struct
import datetime
from paho.mqtt.client import Client
#python3 -m pip install homeassistant-mqtt-binding adafruit-circuitpython-bmp3xx adafruit-circuitpython-sgp30 gpiod
from ha_mqtt.mqtt_thermometer import MqttThermometer
from ha_mqtt.mqtt_sensor import MqttSensor
from ha_mqtt.util import HaDeviceClass
from ha_mqtt.mqtt_device_base import MqttDeviceSettings
from ha_mqtt.ha_device import HaDevice
from adafruit_blinka.microcontroller.generic_linux.libgpiod_pin import Pin
import os.path

import board
import time
#from adafruit_bme280 import basic as adafruit_bme280
#bme390
import adafruit_bmp3xx

if __name__ == "__main__":
    #i2c_bus = busio.I2C() #board.SCL, board.SDA, frequency=100000

    # instantiate an paho mqtt client and connect to the mqtt server
    client = Client("BedroomRPi")
    client.connect("homeassistant.local", 1883)
    client.loop_start()

    # instantiate an MQTTThermometer object
    dev = HaDevice("Bedroom BMP390", "BedroomBMP390")
    th = MqttThermometer(MqttDeviceSettings("Bedroom BMP390", "BedroomTemp",client,dev),"°C")
    pres = MqttSensor(MqttDeviceSettings("Bedroom BMP390", "BedroomPressure",client,dev),"hPa",HaDeviceClass.PRESSURE)

    dev2 = HaDevice("Bedroom SGP30", "BedroomSGP30")
    co2 = MqttSensor(MqttDeviceSettings("Bedroom SGP30", "Bedroom_eCO2",client,dev2),"ppm",HaDeviceClass.PM10)
    tvoc = MqttSensor(MqttDeviceSettings("Bedroom SGP30", "Bedroom_TVOC",client,dev2),"ppb",HaDeviceClass.PM10)    
    #th = MqttThermometer("Bedroom", "BedroomTemp",client)    
    # good for RPI
    #i2c = board.I2C()
    i2c = board.I2C()
    sgp30 = adafruit_sgp30.Adafruit_SGP30(i2c)    

    measurement_time = time.time()
    dataError = True
    while dataError:
        #dataError = eco2 < 5 or voc > 120 or eco2 > 30000
        dataError = False
        eCO2, TVOC = sgp30.iaq_measure()
        print("eco2=",eCO2, " voc=",TVOC," dataError=", dataError)
        print(
            "**** Baseline values: eCO2 = 0x%x, TVOC = 0x%x"
            % (sgp30.baseline_eCO2, sgp30.baseline_TVOC)
        )        
        if not dataError:
            co2.publish_state(round(eCO2, 2))
            tvoc.publish_state(round(TVOC, 2))
            client.publish("Bedroom/SGP30/CO2",eCO2)
            client.publish("Bedroom/SGP30/TVOC",TVOC)         
            print("published SGP30 data")

    dataError = True

    #address = 0x77
    #i2c = board.I2C()  # uses board.SCL and board.SDA

    # the sample method will take a single reading and return a
    # compensated_reading object
    dataError = True
    while dataError:
        bmp = adafruit_bmp3xx.BMP3XX_I2C(i2c)
        bmp.pressure_oversampling = 16
        bmp.temperature_oversampling = 4
        dataError = bmp.temperature > 50 or bmp.pressure > 1100
        print("temp=",bmp.temperature," pressure",bmp.pressure," dataError=",dataError)
        if not dataError:
            th.publish_state(round(bmp.temperature, 2))
            pres.publish_state(round(bmp.pressure, 4))
            client.publish("Bedroom/BME280/Temperature",bmp.temperature)
            client.publish("Bedroom/BME280/Pressure",bmp.pressure)            
            print("published BMP390 data")

    print("stopping loop")
    client.loop_stop()
    #th.close()
    #hum.close()
    client.disconnect()
    print("closed connection")
