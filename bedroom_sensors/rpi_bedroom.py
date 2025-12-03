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
import paho.mqtt.client as paho
#should be https://pypi.org/project/ha-mqtt-discoverable/
#python3 -m pip install ha-mqtt-discoverable adafruit-circuitpython-bmp3xx adafruit-circuitpython-sgp30 gpiod
from ha_mqtt_discoverable import Settings, DeviceInfo
from ha_mqtt_discoverable.sensors import Sensor, SensorInfo
#from adafruit_blinka.microcontroller.generic_linux.libgpiod_pin import Pin
import os.path

import board
import time
#from adafruit_bme280 import basic as adafruit_bme280
#bme390
import adafruit_bmp3xx

if __name__ == "__main__":
    #i2c_bus = busio.I2C() #board.SCL, board.SDA, frequency=100000

    # Configure MQTT settings
    mqtt_settings = Settings.MQTT(host="homeassistant.local")

    # Define devices
    device_bmp390 = DeviceInfo(name="Bedroom BMP390", identifiers="BedroomBMP390")
    device_sgp30 = DeviceInfo(name="Bedroom SGP30", identifiers="BedroomSGP30")

    # Create temperature sensor for BMP390
    temp_info = SensorInfo(name="Temperature", unique_id="temperature", 
                          device_class="temperature", unit_of_measurement="°C", device=device_bmp390, state_class="measurement")
    temp_settings = Settings(mqtt=mqtt_settings, entity=temp_info)
    th = Sensor(temp_settings)

    # Create pressure sensor for BMP390
    pres_info = SensorInfo(name="Pressure", unique_id="pressure", 
                          device_class="pressure", unit_of_measurement="hPa", device=device_bmp390, state_class="measurement")
    pres_settings = Settings(mqtt=mqtt_settings, entity=pres_info)
    pres = Sensor(pres_settings)

    # Create CO2 sensor for SGP30
    co2_info = SensorInfo(name="Bedroom eCO2", unique_id="bedroom_sgp30_eco2", 
                         unit_of_measurement="ppm", device=device_sgp30)
    co2_settings = Settings(mqtt=mqtt_settings, entity=co2_info)
    co2 = Sensor(co2_settings)

    # Create TVOC sensor for SGP30
    tvoc_info = SensorInfo(name="Bedroom TVOC", unique_id="bedroom_sgp30_tvoc", 
                          unit_of_measurement="ppb", device=device_sgp30)
    tvoc_settings = Settings(mqtt=mqtt_settings, entity=tvoc_info)
    tvoc = Sensor(tvoc_settings)    
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
            co2.set_state(round(eCO2, 2))
            tvoc.set_state(round(TVOC, 2))        
            print(f"published SGP30 data eCO2: {eCO2} TVOC: {TVOC}")

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
            th.set_state(round(bmp.temperature, 2))
            pres.set_state(round(bmp.pressure, 4))      
            print("published BMP390 data")

    print("closed connection")
