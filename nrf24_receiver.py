#!/usr/bin/python3
"""
A simple example of sending data from 1 nRF24L01 transceiver to another.
This example was written to be used on 2 devices acting as 'nodes'.
"""
import sys
import argparse
import time
import struct
import datetime
import numpy as np
from pyrf24 import RF24, RF24_PA_LOW, RF24_DRIVER, RF24_PA_MAX
import paho.mqtt.client as paho
import os.path
#on orange pi
import OPi.GPIO as GPIO
#on raspberry pi
#import RPi.GPIO as GPIO
import threading
import socket
import gpiod
from gpiod.line import Edge

hostname = socket.gethostname()
radio_status = f"{hostname}/NRF24/Status"

########### USER CONFIGURATION ###########
# See https:#github.com/TMRh20/RF24/blob/master/pyRF24/readme.md
# Radio CE Pin, CSN Pin, SPI Speed
# CE Pin uses GPIO number with BCM and SPIDEV drivers, other platforms use
# their own pin numbering
# CS Pin addresses the SPI bus number at /dev/spidev<a>.<b>
# ie: RF24 radio(<ce_pin>, <a>*10+<b>); spidev1.0 is 10, spidev1.1 is 11 etc..

# Generic:
#RPI
#radio = RF24(22, 0)
#IRQ_PIN = 27

#opi
radio = RF24(15, 0)
IRQ_PIN = 7

################## Linux (BBB,x86,etc) #########################
# See http:#nRF24.github.io/RF24/pages.html for more information on usage
# See http:#iotdk.intel.com/docs/master/mraa/ for more information on MRAA
# See https:#www.kernel.org/doc/Documentation/spi/spidev for more
# information on SPIDEV

# using the python keyword global is bad practice. Instead we'll use a 1 item
# list to store our float number for the payloads sent/received

nodeID = 0
payloadID = 0
temp = 0.0
voltage = 0.0

# For this example, we will use different addresses
# An address need to be a buffer protocol object (bytearray)
node_addresses = [b"2Node", b"3Node", b"4Node", b"5Node", b"6Node",b"7Node"]
# node_roots =  ["hottub","weather","pool"]
node_roots = ["pool", "weather", "hottub","garden","fridge","watersensor"]

def interrupt_handler():
    """This function is called when IRQ pin is detected active LOW"""
    #print("IRQ pin", channel, "went active LOW.")
    tx_ds, tx_df, rx_dr = radio.whatHappened()   # get IRQ status flags
    if tx_df:
        radio.flush_tx()
    print("Interrupt - tx_ds: {}, tx_df: {}, rx_dr: {}".format(tx_ds, tx_df, rx_dr))
    if rx_dr:
        # process payload and check
        process_payload(True)

# setup IRQ GPIO pin
# The GPIO.BOARD option specifies that you are referring to the pins by the number of the pin on the plug 
# GPIO.setmode(GPIO.BOARD)
# GPIO.setup(IRQ_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
# GPIO.add_event_detect(IRQ_PIN, GPIO.FALLING, callback=interrupt_handler)

def process_payload(check_payload):
    has_payload, pipe_number = radio.available_pipe()
    if has_payload:        
        payloadLen = radio.getDynamicPayloadSize()
        # fetch 1 payload from RX FIFO
        buffer = radio.read(payloadLen)
        print("payload received... pipe:{} buffer:{} radio payload:{}".format(
            pipe_number, len(buffer), payloadLen))

        #t = threading.Thread(target=process_payload2,args=(pipe_number,buffer))
        #t.start()
        process_payload2(pipe_number,buffer)
        return True
    else:
        if check_payload:
            print("weird - IRQ triggered but no payload ???!!!")
        return False

def process_payload2(pipe_number,buffer):
    # use struct.unpack() to convert the buffer into usable data
    # expecting a little endian float, thus the format string "<f"
    # buffer[:4] truncates padded 0s in case payloadSize was not set


    # # publish data
    # client.connect("openhab.local", 1883, 60)

    # client.loop_start()

    TryPublish(radio_status, "ON", 2, True)

    qos = 2
    retain = True
    published = False

    bufStart = 0
    bufEnd = 0
    # unsigned long nodeID;
    bufStart = bufEnd
    bufEnd = bufStart + struct.calcsize('b')
    nodeID = int(struct.unpack("b", buffer[bufStart:bufEnd])[0])

    # unsigned long payloadID;
    bufStart = bufEnd
    bufEnd = bufStart + struct.calcsize('b')
    payloadID = int(struct.unpack("b", buffer[bufStart:bufEnd])[0])
    if (payloadID == 0):
        # float temp;
        bufStart = bufEnd
        bufEnd = bufStart + struct.calcsize('f')
        temp = struct.unpack("<f", buffer[bufStart:bufEnd])[0]

        # float voltage;
        bufStart = bufEnd
        bufEnd = bufStart + struct.calcsize('f')
        voltage = struct.unpack("<f", buffer[bufStart:bufEnd])[0]
        # convert mv to V
        if voltage > 0:
            voltage = voltage * 1.0 / 1000.0

        # float humidity;
        bufStart = bufEnd
        bufEnd = bufStart + struct.calcsize('f')
        humidity = struct.unpack("<f", buffer[bufStart:bufEnd])[0]

        TryPublish(node_roots[nodeID] +
                    "/Arduino/Voltage", voltage, qos, retain)

        if (nodeID == 0 or nodeID == 2):
            TryPublish(
                node_roots[nodeID] + "/DS18B20/Temperature", temp, qos, retain)
        elif nodeID == 1:
            TryPublish(node_roots[nodeID] +
                        "/DHT/Temperature", temp, qos, retain)
            TryPublish(node_roots[nodeID] +
                        "/DHT/Humidity", humidity, qos, retain)
        elif nodeID == 4:
            TryPublish(node_roots[nodeID] +
                        "/AM2320/Temperature", temp, qos, retain)
            TryPublish(node_roots[nodeID] +
                        "/AM2320/Humidity", humidity, qos, retain)

        print("{} {} Data T:{:0.2f} V:{} H:{}%".format(
            str(datetime.datetime.now()), pipe_number, temp, voltage, humidity))

        fullPath = os.path.expanduser(
            "~/last_update_{}.txt".format(nodeID))
        with open(fullPath, 'w') as last_update:
            last_update.write("Pipe {} NodeID {} PayloadID {}".format(
                pipe_number, nodeID, payloadID))
            last_update.write(" Temp: {0:0.1f} °C".format(temp) + "\n")
            last_update.write(
                "Voltage (abs): {0:0.1f}".format(voltage) + "\n")
            last_update.write("Humidity {}".format(humidity))
            # last_update.write("Humidity {} UV {} UVa {} UVb {}".format(humidity,uv_index,uv_a,uv_b))
            last_update.write("\n")

    elif payloadID == 1:
        # debug buffer
        vhex = np.vectorize(hex)
        # print("buffer={}".format(vhex(buffer)))
        # unsigned long amb_als;
        bufStart = bufEnd
        bufEnd = bufStart + struct.calcsize('L')
        amb_als = struct.unpack("<L", buffer[bufStart:bufEnd])[0]

        # unsigned long amb_ir;
        bufStart = bufEnd
        bufEnd = bufStart + struct.calcsize('L')
        amb_ir = struct.unpack("<L", buffer[bufStart:bufEnd])[0]
        #print("amb_ir={}".format(vhex(buffer[bufStart:bufEnd])))

        # float uv_index;
        bufStart = bufEnd;
        bufEnd = bufStart + struct.calcsize('f');
        uv_index = 1.0 * struct.unpack("<f", buffer[bufStart:bufEnd])[0]
        # https:#github.com/adafruit/Adafruit_SI1145_Library/blob/master/examples/si1145test/si1145test.ino
        uv_index = uv_index / 100.0; # the index is multiplied by 100 

        # float uv_index;
        bufStart = bufEnd;
        bufEnd = bufStart + struct.calcsize('L');
        readout_ms = struct.unpack("<L", buffer[bufStart:bufEnd])[0]                
                            
        lux = calcLux(amb_als,amb_ir)
        TryPublish(node_roots[nodeID] +
                    "/GY1145/AL", amb_als, qos, retain)
        TryPublish(node_roots[nodeID] +
                    "/GY1145/IR", amb_ir, qos, retain)
        TryPublish(node_roots[nodeID] +
                    "/GY1145/UV", uv_index, qos, retain)      
        TryPublish(node_roots[nodeID] +
                    "/GY1145/Luminosity", lux, qos, retain)                                                
        print("{} {} Data AmbALS:{} AmbIR:{} UV:{:0.2f} Lux:{} - {} ms".format(
            str(datetime.datetime.now()), pipe_number, amb_als, amb_ir,uv_index,lux,readout_ms))

    elif payloadID == 3:
        # unsigned int lux; should be 4 bytes but is 2
        bufStart = bufEnd
        bufEnd = bufStart + struct.calcsize('f')
        luxValue = struct.unpack("<f", buffer[bufStart:bufEnd])[0]
                            
        TryPublish(node_roots[nodeID] +
                    "/TSL2261/LUX", luxValue, qos, retain)

        # float voltage;
        bufStart = bufEnd
        bufEnd = bufStart + struct.calcsize('f')
        voltage = struct.unpack("<f", buffer[bufStart:bufEnd])[0]
        # convert mv to V
        if voltage > 0:
            voltage = voltage * 1.0 / 1000.0

        TryPublish(node_roots[nodeID] +
                    "/Arduino/Voltage", voltage, qos, retain)

        print("{} {} Data Lux:{} V:{}".format(
            str(datetime.datetime.now()), pipe_number, luxValue,voltage))

    elif payloadID == 4:
        # unsigned int lux; should be 4 bytes but is 2
        bufStart = bufEnd
        bufEnd = bufStart + struct.calcsize('h')
        analogValue = struct.unpack("<h", buffer[bufStart:bufEnd])[0]
                            
        TryPublish(node_roots[nodeID] +
                    "/Arduino/Analog", analogValue, qos, retain)

        # float voltage;
        bufStart = bufEnd
        bufEnd = bufStart + struct.calcsize('f')
        voltage = struct.unpack("<f", buffer[bufStart:bufEnd])[0]
        # convert mv to V
        if voltage > 0:
            voltage = voltage * 1.0 / 1000.0

        TryPublish(node_roots[nodeID] +
                    "/Arduino/Voltage", voltage, qos, retain)

        print("{} {} Data Analog:{} V:{}".format(
            str(datetime.datetime.now()), pipe_number, analogValue,voltage))

    else:
        print("invalid payload id:{}".format(payloadID))

    # debug


        # TryPublish(node_roots[nodeID] + "/TSL2561/Lux",luxMeasure,qos, retain)
        # TryPublish(node_roots[nodeID] + "/VEML6075/UVi",uv_index,qos, retain)
        # TryPublish(node_roots[nodeID] + "/VEML6075/UVa",uv_a,qos, retain)
        # TryPublish(node_roots[nodeID] + "/VEML6075/UVb",uv_b,qos, retain)
        # print("Date={5} Temp={0:0.1f}C Humidity={1:0.1f}% Published={2} ret1={3} ret2={4}".format(temperature, humidity,published,ret1,ret2,datetime.datetime.now()))
    #client.loop_stop()
    # print details about the received packet
    print(
        "{} {} Received {} bytes - node {} - payload {}".format(
            str(datetime.datetime.now()
                ), pipe_number, radio.payloadSize, nodeID, payloadID
        )
    )
    # start_timer = time.monotonic()  # reset the timeout timer


def listen(timeout=1198): #//20 minutes restart
    """Listen for any payloads and print the transaction

    :param int timeout: The number of seconds to wait (with no transmission)
        until exiting function.
    """
    #print("{} {} Data Lux:{} V:{}".format(str(datetime.datetime.now()), pipe_number, luxValue,voltage))
    print(f"{str(datetime.datetime.now())} - waiting for signal... radio payload:{radio.payloadSize}")
    radio.listen = True  # put radio in RX mode

    start_timer = time.monotonic()
    while (time.monotonic() - start_timer) < timeout:
        # process payload but do not check
        if (process_payload(False)):
            start_timer = time.monotonic()
    time.sleep(0.5)
    print(f"{str(datetime.datetime.now())} - No data received - Leaving RX role...")
    # recommended behavior is to keep in TX mode while idle
    radio.listen = False
    


def calcLux(vis, ir):
  vis_dark = 256 # empirical value
  ir_dark = 250 # empirical value
  gainFactor = 1.0
  visCoeff = 5.41 # application notes AN523
  irCoeff = 0.08 # application notes AN523
  visCountPerLux = 0.319 # for incandescent bulb (datasheet)
  irCountPerLux = 8.46 # for incandescent bulb (datasheet)
  corrFactor = 0.18 # my empirical correction factor
  
  # According to application notes AN523: 
  lux = ((vis - vis_dark) * visCoeff - (ir - ir_dark) * irCoeff) * gainFactor

  # the equation above does not consider the counts/Lux depending on light source type
  # I suggest the following equation
  # float lux = (((vis - vis_dark) / visCountPerLux) * visCoeff - ((ir - ir_dark) / irCountPerLux) * irCoeff) * gainFactor * corrFactor
  
  return lux


def TryPublish(topic, payload=None, qos=0, retain=False):
    retries = 0
    success = False
    while (retries < 5 and success == False):
        ret = client.publish(topic, payload, qos, retain)
        ret.wait_for_publish()
        success = ret.rc == paho.MQTT_ERR_SUCCESS
        retries = retries + 1
    if (retries > 1):
        print("WARNING: {} retries for message - success: {}".format(retries, success))
    if (success):
        print("Successfully published {}:{} retries: {}".format(
            topic, payload, retries))
    else:
        print("Error publishing {}:{} retries: {}".format(topic, payload, retries))


if __name__ == "__main__":

    client = paho.Client(paho.CallbackAPIVersion.VERSION2)

    # connect mqtt broker
    client.connect("homeassistant.local", 1883, 60)

    client.loop_start()
    print("connected to MQTT broker")

    # client.on_publish = on_publish
    # initialize the nRF24L01 on the spi bus
    if not radio.begin():
        TryPublish(radio_status, "OFF", 2, True)
        raise RuntimeError("radio hardware is not responding")

    receiver_address = b"1Node"
    #print("{} {} Data Lux:{} V:{}".format(str(datetime.datetime.now()), pipe_number, luxValue,voltage))
    print(f"{str(datetime.datetime.now())} - Receiver address is {receiver_address.hex()} - driver is {RF24_DRIVER}")
    # It is very helpful to think of an address as a path instead of as
    # an identifying device destination

    # to use different addresses on a pair of radios, we need a variable to
    # uniquely identify which address this radio will use to transmit
    # 0 uses address[0] to transmit, 1 uses address[1] to transmit

    # set the Power Amplifier level to -12 dBm since this test example is
    # usually run with nRF24L01 transceivers in close proximity of each other
    radio.set_pa_level(RF24_PA_LOW, False)  # RF24_PA_MAX is default #RF24_PA_LOW
    radio.channel = 5

    #radio.setAutoAck(True)
    # set the TX address of the RX node into the TX pipe
    radio.open_tx_pipe(receiver_address)  # always uses pipe 0

    # set the RX address of the TX node into a RX pipe
    # write the addresses to all pipes.
    for pipe_n, addr in enumerate(node_addresses):
        radio.open_rx_pipe(pipe_n, addr)

    # To save time during transmission, we'll set the payload size to be only
    # what we need. A float value occupies 4 bytes in memory using
    # struct.pack(); "<f" means a little endian unsigned float
    radio.dynamic_payloads = True
    # radio.payloadSize = (2*struct.calcsize('H')+2*struct.calcsize('f')+
    #                     struct.calcsize('f')+3*struct.calcsize('H')+struct.calcsize('f'))
    # for debugging, we have 2 options that print a large block of details
    # (smaller) function that prints raw register values
    # radio.printDetails()
    # (larger) function that prints human readable data
    # radio.printPrettyDetails()

    # on data ready test
    #Configuring IRQ pin to only ignore 'on data sent' event
    #radio.maskIRQ(True, False, False)  # args = tx_ds, tx_df, rx_dr

    try:
        print(f"hostname is {hostname}")        
        TryPublish(radio_status, "ON", 2, True)
        listen()
        client.loop_stop()
    except KeyboardInterrupt:
        print(" Keyboard Interrupt detected. Exiting...")
        radio.powerDown()
        client.loop_stop()
        sys.exit()
