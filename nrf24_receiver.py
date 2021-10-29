"""
A simple example of sending data from 1 nRF24L01 transceiver to another.
This example was written to be used on 2 devices acting as 'nodes'.
"""
import sys
import argparse
import time
import struct
import datetime
from RF24 import RF24, RF24_PA_LOW
import paho.mqtt.client as paho
import os.path

########### USER CONFIGURATION ###########
# See https://github.com/TMRh20/RF24/blob/master/pyRF24/readme.md
# Radio CE Pin, CSN Pin, SPI Speed
# CE Pin uses GPIO number with BCM and SPIDEV drivers, other platforms use
# their own pin numbering
# CS Pin addresses the SPI bus number at /dev/spidev<a>.<b>
# ie: RF24 radio(<ce_pin>, <a>*10+<b>); spidev1.0 is 10, spidev1.1 is 11 etc..

# Generic:
radio = RF24(22, 0)
################## Linux (BBB,x86,etc) #########################
# See http://nRF24.github.io/RF24/pages.html for more information on usage
# See http://iotdk.intel.com/docs/master/mraa/ for more information on MRAA
# See https://www.kernel.org/doc/Documentation/spi/spidev for more
# information on SPIDEV

# using the python keyword global is bad practice. Instead we'll use a 1 item
# list to store our float number for the payloads sent/received

nodeID = 0;
payloadID = 0;  
temp = 0.0;
voltage = 0.0;

# For this example, we will use different addresses
# An address need to be a buffer protocol object (bytearray)
node_addresses = [b"2Node", b"3Node", b"4Node", b"5Node", b"6Node"]
node_roots =  ["hottub","weather","pool"]

def slave(timeout=298):
    """Listen for any payloads and print the transaction

    :param int timeout: The number of seconds to wait (with no transmission)
        until exiting function.
    """
    print("waiting for signal... radio payload:{}".format(radio.payloadSize))
    radio.startListening()  # put radio in RX mode

    start_timer = time.monotonic()
    while (time.monotonic() - start_timer) < timeout:
        has_payload, pipe_number = radio.available_pipe()
        if has_payload:
            # fetch 1 payload from RX FIFO
            buffer = radio.read(radio.payloadSize)
            # use struct.unpack() to convert the buffer into usable data
            # expecting a little endian float, thus the format string "<f"
            # buffer[:4] truncates padded 0s in case payloadSize was not set
            print("payload received... pipe:{} buffer:{} radio payload:{}".format(pipe_number, len(buffer),radio.payloadSize))

            bufStart = 0;
            bufEnd = 0;
            #unsigned long nodeID;
            bufStart = bufEnd;
            bufEnd = bufStart + struct.calcsize('H');
            nodeID = struct.unpack("<H", buffer[bufStart:bufEnd])[0]
            
            #unsigned long payloadID;  
            bufStart = bufEnd;
            bufEnd = bufStart + struct.calcsize('H');
            payloadID = struct.unpack("<H", buffer[bufStart:bufEnd])[0]

            #float temp;
            bufStart = bufEnd;
            bufEnd = bufStart + struct.calcsize('f');
            temp = struct.unpack("<f", buffer[bufStart:bufEnd])[0]

            #float voltage;
            bufStart = bufEnd;
            bufEnd = bufStart + struct.calcsize('f');
            voltage = struct.unpack("<f", buffer[bufStart:bufEnd])[0]
            #convert mv to V
            if voltage > 0:
               voltage = voltage * 1.0 / 1000.0

            #float humidity;
            bufStart = bufEnd;
            bufEnd = bufStart + struct.calcsize('f');
            humidity = struct.unpack("<f", buffer[bufStart:bufEnd])[0]

            #unsigned long luxMeasure;
            bufStart = bufEnd;
            bufEnd = bufStart + struct.calcsize('H');
            luxMeasure = struct.unpack("<H", buffer[bufStart:bufEnd])[0]

            #unsigned long amb_als;
            bufStart = bufEnd;
            bufEnd = bufStart + struct.calcsize('H');
            amb_als = struct.unpack("<H", buffer[bufStart:bufEnd])[0]

            #unsigned long amb_ir;  
            bufStart = bufEnd;
            bufEnd = bufStart + struct.calcsize('H');
            amb_ir = struct.unpack("<H", buffer[bufStart:bufEnd])[0]

            #float uv_index;
            bufStart = bufEnd;
            bufEnd = bufStart + struct.calcsize('f');
            uv_index = struct.unpack("<f", buffer[bufStart:bufEnd])[0]

            #debug
            fullPath = os.path.expanduser("~/last_update_{}.txt".format(nodeID))
            with open(fullPath,'w') as last_update:
                last_update.write("Pipe {} NodeID {} PayloadID {}".format(pipe_number,nodeID,payloadID))
                last_update.write(" Temp: {0:0.1f} °C".format(temp) + "\n")
                last_update.write("Voltage (abs): {0:0.1f}".format(voltage) + "\n")
                last_update.write("Humidity {} LuxMeasure {} Ambiant Light {} Ambiant IR {} UV {}".format(humidity,luxMeasure,amb_als,amb_ir,uv_index))
                last_update.write("\n")

            print ("{} {} Data T:{} V:{} H:{}% Lux:{} AL:{} IR:{} UV:{}".format(str(datetime.datetime.now()),pipe_number,temp,voltage,humidity,luxMeasure,amb_als,amb_ir,uv_index));

            # publish data
            published = False
            ret1 = None;
            ret2 = client.publish(node_roots[nodeID] + "/Arduino/Voltage","{:0.2f}".format(voltage),qos=2, retain=True)
            if (nodeID == 0 or nodeID == 2):
                ret1 = client.publish(node_roots[nodeID] + "/DS18B20/Temperature", "{:0.2f}".format(temp))
            elif nodeID == 1:
                ret1 = client.publish(node_roots[nodeID] + "/DHT/Temperature", "{:0.2f}".format(temp))
                ret7 = client.publish(node_roots[nodeID] + "/DHT/Humidity", "{:0.2f}".format(humidity))
                ret3 = client.publish(node_roots[nodeID] + "/TSL2561/Lux","{:0.2f}".format(luxMeasure))
                ret4 = client.publish(node_roots[nodeID] + "/GY1145/AL","{:0.2f}".format(amb_als))
                ret5 = client.publish(node_roots[nodeID] + "/GY1145/IR","{:0.2f}".format(amb_ir))
                ret6 = client.publish(node_roots[nodeID] + "/GY1145/UV","{:0.3f}".format(uv_index/100.0))
            published = (ret1.rc == paho.MQTT_ERR_SUCCESS)
                #print("Date={5} Temp={0:0.1f}C Humidity={1:0.1f}% Published={2} ret1={3} ret2={4}".format(temperature, humidity,published,ret1,ret2,datetime.datetime.now()))

            # print details about the received packet
            print(
                "{} {} Received {} bytes Published {} - node {} - payload {}".format(
                    str(datetime.datetime.now()), pipe_number, radio.payloadSize, published,nodeID,payloadID
                )
            )
            #start_timer = time.monotonic()  # reset the timeout timer

    print("{} - No data received - Leaving RX role...".format(str(datetime.datetime.now())));
    # recommended behavior is to keep in TX mode while idle
    radio.stopListening()  # put the radio in TX mode


if __name__ == "__main__":

    client = paho.Client()
    #client.on_publish = on_publish
    client.connect("openhab.local", 1883,60)

    # initialize the nRF24L01 on the spi bus
    if not radio.begin():
        raise RuntimeError("radio hardware is not responding")

    receiver_address = b"1Node";
    # It is very helpful to think of an address as a path instead of as
    # an identifying device destination

    # to use different addresses on a pair of radios, we need a variable to
    # uniquely identify which address this radio will use to transmit
    # 0 uses address[0] to transmit, 1 uses address[1] to transmit

    # set the Power Amplifier level to -12 dBm since this test example is
    # usually run with nRF24L01 transceivers in close proximity of each other
    radio.setPALevel(RF24_PA_LOW)  # RF24_PA_MAX is default
    radio.setChannel(5)

    radio.setAutoAck(True)
    # set the TX address of the RX node into the TX pipe
    radio.openWritingPipe(receiver_address)  # always uses pipe 0

    # set the RX address of the TX node into a RX pipe
    # write the addresses to all pipes.
    for pipe_n, addr in enumerate(node_addresses):
        radio.openReadingPipe(pipe_n, addr)    

    # To save time during transmission, we'll set the payload size to be only
    # what we need. A float value occupies 4 bytes in memory using
    # struct.pack(); "<f" means a little endian unsigned float
    radio.payloadSize = (2*struct.calcsize('H')+2*struct.calcsize('f')+
                         struct.calcsize('f')+3*struct.calcsize('H')+struct.calcsize('f'))
    # for debugging, we have 2 options that print a large block of details
    # (smaller) function that prints raw register values
    # radio.printDetails()
    # (larger) function that prints human readable data
    # radio.printPrettyDetails()

    try:
        slave()
    except KeyboardInterrupt:
        print(" Keyboard Interrupt detected. Exiting...")
        radio.powerDown()
        sys.exit()
