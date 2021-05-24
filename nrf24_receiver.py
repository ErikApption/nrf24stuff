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
payload = [0.0,0.0]

def slave(timeout=290):
    """Listen for any payloads and print the transaction

    :param int timeout: The number of seconds to wait (with no transmission)
        until exiting function.
    """
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
            payload[0] = struct.unpack("<f", buffer[:4])[0]
            payload[1] = struct.unpack("<f", buffer[4:8])[0]
            volt = payload[1] / 73.78378
            ret1 = client.publish(root + "/DS18B20/Temperature", "{:0.2f}".format(payload[0]))
            ret2 = client.publish(root + "/Arduino/Humidity","{:0.2f}".format(payload[1]))
            published = (ret1.rc == paho.MQTT_ERR_SUCCESS)
            #print("Date={5} Temp={0:0.1f}C Humidity={1:0.1f}% Published={2} ret1={3} ret2={4}".format(temperature, humidity,published,ret1,ret2,datetime.datetime.now()))
            fullPath = os.path.expanduser('~/last_update.txt')
            with open(fullPath,'w') as last_update:
                last_update.write("Temp: {0:0.1f} °C".format(payload[0]) + "\n")
                last_update.write("Voltage (abs): {0:0.1f}".format(payload[1]))


            # print details about the received packet
            print(
                "{} Received {} bytes on pipe {}: {} + {} ({}) - Published {}".format(
                    str(datetime.datetime.now()),
		    radio.payloadSize,
                    pipe_number,
                    payload[0], volt, payload[1], published
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
    root = "hottub"

    # initialize the nRF24L01 on the spi bus
    if not radio.begin():
        raise RuntimeError("radio hardware is not responding")

    # For this example, we will use different addresses
    # An address need to be a buffer protocol object (bytearray)
    address = [b"1Node", b"2Node"]
    # It is very helpful to think of an address as a path instead of as
    # an identifying device destination

    # to use different addresses on a pair of radios, we need a variable to
    # uniquely identify which address this radio will use to transmit
    # 0 uses address[0] to transmit, 1 uses address[1] to transmit
    radio_number = 1  # uses default value from `parser`

    # set the Power Amplifier level to -12 dBm since this test example is
    # usually run with nRF24L01 transceivers in close proximity of each other
    # radio.setPALevel(RF24_PA_LOW)  # RF24_PA_MAX is default
    radio.setChannel(5)
    # set the TX address of the RX node into the TX pipe
    radio.openWritingPipe(address[radio_number])  # always uses pipe 0

    # set the RX address of the TX node into a RX pipe
    radio.openReadingPipe(1, address[not radio_number])  # using pipe 1

    # To save time during transmission, we'll set the payload size to be only
    # what we need. A float value occupies 4 bytes in memory using
    # struct.pack(); "<f" means a little endian unsigned float
    radio.payloadSize = len(struct.pack("<f", payload[0]))*2

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
