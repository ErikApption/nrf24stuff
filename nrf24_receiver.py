import sys
import argparse
import time
import struct
import datetime
import numpy as np
from pyrf24 import RF24, RF24_PA_LOW, RF24_DRIVER, RF24_PA_MAX, RF24_PA_HIGH
import paho.mqtt.client as paho
import os.path
import threading
import socket
import configparser
import os
import logging
import json
#import gpiod
#from gpiod.line import Edge

# Configure logging
# Check if running as a systemd service
if 'INVOCATION_ID' in os.environ:
    logging.basicConfig(level=logging.INFO, format='%(levelname)s - %(message)s')
else:
    logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

hostname = socket.gethostname()
radio_status = f"weather-gtw/NRF24/Status"
last_hour_msg_count = f"weather-gtw/NRF24/MessageCountLastHour"
msg_count_status_file = os.path.expanduser("~/nrf24_message_counts.json")

# Read configuration
config = configparser.ConfigParser()
config_path = os.path.join(os.path.dirname(__file__), 'receiver.ini')
config.read(config_path)

ce_pin = config.getint('RF24', 'ce_pin', fallback=22)
csn_pin = config.getint('RF24', 'csn_pin', fallback=0)

# List to store message counts for each 15-minute interval
message_counts = [0] * 4
last_update_time = time.time()
message_count = 0

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

#https://wiki.t-firefly.com/en/ROC-RK3588S-PC/usage_gpio.html
#opi GPIO1_A2 - bank = 1 group = 0 X =2
#number = group * 8 + X = 0*8+2 = 2
#pin = bank * 32 + number = 0*32+2 = 34
#GPIO4_A6 - bank = 4 group = 0 X =6
#number = group * 8 + X = 0*8+6 = 6
#pin = bank * 32 + number = 4*32+6 = 134
radio = RF24(ce_pin,csn_pin)

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
    #logging.info("IRQ pin", channel, "went active LOW.")
    tx_ds, tx_df, rx_dr = radio.whatHappened()   # get IRQ status flags
    if tx_df:
        radio.flush_tx()
    logging.info("Interrupt - tx_ds: {}, tx_df: {}, rx_dr: {}".format(tx_ds, tx_df, rx_dr))
    if rx_dr:
        # process payload and check
        process_payload(True)

def save_message_counts_to_json(file_path):
    data = {
        "message_counts": message_counts,
        "message_count": message_count,
        "last_update_time": last_update_time
    }
    with open(file_path, 'w') as json_file:
        json.dump(data, json_file)
    logging.info(f"Message counts saved to {file_path}")

def load_message_counts_from_json(file_path):
    global message_counts, message_count, last_update_time
    if os.path.exists(file_path):
        with open(file_path, 'r') as json_file:
            data = json.load(json_file)
            message_counts = data.get("message_counts", [0] * 4)
            message_count = data.get("message_count", 0)
            last_update_time = data.get("last_update_time", time.time())
        logging.info(f"Message counts loaded from {file_path}")
    else:
        logging.warning(f"{file_path} does not exist. Using default values.")    

def update_message_counts():
    global message_counts, message_count, last_update_time
    # Calculate the total number of messages received in the last hour
    total_messages_last_hour = sum(message_counts) + message_count
    logging.info(f"Messages received in the last hour: {total_messages_last_hour}")
    TryPublish(last_hour_msg_count, f"{total_messages_last_hour}", 2, False)

    current_time = time.time()
    if current_time - last_update_time >= 900:  # 15 minutes
        # Shift the message counts and add the current count
        message_counts.pop(0)
        message_counts.append(message_count)
        # Reset the message count and update the last update time
        message_count = 0
        last_update_time = current_time


# setup IRQ GPIO pin
# The GPIO.BOARD option specifies that you are referring to the pins by the number of the pin on the plug 
# GPIO.setmode(GPIO.BOARD)
# GPIO.setup(IRQ_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
# GPIO.add_event_detect(IRQ_PIN, GPIO.FALLING, callback=interrupt_handler)

def process_payload():
    global message_count
    has_payload, pipe_number = radio.available_pipe()
    if has_payload:        
        payloadLen = radio.getDynamicPayloadSize()
        # fetch 1 payload from RX FIFO
        buffer = radio.read(payloadLen)
        logging.info("payload received... pipe:{} buffer:{} radio payload:{}".format(
            pipe_number, len(buffer), payloadLen))

        # Increment message count
        message_count += 1

        #t = threading.Thread(target=process_payload2,args=(pipe_number,buffer))
        #t.start()
        process_payload2(pipe_number,buffer)
        return True
    else:
        return False

def unpack_from_buffer(buffer, bufEnd_ref, data_type):
    try:
        bufEnd = bufEnd_ref[0]
        data_size = struct.calcsize(data_type)
        bufEnd_new = bufEnd + data_size
        if len(buffer[bufEnd:bufEnd_new]) == data_size:
            bufEnd_ref[0] = bufEnd_new
            return struct.unpack(f"<{data_type}", buffer[bufEnd:bufEnd_new])[0]
        else:
            raise ValueError(f"Buffer too small for {data_type} - buffer len {len(buffer)} - start {bufEnd} - end {bufEnd_new}")
    except Exception as e:
        logging.exception(f"Failed to unpack buffer - data type={data_type} - data size={data_size} - buffer len={len(buffer)} - start={bufEnd} - end={bufEnd_new}")
        logging.error(f"Buffer content: {buffer.hex()}")
        raise

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

    bufEnd_ref = [0]
    # unsigned long nodeID;
    nodeID = unpack_from_buffer(buffer, bufEnd_ref, 'b')
    if (nodeID < 0 or nodeID >= len(node_roots)):
        logging.info(f"ERROR - invalid node id received:{nodeID}")
        return

    # unsigned long payloadID;
    payloadID = unpack_from_buffer(buffer, bufEnd_ref, 'b')
    if (payloadID == 0):
        # float temp;
        temp = unpack_from_buffer(buffer, bufEnd_ref, 'f')

        # float voltage;
        voltage = unpack_from_buffer(buffer, bufEnd_ref, 'f')
        # convert mv to V
        if voltage > 0:
            voltage = voltage * 1.0 / 1000.0

        # float humidity;
        humidity = unpack_from_buffer(buffer, bufEnd_ref, 'f')

        TryPublish(node_roots[nodeID] +
                    "/Arduino/Voltage", voltage, qos, retain)

        # Check for sensor error (-127.0 indicates DS18B20 sensor error)
        if temp == -127.0:
            logging.warning("{} {} Sensor error detected - temperature reading: -127.0 (skipping temperature publish)".format(
                str(datetime.datetime.now()), pipe_number))
        elif (nodeID == 0 or nodeID == 2):
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

        logging.info("{} {} Data T:{:0.2f} V:{} H:{}%".format(
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
        # logging.info("buffer={}".format(vhex(buffer)))

        # unsigned long amb_als;
        amb_als = unpack_from_buffer(buffer, bufEnd_ref, 'I')

        # unsigned long amb_ir;
        amb_ir = unpack_from_buffer(buffer, bufEnd_ref, 'I')

        # float uv_index;
        uv_index = unpack_from_buffer(buffer, bufEnd_ref, 'f')
        uv_index = 1.0 * uv_index
        # https:#github.com/adafruit/Adafruit_SI1145_Library/blob/master/examples/si1145test/si1145test.ino
        uv_index = uv_index / 100.0; # the index is multiplied by 100 

        # float uv_index;
        readout_ms = unpack_from_buffer(buffer, bufEnd_ref, 'I')        
                            
        lux = calcLux(amb_als,amb_ir)
        TryPublish(node_roots[nodeID] +
                    "/GY1145/AL", amb_als, qos, retain)
        TryPublish(node_roots[nodeID] +
                    "/GY1145/IR", amb_ir, qos, retain)
        TryPublish(node_roots[nodeID] +
                    "/GY1145/UV", uv_index, qos, retain)      
        TryPublish(node_roots[nodeID] +
                    "/GY1145/Luminosity", lux, qos, retain)                                                
        logging.info("{} Data AmbALS:{} AmbIR:{} UV:{:0.2f} Lux:{} - {} ms".format(
            pipe_number, amb_als, amb_ir,uv_index,lux,readout_ms))

    elif payloadID == 3:
        # unsigned int lux; should be 4 bytes but is 2
        luxValue = unpack_from_buffer(buffer, bufEnd_ref, 'f')
                            
        TryPublish(node_roots[nodeID] +
                    "/TSL2261/LUX", luxValue, qos, retain)

        # float voltage;
        voltage = unpack_from_buffer(buffer, bufEnd_ref, 'f')
        # convert mv to V
        if voltage > 0:
            voltage = voltage * 1.0 / 1000.0

        TryPublish(node_roots[nodeID] +
                    "/Arduino/Voltage", voltage, qos, retain)

        logging.info("{} Data Lux:{} V:{}".format(pipe_number, luxValue,voltage))

    elif payloadID == 4:
        # unsigned int lux; should be 4 bytes but is 2
        analogValue = unpack_from_buffer(buffer, bufEnd_ref, 'h')
                            
        TryPublish(node_roots[nodeID] +
                    "/Arduino/Analog", analogValue, qos, retain)

        # float voltage;
        voltage = unpack_from_buffer(buffer, bufEnd_ref, 'f')
        # convert mv to V
        if voltage > 0:
            voltage = voltage * 1.0 / 1000.0

        TryPublish(node_roots[nodeID] +
                    "/Arduino/Voltage", voltage, qos, retain)

        logging.info("{} Data Analog:{} V:{}".format(
            pipe_number, analogValue,voltage))

    else:
        logging.info("invalid payload id:{}".format(payloadID))

    # debug


        # TryPublish(node_roots[nodeID] + "/TSL2561/Lux",luxMeasure,qos, retain)
        # TryPublish(node_roots[nodeID] + "/VEML6075/UVi",uv_index,qos, retain)
        # TryPublish(node_roots[nodeID] + "/VEML6075/UVa",uv_a,qos, retain)
        # TryPublish(node_roots[nodeID] + "/VEML6075/UVb",uv_b,qos, retain)
        # logging.info("Date={5} Temp={0:0.1f}C Humidity={1:0.1f}% Published={2} ret1={3} ret2={4}".format(temperature, humidity,published,ret1,ret2,datetime.datetime.now()))
    #client.loop_stop()
    # logging.info details about the received message
    logging.info(
        "{} Received {} bytes - node {} - payload {}".format(
            pipe_number, radio.payloadSize, nodeID, payloadID
        )
    )

    # start_timer = time.monotonic()  # reset the timeout timer


def listen(timeout=1198): #//20 minutes restart
    """Listen for any payloads and logging.info the transaction

    :param int timeout: The number of seconds to wait (with no transmission)
        until exiting function.
    """
    #logging.info("{} {} Data Lux:{} V:{}".format(str(datetime.datetime.now()), pipe_number, luxValue,voltage))
    logging.info(f"waiting for signal... radio payload:{radio.payloadSize}")
    radio.listen = True  # put radio in RX mode

    start_timer = time.monotonic()
    while (time.monotonic() - start_timer) < timeout:
        # process payload but do not check
        if (process_payload()):
            start_timer = time.monotonic()
            # Update message counts every 15 minutes
            update_message_counts()
    # Update message counts every 15 minutes
    update_message_counts()
    time.sleep(0.5)
    logging.info(f"No data received - Leaving RX role...")
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
    retries = -1
    success = False
    while (retries < 5 and success == False):
        ret = client.publish(topic, payload, qos, retain)
        ret.wait_for_publish()
        success = ret.rc == paho.MQTT_ERR_SUCCESS
        retries = retries + 1
    if (retries >= 1):
        logging.info("WARNING: {} retries for message - success: {}".format(retries, success))
    if (success):
        logging.info("Successfully published {}:{} retries: {}".format(
            topic, payload, retries))
    else:
        logging.info("Error publishing {}:{} retries: {}".format(topic, payload, retries))


if __name__ == "__main__":

    client = paho.Client(paho.CallbackAPIVersion.VERSION2)

    # connect mqtt broker
    client.connect("homeassistant.local", 1883, 60)

    client.loop_start()
    logging.info(f"connected to MQTT broker - CSN: {csn_pin} - CE: {ce_pin}")

    # client.on_publish = on_publish
    # initialize the nRF24L01 on the spi bus
    if not radio.begin():
        TryPublish(radio_status, "OFF", 2, True)
        raise RuntimeError("radio hardware is not responding")

    receiver_address = b"1Node"
    #logging.info("{} {} Data Lux:{} V:{}".format(str(datetime.datetime.now()), pipe_number, luxValue,voltage))
    logging.info(f"Receiver address is {receiver_address.hex()} - driver is {RF24_DRIVER} - radio connected: {radio.isChipConnected()}")
    # It is very helpful to think of an address as a path instead of as
    # an identifying device destination

    # to use different addresses on a pair of radios, we need a variable to
    # uniquely identify which address this radio will use to transmit
    # 0 uses address[0] to transmit, 1 uses address[1] to transmit

    # set the Power Amplifier level to -12 dBm since this test example is
    # usually run with nRF24L01 transceivers in close proximity of each other

    radio.set_pa_level(RF24_PA_HIGH, True)  # RF24_PA_MAX is default #RF24_PA_LOW
    #radio.set_pa_level(RF24_PA_LOW, True)  # RF24_PA_MAX is default #RF24_PA_LOW
    #radio.set_pa_level(RF24_PA_MAX, False)  # RF24_PA_MAX is default #RF24_PA_LOW
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
    # for debugging, we have 2 options that logging.info a large block of details
    # (smaller) function that logging.infos raw register values
    # radio.logging.infoDetails()
    # (larger) function that logging.infos human readable data
    # radio.logging.infoPrettyDetails()

    # on data ready test
    #Configuring IRQ pin to only ignore 'on data sent' event
    #radio.maskIRQ(True, False, False)  # args = tx_ds, tx_df, rx_dr
    radio.print_pretty_details()

    try:
        logging.info(f"hostname is {hostname}")
        # Load message counts from JSON file
        load_message_counts_from_json(msg_count_status_file)
        TryPublish(radio_status, "ON", 2, True)
        update_message_counts()        
        listen()
        client.loop_stop()
        save_message_counts_to_json(msg_count_status_file)
    except KeyboardInterrupt:
        logging.warning("Keyboard Interrupt detected. Exiting...")
        radio.powerDown()
        client.loop_stop()
        sys.exit()
