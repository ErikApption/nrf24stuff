# nrf24stuff

## Create virtual environment

```bash
python -m venv venv
```

## Setup packages

```bash
. venv/bin/activate
pip install -r requirements.txt
```

## Linux configuration

- [Github CLI](https://github.com/cli/cli/blob/trunk/docs/install_linux.md)
- Avahi-etc: `sudo apt-get install avahi-daemon libnss-mdns libnss-mymachines`
- GPIO: `sudo usermod -a -G gpio $USER` or `sudo usermod -a -G dialout $USER`
- `sudo usermod -a -G gpio $USER`

## NRF24 Receiver

### NRF 24 Libraries

- [Documentation](https://nrf24.github.io/RF24/)
- Download native release from [Releases](https://github.com/nRF24/RF24/releases)
- Install package `sudo dpkg -i librf24-RPi_1.4.6-1_armhf.deb`
- CMake `sudo apt install cmake`
- Install nrf24 library `python3 -m pip install pyrf24`
### Python Dependencies

`pip3 install paho.mqtt numpy`
`sudo apt install python3-rpi.gpio`

## Bedroom Receiver

`python3 -m pip install homeassistant-mqtt-binding adafruit-circuitpython-bmp3xx adafruit-circuitpython-sgp30`

## Study AM2301

`python3 -m pip install homeassistant-mqtt-binding adafruit-circuitpython-bmp3xx adafruit-circuitpython-sgp30 adafruit-circuitpython-dht`