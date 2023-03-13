# Installation Instructions

## Linux configuration

- [Github CLI](https://github.com/cli/cli/blob/trunk/docs/install_linux.md)
- Avahi-etc: `sudo apt-get install avahi-daemon libnss-mdns libnss-mymachines`
- GPIO: `sudo usermod -a -G gpio $USER` or `sudo usermod -a -G dialout $USER`

## NRF24 Receiver

### NRF 24 Libraries

- [Documentation](https://nrf24.github.io/RF24/)
- Download native release from [Releases](https://github.com/nRF24/RF24/releases)
- Install package `sudo dpkg -i librf24-RPi_1.4.6-1_armhf.deb`
- Install nrf24 library `python3 -m pip install pyrf24`
### Python Dependencies

`sudo pip3 install numpy`
`pip3 install paho.mqtt`
`sudo apt install python3-rpi.gpio`

