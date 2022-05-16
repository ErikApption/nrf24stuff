#!/bin/sh
path=`pwd`
sudo ln -s $path/NRFReceiver.service /etc/systemd/system
sudo systemctl daemon-reload
