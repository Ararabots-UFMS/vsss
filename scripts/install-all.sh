#!/bin/bash
sudo apt install libbluetooth-dev -y
sudo apt install qv4l2 -y

sudo ./install-ros.sh
sudo ./install-fltk.sh
sudo ./install-opencv.sh

cd ..
./setup.py
pip3 install -r requirements.txt