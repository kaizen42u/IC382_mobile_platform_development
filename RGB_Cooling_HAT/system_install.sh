# Author: VincentChan

sudo apt update
sudo apt upgrade -y
sudo apt install -y i2c-tools python3-pip
sudo pip3 install smbus

sudo python3 -m pip install --upgrade pip setuptools wheel
sudo pip install Adafruit-SSD1306
sudo apt-get install python3-rpi.gpio
sudo apt-get --reinstall install libraspberrypi-bin

# Start scanning I2C
sudo i2cdetect -y 1
