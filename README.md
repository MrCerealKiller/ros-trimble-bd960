# ROS Trimble BD960 Driver

## Description
This is a simple driver that converts the NMEA strings provided by the Trimble
BD960 board to be compatible with standard ros messages. Currently, it provides
lattitude, longitude, altitude, and heading (along with a diagonal positional
covariance matrix).

## Supported Sentences
Currently supported NMEA sentence types are:
* GGA
* GST
* HDT

_Support for more sentences may be added in the future_

## Dependencies
* ROS
* PySerial (Python Serial)

## Usage
1. Setup your GPS receiver to run on a serial port using one of the Trimble
configuration utilities. (Ensure that the sentences you enable are supported.)
2. Use the config file (`config/gps.yaml`) to choose your port, baudrate, and
timeout. Your timeout should correspond roughly to the configured frequency.
3. Launch the node using `roslaunch ros_trimble_bd960 gps.launch` and the
driver will begin to output messages on the topics.

## Topics
* **/gps/fix [NavSatFix]** : Status, position, and covariance data
* **/gps/heading [Float64]** : Heading in degrees from True North

## License
Copyright (c) 2019 Jeremy Mallette under MIT License
