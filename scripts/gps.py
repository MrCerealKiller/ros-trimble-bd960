#!/usr/bin/env python

import rospy
from ros-trimble-bd960 import TrimbleBD960

if __name__ == "__main__":
    rospy.init_node('trimble-gps')

    port = rospy.get_param('port', '/dev/ttyUSB0')
    baudrate = rospy.get_param('baudrate', '34800')
    timeout = rospy.get_param('timeout', '1')

    with TrimbleBD960(port, baudrate, timeout) as gps:
        try:
            gps.nmea_stream()
        except KeyboardInterrupt:
            gps.preempt()
