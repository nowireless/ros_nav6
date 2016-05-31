#!/usr/bin/python

import rospy
import sys

import protocol

from serial import Serial
from serial import SerialException

from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu


def publish(port):
    rospy.init_node('heading', anonymous=True)
    pub = rospy.Publisher('imu', Imu, queue_size=0)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        data = port.readline()
        print data
        if not protocol.valid_message(data):
            rospy.logwarn("Invalid message received")
            continue
        #update = protocol.YPRUpdate.from_data(data)
        #print data

if __name__ == "__main__":
    try:
        port = Serial()
        port.baudrate = rospy.get_param("buadrate", 57600)
        port.port = rospy.get_param('port', '/dev/ttyACM0')
        port.timeout = rospy.get_param('timeout', 5)
        port.open()
    except SerialException as e:
        rospy.logfatal("Not able to connect to serial port")
        sys.exit(-1)

    frame_name = rospy.get_param('frame name', 'odom_frame')
    try:
        publish(port)
    except rospy.ROSInterruptException:
        sys.exit(0)


