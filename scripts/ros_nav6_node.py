#!/usr/bin/python

import rospy
import select
import sys
import time

import protocol

from serial import Serial
from serial import SerialException

from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu


def publish(port):
    pub = rospy.Publisher('imu', Imu, queue_size=10)
    rate = rospy.Rate(20)
    rospy.loginfo("Sending Quaternion CMD")
    cmd = protocol.make_quaternion_stream_packet()
    port.flush()
    port.write(cmd.pack())
    response = False
    last = time.time()
    while not rospy.is_shutdown():
        data = port.readline()
        print data

        if not response:
            if protocol.is_stream_response(data):
                rospy.loginfo("Received Stream Response")
                response = True
            else:
                rospy.loginfo("Waiting for Stream Response")
                if (time.time() - last) > 3:
                    last = time.time()
                    rospy.loginfo("Sending Stream CMD")
                    cmd = protocol.make_quaternion_stream_packet()
                    port.flush()
                    port.write(cmd.pack())
        else:
            if protocol.is_stream_response(data):
                rospy.loginfo("Received Stream Response")
            elif protocol.is_ypr_update(data):
                rospy.loginfo("Received YPR Update")
            elif protocol.is_mpu_init_failure(data):
                rospy.logfatal("MPU Init failure")
            else:
                rospy.logwarn("Invalid message received: %s", data)
                continue
        #update = protocol.YPRUpdate.from_data(data)
        #print data

if __name__ == "__main__":
    rospy.init_node('nav6', anonymous=True)
    port = Serial()
    try:
        print "1"
        port.baudrate = rospy.get_param("buadrate", 57600)
        port.port = rospy.get_param('port', '/dev/ttyACM1')
        port.timeout = rospy.get_param('timeout', 5)
        port.open()
    except SerialException as e:
        rospy.logfatal("Not able to connect to serial port")
        port.close()
        sys.exit(-1)

    frame_name = rospy.get_param('frame name', 'odom_frame')
    try:
        publish(port)
    except (rospy.ROSInterruptException, select.error):
        port.close()
        sys.exit(0)


