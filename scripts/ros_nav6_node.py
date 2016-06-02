#!/usr/bin/python

import rospy
import select
import sys
import time

import protocol

from serial import Serial
from serial import SerialException
from suitcase.exceptions import SuitcaseParseError

from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField


class Nav6Node:
    def __init__(self):
        # Setup ros
        rospy.init_node('nav6', anonymous=True)
        frame_name = rospy.get_param('frame name', 'odom_frame')
        self.pub = rospy.Publisher(frame_name, Imu, queue_size=10)

        # Setup Serial Port
        self.port = Serial()

        # Get Params from ros for serial port
        self.port.baudrate = rospy.get_param("buadrate", 57600)
        self.port.port = rospy.get_param('port', '/dev/ttyACM0')
        self.port.timeout = rospy.get_param('timeout', 5)

        self.accel_fsr_g = None

        self.imu_msg = Imu()

    def enable_quaternion_mode(self):
        rospy.loginfo("Enabling Quaternion Mode")

        rospy.loginfo("Waiting for Arduino")
        time.sleep(3) # Don't like this
        rospy.loginfo("Done Waiting for Arduino")

        self.port.flush()

        cmd = protocol.make_quaternion_cmd_packet()
        self.port.write(cmd.pack())

        good = False
        last = time.time()
        while not good:
            data = self.port.readline()
            if len(data) == 0:
                rospy.logwarn("Empty response")
                continue
            if protocol.is_stream_response(data):
                rospy.loginfo("Received Stream Response")
                self.handle_stream_response(data)
                rospy.loginfo("accel_fsr_g: %i", self.accel_fsr_g)
                good = True
            elif protocol.is_mpu_init_failure(data):
                rospy.logfatal("MPU init failure")
                sys.exit(-4)

            if (time.time()-last) > 3:
                rospy.loginfo("Resending Quaternion CMD")
                cmd = protocol.make_quaternion_cmd_packet()
                self.port.write(cmd.pack())
                last = time.time()

    rospy.loginfo("Now in Quaternion Mode")

    def publish(self):
        rate = rospy.Rate(150)
        try:
            self.port.open()
        except SerialException as e:
            rospy.logfatal("Could not open serial port %s, reason: %s", self.port.port, e)
            sys.exit(-2)

        # Not working at the moment
        # cmd = protocol.make_quaternion_stream_packet()
        # self.port.flush()
        # self.port.write(cmd.pack())
        try:
            try:
                self.enable_quaternion_mode()

                while not rospy.is_shutdown():
                    data = self.port.readline();
                    rospy.loginfo(data.replace("\r\n", ""))

                    if len(data) == 0:
                        rospy.logwarn("No response")
                        continue

                    if protocol.is_stream_response(data):
                        rospy.loginfo("Received Stream Response")
                    elif protocol.is_quaternion_update(data):
                        rospy.logdebug("Received Quaternion Update")
                        self.handle_quaternion_update(data)
                    elif protocol.is_mpu_init_failure(data):
                        rospy.logfatal("MPU Init failure")
                    else:
                        rospy.logwarn("Invalid message received: %s", data)
                        continue
            except SerialException as e:
                rospy.logfatal("Serial Exception: %s", e)
                sys.exit(-3)
        except (rospy.ROSInterruptException, select.error) as e:
            rospy.loginfo("Exiting")
            self.port.close()
            sys.exit(0)

    def handle_quaternion_update(self, data):
        try:
            q_update = protocol.QuaternionUpdate.from_data(data)

            # Fix Quaternion Values
            q = [q_update.q1, q_update.q2, q_update.q3, q_update.q4]
            protocol.fix_quaternion(q)

            # Fix linear acceleration

        except SuitcaseParseError as e:
            rospy.logwarn("Could not parse: %s", e)

    def handle_stream_response(self, data):
        try:
            response = protocol.StreamResponse.from_data(data)
            self.accel_fsr_g = response.accel_fsr_g
        except SuitcaseParseError as e:
            rospy.logwarn("Could not parse: %s", e)

if __name__ == "__main__":
    node = Nav6Node()
    node.publish()


