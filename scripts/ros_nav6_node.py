#!/usr/bin/python

import rospy
import select
import sys
import time
import math

import protocol
import quaternion

from serial import Serial
from serial import SerialException
from suitcase.exceptions import SuitcaseParseError

from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from sensor_msgs.msg import Temperature
from std_msgs.msg import Bool


class Nav6Node:
    def __init__(self):
        # Setup ros
        rospy.init_node('nav6', anonymous=True)
        self.frame_name = "imu"
        self.imu_pub = rospy.Publisher("/imu", Imu, queue_size=10)
        self.imu_temp_pub = rospy.Publisher("/imu_tempature", Temperature, queue_size= 10)
        self.mag_pub = rospy.Publisher("/imu_compas", MagneticField, queue_size=10)
        self.cal_pub = rospy.Publisher("/imu_calibrated", Bool, queue_size=10)

        # Setup Serial Port
        self.port = Serial()

        # Get Params from ros for serial port
        self.port.baudrate = rospy.get_param("buadrate", 57600)
        self.port.port = rospy.get_param('port', '/dev/ttyACM0')
        self.port.timeout = rospy.get_param('timeout', 5)

        self.calibrated = False;

        self.accel_fsr_g = None
        self.last_ypr = None
        self.imu_msg = Imu()
        self.mag_msg = MagneticField()
        self.imu_temp_msg = Temperature()

    def enable_quaternion_mode(self):
        rospy.loginfo("Enabling Quaternion Mode")

        rospy.loginfo("Resetting Arduino")
        self.port.setDTR(False)
        time.sleep(1)

        # https://stackoverflow.com/questions/21073086/wait-on-arduino-auto-reset-using-pyserial
        # toss any data already received, see
        # http://pyserial.sourceforge.net/pyserial_api.html#serial.Serial.flushInput
        self.port.flushInput()
        self.port.setDTR(True)
        rospy.loginfo("Arduino has been reset")

        # Wait for the arduino to comeup
        time.sleep(1)
        self.port.flush()

        cmd = protocol.make_quaternion_cmd_packet()
        self.port.write(cmd.pack())

        good = False
        quaternion_mode = False
        last = time.time()
        while not (good and quaternion_mode):
            data = self.port.readline()
            if len(data) == 0:
                rospy.logwarn("Empty response")
                continue
            if protocol.is_stream_response(data):
                rospy.loginfo("Received Stream Response")
                self.handle_stream_response(data)
                rospy.loginfo("accel_fsr_g: %i", self.accel_fsr_g)
                good = True
            elif protocol.is_quaternion_update(data):
                rospy.loginfo("Received Quaternion update")
                quaternion_mode = True
            elif protocol.is_mpu_init_failure(data):
                rospy.logfatal("MPU init failure")

            if (time.time()-last) > 3:
                rospy.loginfo("Resending Quaternion CMD")
                cmd = protocol.make_quaternion_cmd_packet()
                self.port.write(cmd.pack())
                last = time.time()

            self.pub_cal_status()

        rospy.loginfo("Now in Quaternion Mode")

    def publish(self):
        rate = rospy.Rate(150)
        try:
            self.port.open()
        except SerialException as e:
            rospy.logfatal("Could not open serial port %s, reason: %s", self.port.port, e)
            sys.exit(-2)

        try:
            try:
                self.enable_quaternion_mode()

                while not rospy.is_shutdown():
                    data = self.port.readline();
                    rospy.logdebug(data.replace("\r\n", ""))

                    if len(data) == 0:
                        rospy.logwarn("No response")
                        continue

                    if protocol.is_stream_response(data):
                        rospy.loginfo("Received Stream Response")
                        self.handle_stream_response(data)
                    elif protocol.is_quaternion_update(data):
                        rospy.logdebug("Received Quaternion Update")
                        self.handle_quaternion_update(data)
                    elif protocol.is_mpu_init_failure(data):
                        # Is this case needed?
                        rospy.logfatal("MPU Init failure")
                    else:
                        rospy.logwarn("Invalid message received: %s", data)
                        continue

                    self.pub_cal_status();
            except SerialException as e:
                rospy.logfatal("Serial Exception: %s", e)
                sys.exit(-3)
        except (rospy.ROSInterruptException, select.error) as e:
            rospy.loginfo("Exiting")
            self.port.close()
            sys.exit(0)

    def handle_quaternion_update(self, data):
        if not self.calibrated:
            rospy.loginfo("Waiting for calibration")
            return
        try:
            #print data
            update = protocol.QuaternionUpdate.from_data(data)
            q, g, ypr, linear_accel = quaternion.handle_quaternion(update)

            if self.last_ypr is None:
                self.last_ypr = ypr
                return

            #angular_vel = quaternion.calculate_angular_velocity(ypr, self.last_ypr)
            angular_vel = [0,0,0]
            
            self.imu_msg.header.frame_id = self.frame_name
            self.imu_msg.header.stamp = rospy.Time.now()
            self.imu_msg.orientation.w = q[0]
            self.imu_msg.orientation.x = q[1]
            self.imu_msg.orientation.y = q[2]
            self.imu_msg.orientation.z = q[3]

            self.imu_msg.linear_acceleration.x = linear_accel[0]
            self.imu_msg.linear_acceleration.y = linear_accel[1]
            self.imu_msg.linear_acceleration.z = linear_accel[2]
            self.imu_msg.linear_acceleration_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]

            self.imu_msg.angular_velocity.x = angular_vel[0]
            self.imu_msg.angular_velocity.y = angular_vel[1]
            self.imu_msg.angular_velocity.z = angular_vel[2]
            self.imu_msg.angular_velocity_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]
            self.imu_pub.publish(self.imu_msg)

            # Magnetic Field
            m_x = update.mag_x * math.cos(ypr[1]) + update.mag_z*math.cos(ypr[1])
            m_y = update.mag_x * math.sin(ypr[2]) * math.sin(ypr[1]) + update.mag_y * math.cos(ypr[2]) - update.mag_z * math.sin(ypr[2]) * math.cos(ypr[1])

            self.mag_msg.header.stamp = rospy.Time.now()
            self.mag_msg.header.frame_id = self.frame_name
            self.mag_msg.magnetic_field.x = m_x
            self.mag_msg.magnetic_field.y = m_y
            self.mag_msg.magnetic_field.z = 0
            self.mag_msg.magnetic_field_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]
            self.mag_pub.publish(self.mag_msg)
            # Imu Temp in C
            temp_c = update.temp_c
            self.imu_temp_msg.header.stamp = rospy.Time.now()
            self.imu_temp_msg.header.frame_id = self.frame_name
            self.imu_temp_msg.temperature = temp_c
            self.imu_temp_pub.publish(self.imu_temp_msg)
        except SuitcaseParseError as e:
            rospy.logwarn("Could not parse: %s", e)

    def handle_stream_response(self, data):
        try:
            response = protocol.StreamResponse.from_data(data)
            self.accel_fsr_g = response.accel_fsr_g
            rospy.loginfo("Stream Response flags: %i", response.flags)
            if response.flags == 2:
                rospy.loginfo("IMU Calibrated")
                self.calibrated = True
        except SuitcaseParseError as e:
            rospy.logwarn("Could not parse: %s", e)

    def pub_cal_status(self):
        msg = Bool()
        msg.data = self.calibrated
        self.cal_pub.publish(msg)

if __name__ == "__main__":
    node = Nav6Node()
    node.publish()


