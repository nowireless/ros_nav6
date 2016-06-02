import math
import numpy as np
from suitcase.structure import Structure
from suitcase.fields import BaseField
from suitcase.fields import Magic
from suitcase.fields import CRCField
from suitcase.fields import SLInt8
from suitcase.fields import ULInt8

PACKET_START_CHAR = "!"
PROTOCOL_FLOAT_LENGTH = 7
PROTOCOL_UINT16_LENGTH = 4
CHECKSUM_LENGTH = 2
TERMINATOR_LENGTH = 2

MPU_INIT_FAILURE = "mpu_init failed!"
MPU_FIRMWARE_LOAD_ERROR = "Firmware Load ERROR"

UPDATE_RATE_HZ = 100

MSGID_YPR_UPDATE = 'y'
YPR_UPDATE_MESSAGE_LENGTH = 34
YPR_UPDATE_YAW_VALUE_INDEX = 2
YPR_UPDATE_PITCH_VALUE_INDEX = 9
YPR_UPDATE_ROLL_VALUE_INDEX = 16
YPR_UPDATE_COMPASS_VALUE_INDEX = 23
YPR_UPDATE_CHECKSUM_INDEX = 30
YPR_UPDATE_TERMINATOR_INDEX = 32

MSGID_QUATERNION_UPDATE = 'q'
QUATERNION_UPDATE_MESSAGE_LENGTH = 53
QUATERNION_UPDATE_QUAT1_VALUE_INDEX = 2
QUATERNION_UPDATE_QUAT2_VALUE_INDEX = 6
QUATERNION_UPDATE_QUAT3_VALUE_INDEX = 10
QUATERNION_UPDATE_QUAT4_VALUE_INDEX = 14
QUATERNION_UPDATE_ACCEL_X_VALUE_INDEX = 18
QUATERNION_UPDATE_ACCEL_Y_VALUE_INDEX = 22
QUATERNION_UPDATE_ACCEL_Z_VALUE_INDEX = 26
QUATERNION_UPDATE_MAG_X_VALUE_INDEX = 30
QUATERNION_UPDATE_MAG_Y_VALUE_INDEX = 34
QUATERNION_UPDATE_MAG_Z_VALUE_INDEX = 38
QUATERNION_UPDATE_TEMP_VALUE_INDEX = 42
QUATERNION_UPDATE_CHECKSUM_INDEX = 49
QUATERNION_UPDATE_TERMINATOR_INDEX = 51

MSGID_GYRO_UPDATE = 'g'
GYRO_UPDATE_MESSAGE_LENGTH = 46
GYRO_UPDATE_GYRO_X_VALUE_INDEX = 2
GYRO_UPDATE_GYRO_Y_VALUE_INDEX = 6
GYRO_UPDATE_GYRO_Z_VALUE_INDEX = 10
GYRO_UPDATE_ACCEL_X_VALUE_INDEX = 14
GYRO_UPDATE_ACCEL_Y_VALUE_INDEX = 18
GYRO_UPDATE_ACCEL_Z_VALUE_INDEX = 22
GYRO_UPDATE_MAG_X_VALUE_INDEX = 26
GYRO_UPDATE_MAG_Y_VALUE_INDEX = 30
GYRO_UPDATE_MAG_Z_VALUE_INDEX = 34
GYRO_UPDATE_TEMP_VALUE_INDEX = 38
GYRO_UPDATE_CHECKSUM_INDEX = 42
GYRO_UPDATE_TERMINATOR_INDEX = 44

MSGID_STREAM_CMD = 'S'
STREAM_CMD_MESSAGE_LENGTH = 9
STREAM_CMD_STREAM_TYPE_YPR = MSGID_YPR_UPDATE
STREAM_CMD_STREAM_TYPE_QUATERNION = MSGID_QUATERNION_UPDATE
STREAM_CMD_STREAM_TYPE_GYRO = MSGID_GYRO_UPDATE
STREAM_CMD_STREAM_TYPE_INDEX = 2
STREAM_CMD_UPDATE_RATE_HZ_INDEX = 3
STREAM_CMD_CHECKSUM_INDEX = 5
STREAM_CMD_TERMINATOR_INDEX = 7

MSG_ID_STREAM_RESPONSE = 's'
STREAM_RESPONSE_MESSAGE_LENGTH = 46
STREAM_RESPONSE_STREAM_TYPE_INDEX = 2
STREAM_RESPONSE_GYRO_FULL_SCALE_DPS_RANGE = 3
STREAM_RESPONSE_ACCEL_FULL_SCALE_G_RANGE = 7
STREAM_RESPONSE_UPDATE_RATE_HZ = 11
STREAM_RESPONSE_YAW_OFFSET_DEGREES = 15
STREAM_RESPONSE_QUAT1_OFFSET = 22
STREAM_RESPONSE_QUAT2_OFFSET = 26
STREAM_RESPONSE_QUAT3_OFFSET = 30
STREAM_RESPONSE_QUAT4_OFFSET = 34
STREAM_RESPONSE_FLAGS = 38
STREAM_RESPONSE_CHECKSUM_INDEX = 42
STREAM_RESPONSE_TERMINATOR_INDEX = 44


def valid_message(data):
    if type(data) != str:
        return False
    if len(data) < 2:
        return False
    if data[0] != PACKET_START_CHAR:
        return False

    if data[1] is MSGID_YPR_UPDATE:
        return len(data) == YPR_UPDATE_MESSAGE_LENGTH
    elif data[1] is MSGID_QUATERNION_UPDATE:
        return len(data) == QUATERNION_UPDATE_MESSAGE_LENGTH
    elif data[1] is MSGID_GYRO_UPDATE:
        return len(data) == GYRO_UPDATE_MESSAGE_LENGTH
    elif data[1] is MSGID_STREAM_CMD:
        return len(data) == STREAM_CMD_MESSAGE_LENGTH
    elif data[1] is MSG_ID_STREAM_RESPONSE:
        return len(data) == STREAM_RESPONSE_MESSAGE_LENGTH

    return False


def checksum(data):
    ret = 0
    for char in data:
        ret += ord(char)
    return "%02X" % (ret % 256)


class ProtocolChecksum(BaseField):
    def __init__(self, **kwargs):
        BaseField.__init__(self, **kwargs)
        self.bytes_required = 2

    def pack(self, stream):
        if type(self._value) is not str:
            raise RuntimeError, "Not a string"
        stream.write(self._value)

    def unpack(self, data):
        self._value = str(data)


class ProtocolUInt16(BaseField):
    def __init__(self, **kwargs):
        BaseField.__init__(self, **kwargs)
        self.bytes_required = PROTOCOL_UINT16_LENGTH

    def pack(self, stream):
        if self._value is not None:
            stream.write("%04X" % np.uint16(self._value))
        else:
            stream.write("0000")

    def unpack(self, data):
        # Expecting a hexadecimal string

        self._value = np.uint16(int(data, 16))


class ProtocolSInt16(BaseField):
    def __init__(self, **kwargs):
        BaseField.__init__(self, **kwargs)
        self.bytes_required = PROTOCOL_UINT16_LENGTH

    def pack(self, stream):
        if self._value is not None:
            stream.write("%04X" % np.uint16(self._value))
        else:
            stream.write("0000")

    def unpack(self, data):
        # Expecting a hexadecimal string
        self._value = np.int16(int(data, 16))


class ProtocolUInt8(BaseField):
    def __init__(self, **kwargs):
        BaseField.__init__(self, **kwargs)
        self.bytes_required = 1

    def pack(self, stream):
        if self._value is not None:
            stream.write("%02X" % np.uint8(self._value))
        else:
            stream.write("00")

    def unpack(self, data):
        self._value = np.int8(ord(data))


class ProtocolFloat(BaseField):
    def __init__(self, **kwargs):
        BaseField.__init__(self, **kwargs)
        self.bytes_required = PROTOCOL_FLOAT_LENGTH

    def pack(self, stream):
        if self._value is not None:
            f = float(self._value)
            val = ""
            if f < 0:
                val += "-"
            else:
                val += " "

            f = abs(f)
            val += "%03d." % f
            val += "%02d" % int(round((f - math.floor(f)) * 100))
            stream.write(val)
        else:
            stream.write(" 000.00")

    def unpack(self, data):
        #print data
        self._value = float(data)


class YPRUpdate(Structure):
    start = Magic("!")
    type = Magic("y")
    yaw = ProtocolFloat()
    pitch = ProtocolFloat()
    roll = ProtocolFloat()
    compass_heading = ProtocolFloat()
    checksum = CRCField(ProtocolChecksum(), checksum, 0, -4)
    termination = Magic("\r\n")


class QuaternionUpdate(Structure):
    start = Magic("!")
    type = Magic("q")
    q1 = ProtocolUInt16()
    q2 = ProtocolUInt16()
    q3 = ProtocolUInt16()
    q4 = ProtocolUInt16()
    accel_x = ProtocolSInt16()
    accel_y = ProtocolSInt16()
    accel_z = ProtocolSInt16()
    mag_x = ProtocolSInt16()
    mag_y = ProtocolSInt16()
    mag_z = ProtocolSInt16()
    temp_c = ProtocolFloat()
    checksum = CRCField(ProtocolChecksum(), checksum, 0, -4)
    termination = Magic("\r\n")


class StreamCommand(Structure):
    start = Magic("!")
    type = Magic(MSGID_STREAM_CMD)
    stream_type = ULInt8()
    update_rate_hz = ProtocolUInt8()
    checksum = CRCField(ProtocolChecksum(), checksum, 0, -4)
    termination = Magic("\r\n")


class StreamResponse(Structure):
    start = Magic("!")
    type = Magic("s")

    stream_type = ProtocolUInt8()

    gyro_fsr_dps = ProtocolUInt16()
    accel_fsr_g = ProtocolUInt16()
    update_rate_hz = ProtocolUInt16()
    yaw_offset_degrees = ProtocolFloat()
    q1_offset = ProtocolUInt16()
    q2_offset = ProtocolUInt16()
    q3_offset = ProtocolUInt16()
    q4_offset = ProtocolUInt16()
    flags = ProtocolUInt16()

    checksum = CRCField(ProtocolChecksum(), checksum, 0, -4)
    termination = Magic("\r\n")


def is_stream_response(data):
    return len(data) >= 2 and data[1] == MSG_ID_STREAM_RESPONSE


def is_ypr_update(data):
    return len(data) >= 2 and data[1] == MSGID_YPR_UPDATE


def is_quaternion_update(data):
    return len(data) >= 2 and data[1] == MSGID_QUATERNION_UPDATE


def is_gyro_update(data):
    return len(data) >= 2 and data[1] == MSGID_GYRO_UPDATE


def is_mpu_init_failure(data):
    if MPU_INIT_FAILURE in data:
        return True
    elif MPU_FIRMWARE_LOAD_ERROR in data:
        return True
    return False


def make_quaternion_cmd_packet():
    ret = StreamCommand()
    ret.update_rate_hz = UPDATE_RATE_HZ
    ret.stream_type = ord(MSGID_QUATERNION_UPDATE)
    return ret


def fix_quaternion(q):
    q2 = map(lambda x:  x/16384.0, q)
    for i in xrange(0, 3):
        if q2[i] >= 2:
            q2[i] -= 4

if __name__ == "__main__":
    # y = YPRUpdate()
    # y.yaw = -222.11
    # y.pitch = 1
    # y.roll = 2
    # y.compas_heading = 3

    # print y.pack()
    # print float(" 052.00")
    # ypr = YPRUpdate.from_data("!y-000.09 006.18-005.11 147.26DF\r\n")
    # print y.yaw
    # print (make_quaternion_cmd_packet().pack())
    # q = QuaternionUpdate.from_data("!q22C8DF8A2AC8FE90D2DADAB2E5D6FEDD021F004A 023.554C\r\n")
    # print "22C8", int("22C8", 16), np.uint16(int("22C8", 16))
    # print q.temp_c

    s = StreamResponse.from_data("!sq07D000020064-081.38228BDF782AEAFE80000250\r\n")
    print s
