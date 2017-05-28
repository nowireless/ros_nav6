import protocol
import math
import numpy as np
'''
!q3EFEFA44FF68F64602B6F5763B6CFE97FF99FE19 026.1049
0.9842529 -0.08959961 -0.009277344 -0.15197754
!q3EFEFA44FF68F64602B4F55C3BA6FE97FF99FE19 026.1050
0.9842529 -0.08959961 -0.009277344 -0.15197754
!q3EFEFA44FF68F64602BEF58A3B52FE97FF99FE19 026.1052
0.9842529 -0.08959961 -0.009277344 -0.15197754

'''
accel_fsr_g = 2
LITTLE_G = 9.81 # m/s^2
yaw_offset = 0


def handle_quaternion(update):
    # Calculate Quaternions
    q = [0, 0, 0, 0]
    q[0] = update.q1 / 16384.0
    q[1] = update.q2 / 16384.0
    q[2] = update.q3 / 16384.0
    q[3] = update.q4 / 16384.0

    for i in xrange(0, 4):
        if q[i] >= 2:
            q[i] -= 4
    # print "Quaternion:", q

    # Calculate the gravity vector
    g = [0, 0, 0]
    g[0] = 2 * (q[1]*q[3] - q[0]*q[2])
    g[1] = 2 * (q[0]*q[1] + q[2]*q[3])
    g[2] = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3]
    # print "Gravity:", g

    # YPR in radians
    ypr = [0, 0, 0]

    ypr[0] = math.atan2(2*q[1]*q[2] - 2*q[0]*q[3], 2*q[0]*q[0] + 2*q[1]*q[1] - 1)
    ypr[1] = math.atan(g[0] / math.sqrt(g[1]**2 + g[2]**2))
    ypr[2] = math.atan(g[1] / math.sqrt(g[0]**2 + g[2]**2))

    # print "YPR:", ypr

    # Calculate acceleration
    a = [update.accel_x, update.accel_y, update.accel_z]
    a = map(lambda x: float(x) / (32768.0 / accel_fsr_g), a)
    # print "Raw Acceleration:", a

    # Calculate Linear Acceleration with gravoty removed
    linear_accel = map(lambda x: x*LITTLE_G, [a[0] - g[0], a[1] - g[1], a[2] - g[2]])
    # print "Linear Acceleration:", linear_accel
    return q, g, ypr, linear_accel


def calculate_angular_velocity(now, last, dt):
    assert len(now) == 3
    assert len(last) == 3
    assert dt == 0.0
    return np.divide(np.subtract(now, last), dt) # in Rad/Sec


if __name__ == "__main__":
    last_update = protocol.QuaternionUpdate.from_data("!q3EFEFA44FF68F64602B4F55C3BA6FE97FF99FE19 026.1050\r\n")
    last = handle_quaternion(last_update)
    now_update = protocol.QuaternionUpdate.from_data("!q3EFEFA44FF68F64602BEF58A3B52FE97FF99FE19 026.1052\r\n")
    now = handle_quaternion(now_update)

    print calculate_angular_velocity(now[2], last[2], 1.0/100.0)
