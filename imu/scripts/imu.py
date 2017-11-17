#!/usr/bin/env python
import rospy, tf
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from ekf import EKF
import numpy as np


ekf_imu = EKF()
ekf_init = False
x = None
S = None
last_time = None


def callback(data):
    global ekf_init, x, S, last_time
    gyro_dir = np.array([1, 1, 1])
    acc_dir = np.array([-1, -1, -1])


    d = np.array(data.data)
    cnt = d[0] 
    acc = d[1:4] * acc_dir / 16384.0
    gyro = d[4:7] * gyro_dir / 131.0 * np.pi / 180.0

    cur_time = rospy.Time.now()
    if last_time == None:
        last_time = cur_time
    else:
        dt = (cur_time - last_time).to_sec()
        # rospy.loginfo(dt)
    
    if not ekf_init: 
        r = ekf_imu.init_filter(gyro, acc)
        if r is not None:
            x, S = r
            ekf_init = True
    else:
        x, S = ekf_imu.filter(x, S, gyro, acc, dt)

    if ekf_init == True:
        br = tf.TransformBroadcaster()
        br.sendTransform((0, 0, 0),
                         (x[1][0], x[2][0], x[3][0], x[0][0]),
                         cur_time,
                         "IMU",
                         "world")

    pub = rospy.Publisher('gyro-x', Float32, queue_size=10)
    pub.publish(gyro[0])
    pub = rospy.Publisher('gyro-y', Float32, queue_size=10)
    pub.publish(gyro[1])
    pub = rospy.Publisher('gyro-z', Float32, queue_size=10)
    pub.publish(gyro[2])

    pub = rospy.Publisher('acc-x', Float32, queue_size=10)
    pub.publish(acc[0])
    pub = rospy.Publisher('acc-y', Float32, queue_size=10)
    pub.publish(acc[1])
    pub = rospy.Publisher('acc-z', Float32, queue_size=10)
    pub.publish(acc[2])

    # gravity in body frame
    g_b = -np.array([2 * (x[1][0] * x[3][0] + x[0][0] * x[2][0]),
                     2 * (x[2][0] * x[3][0] - x[0][0] * x[1][0]),
                     1 - 2 * (x[1][0] ** 2 + x[2][0] ** 2)])
    # z axis in world fram
    z_w = np.array([0, 0, -1])
    tilt = np.arccos(np.sum(g_b * z_w))
    axis = np.cross(z_w, g_b)
    print axis

    pub = rospy.Publisher('tilt', Float32, queue_size=10)
    pub.publish(tilt)
    pub = rospy.Publisher('axis', Float32, queue_size=10)
    pub.publish(axis)

    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    tilt_threshold = 5 * np.pi /180
    if tilt < 60 * np.pi / 180:
        if axis[0] > tilt_threshold:
            vel_msg.angular.z = (axis[0] - tilt_threshold) * -3
        elif axis[0] < -tilt_threshold:
            vel_msg.angular.z = (axis[0] + tilt_threshold) * -3

        if axis[1] > tilt_threshold:
            vel_msg.linear.x = (axis[1] - tilt_threshold) * 3
        elif axis[1] < -tilt_threshold:
            vel_msg.linear.x = (axis[1] + tilt_threshold) * 3

    velocity_publisher.publish(vel_msg)

    last_time = cur_time


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('imu', anonymous=True)

    rospy.Subscriber("chatter", Int16MultiArray, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.loginfo('spin')
    rospy.spin()


if __name__ == '__main__':
    listener()
