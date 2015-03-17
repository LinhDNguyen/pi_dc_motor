#!/usr/bin/env python2

import rospy
from time import sleep

from pi_dc_motor.msg import Motor

pub = None
state = 0

def node_shutdown():
    rospy.loginfo("NODE SHUTDOWN")
    if not pub:
        return
    m = Motor()
    m.left = 0
    m.right = 0
    pub.publish(m)

def talker():
    global pub
    pub = rospy.Publisher('pi_motor_control', Motor, queue_size=10)
    rospy.init_node('pi_car', anonymous=True)
    rospy.on_shutdown(node_shutdown)
    # rate = rospy.Rate(10) # 10hz
    m = Motor()
    while not rospy.is_shutdown():
        # Front
        m.left = 100
        m.right = 100
        pub.publish(m)
        sleep(1)

        # Stop
        m.left = 0
        m.right = 0
        pub.publish(m)
        sleep(1)

        # Back
        m.left = -100
        m.right = -100
        pub.publish(m)
        sleep(1)

        # Stop
        m.left = 0
        m.right = 0
        pub.publish(m)
        sleep(1)

        # rate.sleep()
    # Stop
    m.left = 0
    m.right = 0
    pub.publish(m)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        rospy.loginfo("STOP")
        m = Motor()
        m.left = 0
        m.right = 0
        pub.publish(m)
    except:
        import traceback
        rospy.loginfo(traceback.format_exc())