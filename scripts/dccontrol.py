#!/usr/bin/env python2

import rospy
import threading
import RPi.GPIO as GPIO
from time import sleep

from pi_dc_motor.msg import Motor

motorLeft = None
motorRight = None
TIME_OUT = 500      # Maximum time for waiting next instruction
timer = None

def config_gpio():
    global motorLeft
    global motorRight
    # Numbering by GPIO number, not header pin numberring
    GPIO.setmode(GPIO.BCM)

    # Use GPIO12 (PWM0) J8-32 to control speed of motor A
    #     GPIO13 (PWM1) J8-33 to control speed of motor B
    # Use GPIO07 J8-26   |
    #     GPIO08 J8-24  | to control direction of motor A
    # Use GPIO25 J8-22  |
    #     GPIO24 J8-18  | to control direction of motor B

    # Config pins
    # Motor A
    GPIO.setup(12, GPIO.OUT)
    GPIO.setup(7, GPIO.OUT, initial=0)
    GPIO.setup(8, GPIO.OUT, initial=0)
    motorRight = GPIO.PWM(12, 1000)
    # Motor B
    GPIO.setup(13, GPIO.OUT)
    GPIO.setup(25, GPIO.OUT, initial=0)
    GPIO.setup(24, GPIO.OUT, initial=0)
    motorLeft = GPIO.PWM(13, 1000)

    motorLeft.start(0)
    motorRight.start(0)

def left_dir(direction):
    if direction > 0:
        GPIO.output(25, 1)
        GPIO.output(24, 0)
    elif direction == 0:
        GPIO.output(25, 0)
        GPIO.output(24, 0)
    else:
        GPIO.output(25, 0)
        GPIO.output(24, 1)
def right_dir(direction):
    if direction > 0:
        GPIO.output(7, 1)
        GPIO.output(8, 0)
    elif direction == 0:
        GPIO.output(7, 0)
        GPIO.output(8, 0)
    else:
        GPIO.output(7, 0)
        GPIO.output(8, 1)
def stop():
    rospy.loginfo(rospy.get_caller_id() + " Motor control: Stop motors")
    global motorLeft
    global motorRight
    left_dir(0)
    right_dir(0)
    motorLeft.ChangeDutyCycle(0)
    motorRight.ChangeDutyCycle(0)
def go(left, right):
    global motorLeft
    global motorRight
    ld = 1
    rd = 1
    if left < 0:
        ld = -1
    elif left == 0:
        ld = 0
    if right < 0:
        rd = -1
    elif right == 0:
        rd = 0
    left_dir(ld)
    right_dir(rd)
    motorLeft.ChangeDutyCycle(abs(left))
    motorRight.ChangeDutyCycle(abs(right))

def terminate():
    global motorLeft
    global motorRight
    # Stop and clean
    stop()
    motorLeft.stop()
    motorRight.stop()
    GPIO.cleanup()

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + " Motor control: left=%d, right=%d" % (data.left, data.right))
    # Check if timer is finished
    global timer
    if timer.isAlive():
        # If timer is alive, cancel it and restart
        timer.cancel()
    left = data.left
    right = data.right

    if left < -100:
        left = -100
    elif left > 100:
        left = 100
    if right < -100:
        right = -100
    elif right > 100:
        right = 100
    go(left, right)

    # Create timer thread again
    timer = threading.Timer(TIME_OUT/1000.0, stop)
    # Start timeout timer
    timer.start()

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('pi_dc_motor', anonymous=True)

    config_gpio()

    # Create time out timer, this timer ensure the next instruction must come
    # in TIME_OUT (ms)
    global timer
    timer = threading.Timer(TIME_OUT/1000.0, stop)
    timer.start()

    rospy.Subscriber("pi_motor_control", Motor, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    terminate()

if __name__ == '__main__':
    listener()

