#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
import RPi.GPIO as GPIO
LED_L = 22
LED_F = 17
LED_R = 27


def callback(led_msg):
    print(led_msg)
    if led_msg.data == 100: 
        GPIO.output(LED_L, True)
        GPIO.output(LED_F, False)
        GPIO.output(LED_R, False)

    elif led_msg.data == 101:
        GPIO.output(LED_L, True)
        GPIO.output(LED_F, False)
        GPIO.output(LED_R, True)

    elif led_msg.data == 110:
        GPIO.output(LED_L, True)
        GPIO.output(LED_F, True)
        GPIO.output(LED_R, False)

    elif led_msg.data == 111:
        GPIO.output(LED_L, True)
        GPIO.output(LED_F, True)
        GPIO.output(LED_R, True)

    elif led_msg.data == 1:
        GPIO.output(LED_L, False)
        GPIO.output(LED_F, False)
        GPIO.output(LED_R, True)

    elif led_msg.data == 10:
        GPIO.output(LED_L, False)
        GPIO.output(LED_F, True)
        GPIO.output(LED_R, False)

    elif led_msg.data == 11:
        GPIO.output(LED_L, False)
        GPIO.output(LED_F, True)
        GPIO.output(LED_R, True)
        
    else:
        GPIO.output(LED_L, False)
        GPIO.output(LED_F, False)
        GPIO.output(LED_R, False)

    
def led_ctrl():
    rospy.init_node('led_controller', anonymous=True)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(LED_L, GPIO.OUT)
    GPIO.setup(LED_F, GPIO.OUT)
    GPIO.setup(LED_R, GPIO.OUT)
    GPIO.output(LED_L, False)
    GPIO.output(LED_F, False)
    GPIO.output(LED_R, False)
    
    rospy.Subscriber("gas_detect_led", Int32, callback)
    rospy.spin()

if __name__ == '__main__':
    led_ctrl()
    GPIO.cleanup()
