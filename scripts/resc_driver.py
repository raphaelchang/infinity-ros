#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import serial
import string
import time

class resc_driver(object):
    def __init__(self):
        self.initialized = False
        self.rpm_sub = rospy.Subscriber("~rpm", Float64, self.rpm_cb)

        self.port = rospy.get_param('~port', '/dev/ttyACM0')
        self.baudrate = rospy.get_param('~baudrate', 9600)
        self.t_out = rospy.get_param('~timeout', 0.05)
        self.initialized = False

        try:
            self.s = serial.Serial(self.port, self.baudrate,
                                   timeout=self.t_out)
            self.output = 'P\x01usb_override_set\n'

            self.s.write(self.output.encode())
            print(self.s.readline())

            self.initialized = True
        except serial.SerialException:
            rospy.logerr("Serial Exception!")

    def rpm_cb(self, msg):
        if self.initialized and self.s.isOpen():
            rpm = msg.data

            self.output = 'P\x01speed_set ' + str(rpm) + '\n'


            self.s.write(self.output.encode())

if __name__ == '__main__':
    rospy.init_node("resc_driver")
    node = resc_driver()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Shutting Down.")

