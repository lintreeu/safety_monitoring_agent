#!/usr/bin/env python3
import sys
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
sys.path.append('/usr/local/lib/python3.5/dist-packages')

import rospy
from std_msgs.msg import Float64, String

class CloseDistanceController():

    def __init__(self):

        rospy.init_node('Agent')
        rospy.Subscriber('front_distance', Float64, self._control)
        self.pub = rospy.Publisher('command', String, queue_size=10) 
        self.value = 0 
        self.condition = 'normal control'

    
    def update(self, condtion):
        self.condtion = condtion

    #the callback function
    #use to control the speed of the agv based on the signal continuously detected by the sensors
    def _control(self, sig):

        self.value = sig.data
        if self.condtion == 'emergency_control':
            if (self.value <= 20) and (self.value > 0) :
                  print ("The rule action is stop")
                  self.pub.publish("stop")

            if self.value > 20:
                  print("The rule action is \"go\"")
                  self.pub.publish("go")
        
        if self.condtion == 'normal control':
            pass

    def display(self):
        print('value:', self.value, 'condtion', self.condition)

    def run(self):
        rospy.spin()

class VirtualController:
    def __init__(self):
        self.value = ""
    def display(self):
        print(self.value)
    def update(self, msg):
        self.value = msg
    def run(self):
        while True:
            print('the signal received',self.value)


if __name__ == '__main__':
    
    # controller = VirtualController()
    # controller.run()
    controller = CloseDistanceController()
    controller.run()

