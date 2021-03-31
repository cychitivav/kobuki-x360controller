#!/usr/bin/python
import signal
import rospy
from xbox360controller import Xbox360Controller
from geometry_msgs.msg  import Twist
import numpy as np


def publisher_x360():
    with Xbox360Controller(0, axis_threshold=0.1) as controller:
        # Left and right axis move event
        controller.axis_l.when_moved = on_axis_moved
        controller.axis_r.when_moved = on_axis_moved

        signal.pause()

def on_axis_moved(axis):
    msg = Twist()

    if axis.name == "axis_l":
        msg.linear.x = axis.y
        msg.linear.y = axis.x

    if axis.name == "axis_r":
        msg.angular.z = axis.x*np.pi

    pub.publish(msg)
        

if __name__ == "__main__":
    rospy.init_node("x360_controller", anonymous=True) 
    
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    publisher_x360()     
    rospy.spin()



