#!/usr/bin/python
import signal
import rospy
from xbox360controller import Xbox360Controller
from geometry_msgs.msg  import Twist
import numpy as np


class x360controller():
    def __init__(self):
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        while not rospy.is_shutdown():
            try:
                with Xbox360Controller(0, axis_threshold=0.2) as controller:
                    # Left and right axis move event
                    controller.axis_l.when_moved = self.on_axis_moved
                    controller.axis_r.when_moved = self.on_axis_moved

                    # Button stop
                    controller.button_a.when_released = self.on_button_released

                    signal.pause()
            except Exception:
                pass

    def on_axis_moved(self, axis):
        msg = Twist()

        if axis.name == "axis_l":
            msg.linear.x = axis.y
            msg.linear.y = axis.x

        if axis.name == "axis_r":
            msg.angular.z = axis.x

        self.pub.publish(msg)
        print(msg)

    def on_button_released(self, button):
        rospy.loginfo("Stopping")
        msg = Twist()
        self.pub.publish(msg)

        rospy.loginfo("Shutting down")
        rospy.signal_shutdown("Request shutdown")

if __name__ == "__main__":
    rospy.init_node("x360_controller", anonymous=True) 
    
    x360controller()     
    rospy.spin()
