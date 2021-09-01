#!/usr/bin/python
import rospy
from xbox360controller import Xbox360Controller
from geometry_msgs.msg  import Twist
import numpy as np


class x360controller():
    def __init__(self):
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        try:
            self.controller = Xbox360Controller(0, axis_threshold=0.1)
                
            # Left and right axis move event
            self.controller.axis_l.when_moved = self.movement
            self.controller.axis_r.when_moved = self.movement

            # Button stop and kill
            self.controller.button_a.when_released = self.kill
            self.controller.button_b.when_released = self.stop
        except Exception:
            rospy.logwarn("x360 controller disconnected")
            rospy.signal_shutdown("controller disconnected")

    def movement(self, axis):
        if not rospy.is_shutdown():   
            msg = Twist()

            msg.linear.x = self.controller.axis_l.y
            msg.linear.y = self.controller.axis_l.x
            msg.angular.z = self.controller.axis_r.x*np.pi

            self.pub.publish(msg)

    def kill(self, button):
        if not rospy.is_shutdown():   
            rospy.loginfo("Shutting down")
            rospy.signal_shutdown("Request shutdown")

    def stop(self, button):
        if not rospy.is_shutdown():   
            rospy.loginfo("Stopping")
            msg = Twist()
            self.pub.publish(msg)
        

if __name__ == "__main__":
    rospy.init_node("x360_controller", anonymous=True) 
    
    x360controller()     
    rospy.spin()
