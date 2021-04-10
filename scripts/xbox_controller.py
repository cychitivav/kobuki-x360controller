#!/usr/bin/python
import rospy
from xbox360controller import Xbox360Controller
from geometry_msgs.msg  import Twist
import numpy as np


class x360controller():
    def __init__(self):
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        try:
            controller = Xbox360Controller()
                
            # Left and right axis move event
            controller.axis_l.when_moved = self.movement

            # Button stop and kill
            controller.button_a.when_released = self.kill
            controller.button_b.when_released = self.stop
        except Exception:
            rospy.logwarn("x360 controller disconnected")
            rospy.signal_shutdown("controller disconnected")

    def movement(self, axis):
        if not rospy.is_shutdown():   
            msg = Twist()

            msg.linear.x = axis.y
            msg.angular.z = axis.x*np.pi

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
