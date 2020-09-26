#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point

def foo():
    pub = rospy.Publisher('/fake_point', Point, queue_size=10)
    rospy.init_node('FakePointNode', anonymous=True)
    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        pt_msg  = Point()
        pt_msg.x = 0.0 # [-0.4,-0.1, 0.1, 0.4]
        pt_msg.y = 0.2
        pt_msg.z = 0.2
        pub.publish(pt_msg)
        rate.sleep()

if __name__=='__main__':
    try:
        foo()
    except rospy.ROSInterruptException:
        pass
