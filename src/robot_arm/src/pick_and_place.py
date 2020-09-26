#!/usr/bin/env python3
import rospy
import time
from interbotix_sdk.robot_manipulation import InterbotixRobot
from interbotix_descriptions import interbotix_mr_descriptions as mrd
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import String

class PickAndPlace:
    def __init__(self):
        self.arm = InterbotixRobot(robot_name="rx150", mrd=mrd)
        self.arm.set_gripper_pressure(2)
        self.pt_sub = rospy.Subscriber('/detections/real_center_base_link', PointStamped, self.callback, queue_size=1)
        self.relay_pub = rospy.Publisher('/vacuum', String, queue_size = 1)
        self.target = Point()
        self.target.x = 0.2
        self.target.y = 0.2
        self.target.z = 0.2

    def callback(self,msg):
        # Wake up
        self.arm.go_to_home_pose()

        # Move to passed point
        pt = msg.point
        self.arm.set_ee_pose_components(x=pt.x, y=pt.y, z=0.0762)
        self.turn_on_vacuum()
        time.sleep(2.0)
        self.arm.go_to_home_pose()

        # Move to target
        self.arm.set_ee_pose_components(x=self.target.x, y=self.target.y, z=self.target.z)
        time.sleep(3.0)
        self.turn_off_vacuum()
        time.sleep(1.0)

        # Sleep
        self.arm.go_to_home_pose()
        self.arm.go_to_sleep_pose()


    def turn_on_vacuum(self):
        self.relay_pub.publish(String("ON"))
        

    def turn_off_vacuum(self):
        self.relay_pub.publish(String("OFF"))

if __name__=='__main__':
    foo = PickAndPlace()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
