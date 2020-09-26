#!/usr/bin/env python3
import rospy
import time
from interbotix_sdk.robot_manipulation import InterbotixRobot
from interbotix_descriptions import interbotix_mr_descriptions as mrd
from geometry_msgs.msg import PointStamped, Point

class PickAndPlace:
    def __init__(self):
        self.arm = InterbotixRobot(robot_name="rx150", mrd=mrd)
        self.arm.set_gripper_pressure(2)
        self.pt_sub = rospy.Subscriber('/detections/real_center_base_link', PointStamped, self.callback, queue_size=1)
        self.target = Point()
        self.target.x = 0.0
        self.target.y = -0.2
        self.target.z = 0.4

    def callback(self,msg):
    	# Wake up
        self.arm.go_to_home_pose()
        self.arm.close_gripper()
        self.arm.open_gripper()

        # Move to passed point
        pt = msg.point
        self.arm.set_ee_pose_components(x=pt.x, y=pt.y, z=pt.z)
        self.turn_on_vacuum()
        time.sleep(2.0)
        self.arm.close_gripper()

        # Move to target
        self.arm.set_ee_pose_components(x=self.target.x, y=self.target.y, z=self.target.z)
        time.sleep(3.0)
        self.turn_off_vacuum()
        self.arm.open_gripper()

        # Sleep
        self.arm.go_to_home_pose()
        self.arm.go_to_sleep_pose()
        self.arm.close_gripper()


    def turn_on_vacuum(self):
    	print("Turning on vacuum")

    def turn_off_vacuum(self):
    	print("Turning off vacuum")

if __name__=='__main__':
    foo = PickAndPlace()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
