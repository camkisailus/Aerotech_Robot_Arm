import rospy
from geometry_msgs.msg import Point
from interbotix_sdk.robot_manipulation import InterbotixRobot
from interbotix_descriptions import interbotix_mr_descriptions as mrd
import numpy as np

class FakePathFollower:
    def __init__(self):
        self.arm = InterbotixRobot(robot_name="rx150", mrd=mrd)
        self.arm.set_gripper_pressure(1.5)
        self.subscriber = rospy.Subscriber('/fake_point',Point, self._callback, queue_size=1)

    def _callback(self, msg):
        self.move(msg.x, msg.y, msg.z)

    def move(self, x_coord, y_coord, z_coord):
        self.arm.set_ee_pose_components(x=x_coord, y=y_coord, z=z_coord)

if __name__=='__main__':
    foo = FakePathFollower()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
    
