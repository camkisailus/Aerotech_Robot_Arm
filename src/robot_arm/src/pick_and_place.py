#!/usr/bin/env python3
import rospy
import time
from interbotix_sdk.robot_manipulation import InterbotixRobot
from interbotix_descriptions import interbotix_mr_descriptions as mrd
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import String


class PickAndPlace:
    def __init__(self):
        self.arm = InterbotixRobot(robot_name="rx150", mrd=mrd, gripper_pressure=1)
        self.pt_sub = rospy.Subscriber(
            '/detections/real_center_base_link',
            PointStamped,
            self.callback,
            queue_size=1)
        self.relay_pub = rospy.Publisher('/vacuum', String, queue_size=1)
        self.request_detection_pub = rospy.Publisher('/robot/get_new_point', String, queue_size=1)
        self.target = Point()
        self.target.x = 0.3048
        self.target.y = 0.0
        self.target.z = 0.063
        self.arm.go_to_home_pose()
        self.detect_mode = True

    def request_detection(self):
        self.request_detection_pub.publish("foo")


    def move_to_point(self, pt):
        """
            Abstract the set_ee_pose_compenent method to clean up callback. 

            Args:
                pt (Point) -- desired end effector point
        """
        # List of hard coded positions that the IKSolver will use as seeds to better solve the planning problem
        custom_guesses = [
            [-0.012271846644580364, 0.7470486760139465, -0.26691266894340515, 0.6181942820549011, 0.08897088468074799],
            [-0.6181942820549011, 1.0047574043273926, 0.4218447208404541, 0.11044661700725555, 0.0920388475060463],
            [-1.121340036392212, 0.8620972037315369, -0.04601942375302315, 0.7900001406669617, 0.006135923322290182],
            [-1.7794177532196045, 0.5292233824729919, -0.503145694732666, 0.9526020884513855, -0.00920388475060463]
        ]
        for guess in custom_guesses:
            _, res = self.arm.set_ee_pose_components(x=pt.x, y=pt.y, z=pt.z, custom_guess=guess)
            if(res):
                # Planning problem solved
                return True
        
        return False


    def callback(self, msg):
        """
            Main callback that moves the robot to the detected point, turns on the vacuum, moves to target
            and then turns off the vacuum and returns home
        """ 
        self.detect_mode = False      
        pt = msg.point
        # Hard code the z value so the vacuum head is appropriately placed above the lens
        pt.z = 0.13
        self.move_to_point(pt)
        pt.z = 0.067
        self.move_to_point(pt)
        self.turn_on_vacuum()
        time.sleep(2.0)
        pt.z = 0.15
        self.move_to_point(pt)
        self.arm.go_to_home_pose()
        # Move to target
        self.move_to_point(self.target)
        pt.x = self.target.x
        pt.y = self.target.y
        pt.z = 0.13
        time.sleep(1.0)
        self.turn_off_vacuum()
        time.sleep(1.0)
        self.move_to_point(pt)

        # Home
        self.arm.go_to_home_pose()
        self.detect_mode = True
    
    def turn_on_vacuum(self):
        self.relay_pub.publish(String("ON"))

    def turn_off_vacuum(self):
        self.relay_pub.publish(String("OFF"))
    
    def run(self):
        if(self.detect_mode):
            self.request_detection()


if __name__ == '__main__':
    foo = PickAndPlace()
    time.sleep(5)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        foo.run()
        rate.sleep()
