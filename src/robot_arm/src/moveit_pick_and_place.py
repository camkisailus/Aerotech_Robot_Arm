#!/usr/bin/env python

import sys
import time
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import *
from moveit_commander.conversions import pose_to_list


class PickAndPlace(object):

    def __init__(self):
        # First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface',
                        anonymous=True)
        self.robot_name = "rx150"
        self.dof = 5

        # Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
        # the robot:
        robot = moveit_commander.RobotCommander()

        # Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
        # to the world surrounding the robot:
        scene = moveit_commander.PlanningSceneInterface()

        # Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        # to one group of joints.  In this case the group is the joints in the Interbotix
        # arm so we set ``group_name = interbotix_arm``. If you are using a different robot,
        # you should change this value to the name of your robot arm planning group.
        # This interface can be used to plan and execute motions on the
        # Interbotix Arm:
        group_name = "interbotix_arm"
        group = moveit_commander.MoveGroupCommander(group_name)

        # We create a `DisplayTrajectory`_ publisher which is used later to publish
        # trajectories for RViz to visualize:
        display_trajectory_publisher = rospy.Publisher(
            "move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20)
        # We can get the name of the reference frame for this robot:
        planning_frame = group.get_planning_frame()

        # We can also print the name of the end-effector link for this group:
        eef_link = group.get_end_effector_link()

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()

        self.robot = robot
        self.scene = scene
        self.group = group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        self.pt_sub = rospy.Subscriber(
            '/detections/real_center_base_link',
            PointStamped,
            self.callback,
            queue_size=1)
        self.home = Point()
        self.home.x = 0.25
        self.home.y = 0.0
        self.home.z = 0.2

        # Add obstacles in the scene
        box1_pose = Pose()
        box1_pose.position.x = 0.1
        box1_pose.position.y = 0.25
        box1_pose.position.z = 0.0
        box2_pose = Pose()
        box2_pose.position.x = -0.1
        box2_pose.position.y = 0.25
        box2_pose.position.z = 0.0
        camera_pose = Pose()
        camera_pose.position.x = 0.0
        camera_pose.position.y = 0.25
        camera_pose.position.z = 0.1

        

        self.go_to_pose_goal(self.home)
        self.add_box("can_1", box1_pose, 0.08, 0.08, 0.14)
        self.add_box("can_2", box2_pose, 0.08, 0.08, 0.14)
        self.add_box("camera_holder", camera_pose, 0.3048, 0.08, 0.01)

    def callback(self, msg):
        pt = msg.point
        self.go_to_pose_goal(pt)

    def add_box(self, name, pose, size_x, size_y, size_z):
        """
        Add a collision object into the planning scene

        Args:
            name (String) : name that object will be called in planning scene
            pose (geometry_msgs/Pose) : pose of box wrt rx150/base_link
            size_x (float) : dimension of box in x axis (m)
            size_y (float) : dimension of box in y axis (m)
            size_z (float) : dimension of box in z axis (m)
        """
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = self.robot_name + "/base_link"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = pose.position.x
        box_pose.pose.position.y = pose.position.y
        box_pose.pose.position.z = pose.position.z
        box_name = name
        self.scene.add_box(box_name, box_pose, size=(size_x, size_y, size_z))

    def go_to_pose_goal(self, pose):
        """
            Planning to a Pose Goal
            Args:
                pose (geometry_msgs/Pose) : desired end effector pose

        """
        p = [pose.x, pose.y, pose.z]
        q = [1, 0, 0, 0]
        pose_goal = p + q
        pose_6 = [pose.x, pose.y, pose.z, 0, 0, 0]
        self.group.set_orientation_target(q)
        self.group.set_position_target(p)
        #self.group.set_pose_target(pose_6)


        # Call the planner to compute the plan and execute it.
        plan = self.group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.group.stop()
        # It is always good to clear your targets after planning with poses.
        self.group.clear_pose_targets()


if __name__ == '__main__':
    foo = PickAndPlace()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
