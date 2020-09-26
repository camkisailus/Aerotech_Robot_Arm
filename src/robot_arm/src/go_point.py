import rospy
from interbotix_sdk.robot_manipulation import InterbotixRobot
from interbotix_descriptions import interbotix_mr_descriptions as mrd


def main():
    arm = InterbotixRobot(robot_name="rx150", mrd=mrd)
    arm.go_to_home_pose()
    arm.set_ee_pose_components(x=0.2, y= 0.4, z=0.6)

if __name__=='__main__':
    main()
