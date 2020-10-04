import rospy
from interbotix_sdk.robot_manipulation import InterbotixRobot
from interbotix_descriptions import interbotix_mr_descriptions as mrd


def main():
    arm = InterbotixRobot(robot_name="rx150", mrd=mrd)
    arm.go_to_home_pose()
    arm.set_ee_pose_components(x=0.0, y=-0.2, z=0.1)

if __name__=='__main__':
    main()
