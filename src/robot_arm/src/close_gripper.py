import rospy
from interbotix_sdk.robot_manipulation import InterbotixRobot
from interbotix_descriptions import interbotix_mr_descriptions as mrd


def main():
    arm = InterbotixRobot(robot_name="rx150", mrd=mrd, gripper_pressure=1)
    arm.close_gripper()

if __name__=='__main__':
    main()
