#!/usr/bin/env python2
import rospy
import tf
from geometry_msgs.msg import PointStamped


class TfTransform():
    def __init__(self):
        self.tf = tf.TransformListener()
        self.point_sub = rospy.Subscriber('/detections/real_center', PointStamped, self.callback, queue_size=1)
        self.point_pub = rospy.Publisher('/detections/real_center_base_link', PointStamped, queue_size=1)

    def callback(self, msg):
        # Transform passed point from /camera_link to rx150/base_link
        if self.tf.frameExists('rx150/base_link') and self.tf.frameExists('camera_link'):
        	base_link_point = self.tf.transformPoint("rx150/base_link", msg)
        	self.point_pub.publish(base_link_point)


if __name__ == '__main__':
    rospy.init_node('tf_transform_node', anonymous=True)
    foo = TfTransform()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
