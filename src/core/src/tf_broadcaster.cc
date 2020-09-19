#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "robot_tf_publisher");
	ros::NodeHandle nh;
	

	ros::Rate r(100);

	tf::TransformBroadcaster broadcaster;
	double x_trans, y_trans, z_trans;
	nh.getParam("/TfBroadcaster/x_trans", x_trans);
	nh.getParam("/TfBroadcaster/y_trans", y_trans);
	nh.getParam("/TfBroadcaster/z_trans", z_trans);
	while(nh.ok()){
		broadcaster.sendTransform(
			tf::StampedTransform(
				tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(x_trans,y_trans,z_trans)),
				ros::Time::now(), "rx150/base_link", "camera_link"));
		r.sleep();
	}
}
