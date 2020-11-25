/*
This code deprojects pixels into points in 3D space using basic stereo vision techniques. This is integral in the autonomy stack as it takes pixel detections as an input and outputs points in the camera's coordinate frame.
*/

#include "ros/ros.h"

#include <tf/transform_listener.h>
#include <sensor_msgs/CameraInfo.h>
#include "geometry_msgs/PointStamped.h"

#include <unordered_map>

using namespace sensor_msgs;
using namespace geometry_msgs;

class DeprojectPixelToPoint
{
	public:
		DeprojectPixelToPoint(){
			point_pub_ = nh_.advertise<PointStamped>( "/detections/real_center", 0 );
			camera_info_sub_ = nh_.subscribe("/camera/aligned_depth_to_color/camera_info", 1000, &DeprojectPixelToPoint::camera_info_callback, this);
			pixel_sub_ = nh_.subscribe("/detector/bb_center",1000, &DeprojectPixelToPoint::deproject_callback, this);
			nh_.getParam("/PixelToPointNode/camera_height", camera_height);
		}

		void camera_info_callback(const CameraInfoConstPtr& camera_info){
			cam_info["cx"] = camera_info->P[2];
			cam_info["cy"] = camera_info->P[6];
			cam_info["fx"] = camera_info->P[0];
			cam_info["fy"] = camera_info->P[5];
		}

		void deproject_callback(const PointStampedConstPtr& pixel_stamped){
		    PointStamped pt_msg;
		    pt_msg.point.x = (pixel_stamped->point.x*camera_height - cam_info["cx"]*camera_height) / cam_info["fx"];
		    pt_msg.point.y = (pixel_stamped->point.y*camera_height - cam_info["cy"]*camera_height) / cam_info["fy"];
		    pt_msg.point.z = camera_height; 
			pt_msg.header.stamp = ros::Time::now();
		    pt_msg.header.frame_id = "camera_color_optical_frame";
		    point_pub_.publish(pt_msg);
		}

	private:
		ros::NodeHandle nh_;
		ros::Publisher point_pub_;
		ros::Subscriber pixel_sub_;
		ros::Subscriber camera_info_sub_;
		std::unordered_map<std::string, double> cam_info;
		double camera_height;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pixel_to_point");

    DeprojectPixelToPoint foo;

    ros::spin();

	return 0;
}
