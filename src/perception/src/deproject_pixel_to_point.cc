#include "ros/ros.h"

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
			pixel_sub_ = nh_.subscribe("/camera/detections/bb_center",1000, &DeprojectPixelToPoint::deproject_callback, this);
			//nh_.getParam("/PixelToPointNode/camera_height", camera_height);
		}

		void camera_info_callback(const CameraInfoConstPtr& camera_info){
			cam_info["cx"] = camera_info->P[2];
			cam_info["cy"] = camera_info->P[6];
			cam_info["fx"] = camera_info->P[0];
			cam_info["fy"] = camera_info->P[5];
			cam_info["Tx"] = camera_info->P[3];
			cam_info["Ty"] = camera_info->P[7];
		}

		void deproject_callback(const PointStampedConstPtr& pixel_stamped){
		    Point pixel = pixel_stamped->point;
		    auto px = pixel.x;
		    auto py = pixel.y;

		    auto cx = cam_info["cx"];
		    auto cy = cam_info["cy"];
		    auto fx = cam_info["fx"];
		    auto fy = cam_info["fy"];
		    auto Tx = cam_info["Tx"];
		    auto Ty = cam_info["Ty"];

		    //auto Z = 0.001*depth_ptr->image.at<u_int16_t>(px, py); // Meters

		    auto X = (px*camera_height - cx*camera_height) / fx; // Meters
		    auto Y = (py*camera_height - cy*camera_height) / fy; // Meters

		    PointStamped pt_msg;
		    Point pt;
		    pt.x = X;
		    pt.y = Y;
		    pt.z = camera_height;
		    pt_msg.header.stamp = ros::Time::now();
		    pt_msg.header.frame_id = "camera_link";
		    pt_msg.point = pt;
		    point_pub_.publish(pt_msg);
		}

	private:
		ros::NodeHandle nh_;
		ros::Publisher point_pub_;
		ros::Subscriber pixel_sub_;
		ros::Subscriber camera_info_sub_;
		std::unordered_map<std::string, double> cam_info;
		double camera_height = 0.1397;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pixel_to_point");

    DeprojectPixelToPoint foo;

    ros::spin();

	return 0;
}
