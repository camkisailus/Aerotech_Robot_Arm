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
			pixel_sub_ = nh_.subscribe("/camera/blob_detections/bb_center",1000, &DeprojectPixelToPoint::deproject_callback, this);
			nh_.getParam("/PixelToPointNode/camera_height", camera_height);
		}

		void camera_info_callback(const CameraInfoConstPtr& camera_info){
			cam_info["cx"] = camera_info->P[2];
			cam_info["cy"] = camera_info->P[6];
			cam_info["fx"] = camera_info->P[0];
			cam_info["fy"] = camera_info->P[5];
			// Tx and Ty are 0 and unused
			//cam_info["Tx"] = camera_info->P[3];
			//cam_info["Ty"] = camera_info->P[7];
			frame_id = camera_info->header.frame_id;
		}

		void deproject_callback(const PointStampedConstPtr& pixel_stamped){
		    PointStamped pt_msg, pt_msg_transformed;
		    Point pt;
		    pt.x = (pixel_stamped->point.x*camera_height - cam_info["cx"]*camera_height) / cam_info["fx"];
		    pt.y = (pixel_stamped->point.y*camera_height - cam_info["cy"]*camera_height) / cam_info["fy"];
		    pt.z = camera_height;
		    pt_msg.header.stamp = ros::Time::now();
		    pt_msg.header.frame_id = "camera_color_optical_frame";
		    pt_msg.point = pt;
		    // Transform from camera_color_optical_frame to camera_link
		    ROS_INFO("camera_link: %s", camera_link);
		    ROS_INFO("pt_msg frame: %s", pt_msg.header.frame_id);
		    listener.transformPoint(camera_link, pt_msg, pt_msg_transformed);
		    // For some reason these get messed up in the transform.. Fix them here
		    auto z = pt_msg_transformed.point.z;
		    auto y = pt_msg_transformed.point.y;
		    auto x = pt_msg_transformed.point.x;
		    pt_msg_transformed.point.z = x;
		    pt_msg_transformed.point.y = z;
		    pt_msg_transformed.point.x = y;
		    
		    point_pub_.publish(pt_msg_transformed);
		}

	private:
		ros::NodeHandle nh_;
		ros::Publisher point_pub_;
		ros::Subscriber pixel_sub_;
		ros::Subscriber camera_info_sub_;
		std::unordered_map<std::string, double> cam_info;
		std::string frame_id;
		std::string camera_link = "camera_link";
		double camera_height;
		tf::TransformListener listener;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pixel_to_point");

    DeprojectPixelToPoint foo;

    ros::spin();

	return 0;
}
