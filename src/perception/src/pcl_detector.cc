#include "ros/ros.h"

#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Point.h"

#include <vector>
#include <cmath>

using namespace sensor_msgs;
using namespace std;
using namespace geometry_msgs;

class PointCloudDetector{
public:
	PointCloudDetector(){
		point_pub_ = nh_.advertise<geometry_msgs::Point>("/camera/blob_detections/point", 1);
		pcl_sub_ = nh_.subscribe("/camera/depth_registered/points", 1000, &PointCloudDetector::callback, this);

	}

	void callback(const PointCloud2::ConstPtr& msg){
		auto row_step = msg->row_step;
		auto point_step = msg->point_step;
		auto x = 640/2;
		auto y = 480/2;
		Point pt;
		pt.x = msg->data[x*row_step + y*point_step];
		pt.y = msg->data[x*row_step + y*point_step + 4];
		pt.z = msg->data[x*row_step + y*point_step + 8];
		point_pub_.publish(pt);

	}


private:
	ros::NodeHandle nh_;
	ros::Publisher point_pub_;
	ros::Subscriber pcl_sub_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_detector");

    PointCloudDetector foo;

    ros::spin();

	return 0;
}