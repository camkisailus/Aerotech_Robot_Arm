#include "ros/ros.h"

#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "geometry_msgs/PointStamped.h"

#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>

#include <vector>

using namespace cv;
using namespace sensor_msgs;
using namespace std;

class CircleDetector{
public:
	CircleDetector(){
		detection_pub_ = nh_.advertise<Image>("/camera/circle_detections/image_raw", 0);
		pixel_detection_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/camera/circle_detections/bb_center",0);
		image_sub_ = nh_.subscribe("/camera/color/image_raw", 1000, &CircleDetector::callback, this);
	}

	void callback(const Image::Ptr& img){
		cv_bridge::CvImagePtr cv_ptr;
		try{
			cv_ptr = cv_bridge::toCvCopy(img, image_encodings::BGR8);
		}catch(cv_bridge::Exception& e){
			ROS_ERROR("cv_bridge exception: %s", e.what());
		}

		Mat gray;
		// Convert color from BGR to gray scale
		cvtColor(cv_ptr->image, gray, COLOR_BGR2GRAY);
		// Blur
		medianBlur(gray, gray, 5);

		vector<Vec3f> circles;
		// HoughCircles(image, output vector, detection method, 
		//				inverse ratio of accumulator to image,
		//				minDist between detections,
		//				param1 higher threshold of 2 passed to canny edge (lower is half),
		//				param2 accumulator treshold for circle centers at detection stage (smaller = more false detections),
		//				minRadius,
		//				maxRadius)
		HoughCircles(gray, circles, HOUGH_GRADIENT,1, gray.rows/16, 200, 100, 30, 150);

		for(size_t i = 0; i <circles.size(); i++){
			Vec3i c = circles[i];
			Point center = Point(c[0], c[1]);
			circle(cv_ptr->image, center, 1, Scalar(0,100,100), 3, LINE_AA);
			int radius = c[2];
			circle(cv_ptr->image, center, radius, Scalar(255,0,255), 3, LINE_AA);
		}

		// Publish img_msg
		cv_bridge::CvImage frame;
		frame.header = cv_ptr->header;
		//Encoding for thresh image
		//frame.encoding = image_encodings::TYPE_8UC1;
		//Encoding for cv_ptr->image
		frame.encoding = image_encodings::BGR8;
		frame.image = cv_ptr->image;
		detection_pub_.publish(frame.toImageMsg());


	}


private:
	ros::NodeHandle nh_;
	ros::Publisher detection_pub_;
	ros::Publisher pixel_detection_pub_;
	ros::Subscriber image_sub_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "circle_detector");

    CircleDetector foo;

    ros::spin();

	return 0;
}