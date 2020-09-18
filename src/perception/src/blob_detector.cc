#include "ros/ros.h"

#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "geometry_msgs/PointStamped.h"

#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <vector>
#include <cmath>

using namespace sensor_msgs;
using namespace cv;
using namespace std;

class BlobDetector{
public:
	BlobDetector(){
		detection_pub_ = nh_.advertise<Image>("/camera/detections/image_raw", 0);
		pixel_detection_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/camera/detections/bb_center",0);
		image_sub_ = nh_.subscribe("/camera/color/image_raw", 1000, &BlobDetector::callback, this);

	}

	void callback(const Image::Ptr& img){
		cv_bridge::CvImagePtr cv_ptr;
		try{
			cv_ptr = cv_bridge::toCvCopy(img, image_encodings::BGR8);
		}catch(cv_bridge::Exception& e){
			ROS_ERROR("cv_bridge exception: %s", e.what());
		}

		// Mess with image
		cv::Mat gray, blurred, thresh;
		auto maxValue = 255;
		auto threshValue = 200;
		// Convert color from BGR to gray scale
		cvtColor(cv_ptr->image, gray, COLOR_BGR2GRAY);
		// Blur gray scale image
		GaussianBlur(gray, blurred, Size(5,5), 0);
		// Threshold to binary. pixels > 200 = 1 else 0
		threshold(blurred, thresh, threshValue, maxValue, THRESH_BINARY );

		// Find contours
		vector<vector<cv::Point>> contours;
		vector<Vec4i> hierarchy;
		// CV_RETR_EXTERNAL only returns extreme contours, so contours within the blob are ignored
		findContours( thresh, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

		// Draw contours
		vector<vector<cv::Point> > contours_poly( contours.size() );
		vector<Rect> boundRect( contours.size() );
		for( size_t i = 0; i < contours.size(); i++ )
	    {
	        // Contours are stored as vector of points, this finds the polygonal shape of those points
	        approxPolyDP( contours[i], contours_poly[i], 3, true );
	        // Get the bounding rectangle of this ^ polygon
	        boundRect[i] = boundingRect( contours_poly[i] );
	    }

	    for( size_t i = 0; i < contours.size(); i++ )
	    {
	    	bool publish = true;
	    	cv::Point center;
	    	center.x = boundRect[i].x + boundRect[i].width/2;
	    	center.y = boundRect[i].y + boundRect[i].height/2;
	        Scalar color = Scalar(255,0,0);
	        rectangle( cv_ptr->image, boundRect[i].tl(), boundRect[i].br(), color, 2 );
	    	circle( cv_ptr->image, center, 5, color);
	    	for(size_t j = 0; j < published_blobs.size(); j++){
	    		auto dist = sqrt(center.x*center.x + center.y*center.y);
	    		if(abs(dist - published_blobs[j])<25){
	    			// if the center of the detection is with 25 pixels of any other already detected circle do not publish.
	    			publish = false;
	    		}
	    	}
	    	if(publish){
	    		published_blobs.push_back(sqrt(center.x*center.x + center.y*center.y));
	    		geometry_msgs::PointStamped pt_msg;
	    		//'{stamp: now, frame_id: base_link}' '[320.0, 360.0, 0.0]'
	    		pt_msg.header.stamp = ros::Time::now();
	    		pt_msg.header.frame_id = "camera_link";
	    		geometry_msgs::Point pt;
	    		pt.x = center.x;
	    		pt.y = center.y;
	    		pt.z = 0;
	    		pt_msg.point = pt;
	    		pixel_detection_pub_.publish(pt_msg);
	    	}
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
	std::vector<double> published_blobs;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "blob_dector");

    BlobDetector foo;

    ros::spin();

	return 0;
}