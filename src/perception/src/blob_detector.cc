#include "ros/ros.h"

#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/String.h"

#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <mutex>

#include <vector>
#include <cmath>

using namespace sensor_msgs;
using namespace cv;
using namespace std;

std::mutex frame_lock;

class BlobDetector{
public:
	BlobDetector(){
		detection_pub_ = nh_.advertise<Image>("/camera/blob_detections/image_raw", 0);
		pixel_detection_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/camera/blob_detections/bb_center",0);
		image_sub_ = nh_.subscribe("/camera/color/image_raw", 1000, &BlobDetector::camera_callback, this);
		detect_sub_ = nh_.subscribe("/robot/get_new_point", 1000, &BlobDetector::pub_callback, this);

	}

	void pub_callback(const std_msgs::String& msg){
		detect();		
	}

	void camera_callback(const Image::Ptr& img){
		frame_lock.lock();
		latest_frame = *img;
		frame_lock.unlock();
	}

	void detect(void){
		frame_lock.lock();
		cv_bridge::CvImagePtr cv_ptr;
		try{
			cv_ptr = cv_bridge::toCvCopy(latest_frame, image_encodings::BGR8);
		}catch(cv_bridge::Exception& e){
			ROS_ERROR("cv_bridge exception: %s", e.what());
			frame_lock.unlock();
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
		vector<Rect> bb_circles;
		for( size_t i = 0; i < contours.size(); i++ )
	    {
	        // Contours are stored as vector of points, this finds the polygonal shape of those points
	        approxPolyDP( contours[i], contours_poly[i], 3, true );
	        auto area = contourArea(contours[i]);
	        if((contours_poly[i].size() > 15) && (area >30) ){
	        	// Get the bounding rectangle of this ^ polygon
	        	bb_circles.push_back(boundingRect( contours_poly[i] ));
	        }
	    }

	    for( size_t i = 0; i < bb_circles.size(); i++ )
	    {
	    	bool publish = true;
	    	cv::Point center;
	    	center.x = bb_circles[i].x + bb_circles[i].width/2;
	    	center.y = bb_circles[i].y + bb_circles[i].height/2;
	    	for(size_t j = 0; j < published_blobs.size(); j++){
	    		auto dist = sqrt(center.x*center.x + center.y*center.y);
	    		if(abs(dist - published_blobs[j])<100){
	    			// if the center of the detection is with 100 pixels of any other already detected circle do not publish.
	    			publish = false;
	    		}
	    	}
	    	if(publish){
	    		Scalar color = Scalar(0, 0, 255);
		        rectangle( cv_ptr->image, bb_circles[i].tl(), bb_circles[i].br(), color, 2 );
		    	circle( cv_ptr->image, center, 5, color);

		    	published_blobs.push_back(center.x*center.x + center.y*center.y);
		    	
	    		geometry_msgs::PointStamped pt_msg;
	    		pt_msg.header.stamp = ros::Time::now();
	    		pt_msg.header.frame_id = "camera_color_optical_frame";
	    		geometry_msgs::Point pt;
	    		pt.x = center.x;
	    		pt.y = center.y;
	    		pt.z = 0;
	    		pt_msg.point = pt;
	    		pixel_detection_pub_.publish(pt_msg);
	    		// Only publish one at a time
	    		return;
	    	}else{
	    		Scalar color = Scalar(0,255,0);
	        	rectangle( cv_ptr->image, bb_circles[i].tl(), bb_circles[i].br(), color, 2 );
	    		circle( cv_ptr->image, center, 5, color);
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
		frame_lock.unlock();

	}


private:
	ros::NodeHandle nh_;
	ros::Publisher detection_pub_;
	ros::Publisher pixel_detection_pub_;
	ros::Subscriber image_sub_;
	ros::Subscriber detect_sub_;
	std::vector<double> published_blobs;
	Image latest_frame;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "blob_detector");

    BlobDetector foo;

    ros::spin();

	return 0;
}