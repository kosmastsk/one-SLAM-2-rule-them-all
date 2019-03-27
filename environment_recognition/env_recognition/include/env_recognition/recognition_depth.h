#ifndef RECOGNITION_RGB_H
#define RECOGNITION_RGB_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <math.h>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace std;

namespace env_recognition
{
static int lowThreshold_ = 0;
static const int ratio_ = 3;
static const int kernel_size_ = 3;

class RecognitionDepth
{
	private:
		ros::NodeHandle n_;
		std_msgs::String msg_env_;	//! Publish the recognised environment through this msg

		//! Random variables
		float temp_edges_;			//! Percentage of the non-black values (meaning the edges)
		float list_edges_[100];		//! A list of the last 100 non-black values
		float sum_edges_; 			//! Sum of the last 100 non-black values
		float average_edges_;		//! Mean value of the last 100 non-black values

		string temp_env_;			//! Environment type to be published
		string previous_;
		int counter_msgs_;			//! Counts the laser and grayscale measuremets so far
		int index_;					//! Index of the list we are working on 
		int samples_; 				//! Samples, from which average value will occur

		//! OpenCV variables
		cv_bridge::CvImagePtr ptr_;	//! The cv format of the received image
		Mat src_;					//! The uint8 format of the cv image
    	Mat dst_, detected_edges_;	//! The edge format of the uint8 image
		
		//! ROS Parameters
		float threshold_brightness_;	//! Higher -> Bright
		float threshold_darkness_;		//! Lower  -> Dark
		float threshold_complexity_;    //! Lower  -> Simple || Higher -> Complex

		//! ROS Publisher
		ros::Publisher env_pub_;

		//! ROS Subscriber
		ros::Subscriber rgb_sub_;

		//! Random Functions
		void initParams 	(void);
		void initValues		(void);
		void depthCallback	(const sensor_msgs::Image::ConstPtr &msg_rgb);
		void averageValues	(void);
		void caseCheck		(void);

	public:
		RecognitionDepth 		(void);
		~RecognitionDepth 	(void);
};
}

#endif