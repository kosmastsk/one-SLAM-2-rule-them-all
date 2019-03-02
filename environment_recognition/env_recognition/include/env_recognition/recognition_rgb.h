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

using namespace std;

namespace env_recognition
{
class RecognitionRgb
{
	private:
		ros::NodeHandle n_;
		std_msgs::String msg_env_;	//! Publish the recognised environment through this msg

		//! Random variables
		float temp_gray_;			//! Average grayscale value of each measurement
		float list_gray_[100];		//! A list of the last 100 grayscale average values
		float sum_gray_; 			//! Sum of the last 100 grayscale average values
		float average_gray_;		//! Mean value of the last 100 grayscale average values

		string temp_env_;			//! Environment type to be published
		string previous_;			//! The previous type of environment
		int counter_msgs_;			//! Counts the laser and grayscale measuremets so far
		int index_;					//! Index of the list we are working on 
		int samples_; 				//! Samples, from which average value will occur

		//! OpenCV variables
		cv_bridge::CvImagePtr cv_ptr_; 	//! Here we store a copy of out RGB image and then modify it
		
		//! ROS Parameters
		float threshold_brightness_;	//! Higher -> Bright
		float threshold_darkness_;		//! Lower -> Dark

		//! ROS Publisher
		ros::Publisher env_pub_;

		//! ROS Subscriber
		ros::Subscriber rgb_sub_;

		//! Random Functions
		void initParams 	(void);
		void initValues		(void);
		void rgbCallback	(const sensor_msgs::Image::ConstPtr &msg_rgb);
		void averageValues	(void);
		void caseCheck		(void);

	public:
		RecognitionRgb 		(void);
		~RecognitionRgb 	(void);
};
}

#endif