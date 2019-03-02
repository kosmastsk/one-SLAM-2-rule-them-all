#include <env_recognition/recognition_rgb.h>

namespace env_recognition
{ 

/*
*********************
*    Constructor    * 
*********************
*/

RecognitionRgb::RecognitionRgb (void)
{
	initParams();
	initValues();

	rgb_sub_ = n_.subscribe("/camera/rgb/image_raw", 10, &RecognitionRgb::rgbCallback, this);
	env_pub_ = n_.advertise<std_msgs::String>("/environment", 10);
}

/*
********************
*    Destructor    *  
********************
*/

RecognitionRgb::~RecognitionRgb (void)
{
	ROS_INFO("[RecognitionRgb] Node destroyed.");
}

/*
*****************************************
*    Initialize the necessary values    *  
*****************************************
*/

void RecognitionRgb::initValues (void)
{
	counter_msgs_ = 0;
	sum_gray_ = 0;
	temp_env_ = ""; 
	index_ = 0;
	samples_ = 0;

	for(int i; i<100; i++)
	{
		list_gray_[i] = 0;
	}
}

/*
***********************************
*    Initialize the parameters    *  
***********************************
*/

void RecognitionRgb::initParams (void)
{
	if (n_.hasParam(ros::this_node::getName() + "/threshold_brightness"))
	{
		n_.getParam(ros::this_node::getName() + "/threshold_brightness", threshold_brightness_);
	}  
	else 
	{
		ROS_WARN("[Parameters] Parameter threshold_brightness not found.\
			Using Default");
		threshold_brightness_ = 180.0;
	}

	if (n_.hasParam(ros::this_node::getName() + "/threshold_darkness"))
	{
		n_.getParam(ros::this_node::getName() + "/threshold_darkness", threshold_darkness_);
	}  
	else 
	{
		ROS_WARN("[Parameters] Parameter threshold_darkness not found.\
			Using Default");
		threshold_darkness_ = 80.0;
	}
}

/*
**************************************************
*    Get data from rgb camera and modify them    * 
**************************************************
*/

void RecognitionRgb::rgbCallback (const sensor_msgs::Image::ConstPtr &msg_rgb)
{
	//! Convert the image from RGB to Grayscale
    try
    {
    	cv_ptr_ = cv_bridge::toCvCopy(msg_rgb, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      	ROS_ERROR("cv_bridge exception: %s", e.what());
      	return;
    }
    
    cv::Mat grayMat;
    cv::cvtColor(cv_ptr_->image, grayMat, cv::COLOR_BGR2GRAY);
    
    //! Calculate the average of all the grayscale values of the current measurement
	int sum = 0;
	int size = cv_ptr_->image.rows * cv_ptr_->image.cols;

	for(int i=0; i<size; i++)
		sum += cv_ptr_->image.data[i];

	temp_gray_ = (float)sum/(float)size;
	averageValues();
}

/*
************************************************
*    Calculate the average values until now    *  
************************************************
*/

void RecognitionRgb::averageValues (void)
{
	//! For the first 100 messages, raise the samples_ by 1. Then, keep it to 100
	//! Also, reset the index of the list (index_), everytime it reaches 100
	if(counter_msgs_%100 == 0)
		index_ = 0;
	if(counter_msgs_ < 100)
		samples_++;
	counter_msgs_++;

	//! This variable to store the element of the list that is going to be overriden
	float previous;

	//! Calculate average grayscale values so far
	previous = list_gray_[index_];
	list_gray_[index_] = temp_gray_;
	sum_gray_ += list_gray_[index_] - previous;
	average_gray_ = sum_gray_ / (float)samples_;

	ROS_INFO("*[index_] %d", index_);
	ROS_INFO("*[samples_] %d", samples_);
	ROS_INFO("*[temp_gray_] %.2f", temp_gray_);
	ROS_INFO("*[average_gray_] %.2f", average_gray_);
	ROS_INFO("*************************************************");
	
	//! Raise the index
	index_++;
	caseCheck();
}

/*
*******************************************************************
*    Check the type of environment we are into, and publish it    *  
*******************************************************************
*/

void RecognitionRgb::caseCheck (void)
{
	string brightness;
	if(average_gray_ < threshold_darkness_)
		brightness = "|| Brightness: Dark   ||";
	else if(average_gray_ > threshold_brightness_)
		brightness = "|| Brightness: Bright ||";
	else
		brightness = "|| Brightness: Normal ||";

	//temp_env_ = brightness;
	temp_env_ = brightness;
	if(previous_ != temp_env_)
	{
		previous_ = temp_env_;
		msg_env_.data = temp_env_.c_str();
		env_pub_.publish(msg_env_);
	}
}

}