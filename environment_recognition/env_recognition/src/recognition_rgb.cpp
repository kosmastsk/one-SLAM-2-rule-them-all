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
	sum_edges_ = 0;
	temp_env_ = ""; 
	index_ = 0;
	samples_ = 0;

	for(int i; i<100; i++)
	{
		list_gray_[i] = 0;
		list_edges_[i] = 0;
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

	if (n_.hasParam(ros::this_node::getName() + "/threshold_complexity"))
	{
		n_.getParam(ros::this_node::getName() + "/threshold_complexity", threshold_complexity_);
	}  
	else 
	{
		ROS_WARN("[Parameters] Parameter threshold_complexity not found.\
			Using Default");
		threshold_complexity_ = 2.0;
	}
}

/*
**********************************
*    Get data from rgb camera    * 
**********************************
*/

void RecognitionRgb::rgbCallback (const sensor_msgs::Image::ConstPtr &msg_rgb)
{
    try
    {
    	src_ = cv_bridge::toCvCopy(msg_rgb, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      	ROS_ERROR("cv_bridge exception: %s", e.what());
      	return;
    }
    
    //! RGB -> Gray
    cvtColor(src_->image, src_gray_, COLOR_BGR2GRAY);
    
	int sum = 0;
	int size = src_gray_.rows * src_gray_.cols;

	//! Brightness detection
	for(int i=0; i<size; i++)
		sum += src_gray_.data[i];

	temp_gray_ = (float)sum/size;

	//! Edges detection
	blur(src_gray_, detected_edges_, Size(3,3));
    Canny(detected_edges_, detected_edges_, lowThreshold_, lowThreshold_*ratio_, kernel_size_);
    dst_ = Scalar::all(0);
    src_->image.copyTo(dst_, detected_edges_);

    temp_edges_ = 0;
    for(int i = 0; i < dst_.rows; ++i) 
        for(int j = 0; j < dst_.cols; j++)
            if (dst_.at<float>(i,j) != 0)
                ++temp_edges_;

    temp_edges_ = 100 * ((temp_edges_)/((dst_.rows) * (dst_.cols)));

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

	//! Temp variable to store the element of the list that is going to be overriden
	float previous;

	//! Calculate average gray values so far
	previous = list_gray_[index_];
	list_gray_[index_] = temp_gray_;
	sum_gray_ += list_gray_[index_] - previous;
	average_gray_ = sum_gray_/samples_;

	//! Calculate average edges so far
	previous = list_edges_[index_];
	list_edges_[index_] = temp_edges_;
	sum_edges_ += list_edges_[index_] - previous;
	average_edges_ = sum_edges_/samples_;

	ROS_INFO("*[index_] %d", index_);
	ROS_INFO("*[samples_] %d", samples_);
	ROS_INFO("*[temp_gray_] %.2f", temp_gray_);
	ROS_INFO("*[average_gray_] %.2f", average_gray_);
	ROS_INFO("*[temp_edges_] %.2f%%", temp_edges_);
	ROS_INFO("*[average_edges_] %.2f%%", average_edges_);
	ROS_INFO("*************************************************");
	
	//! Raise the index
	index_++;
	caseCheck();
}

void RecognitionRgb::caseCheck (void)
{
	string brightness;
	if(average_gray_ < threshold_darkness_)
		brightness = "|| Brightness: Dark   ";
	else if(average_gray_ > threshold_brightness_)
		brightness = "|| Brightness: Bright ";
	else
		brightness = "|| Brightness: Normal ";

	string complexity;
	if(average_gray_ < threshold_complexity_)
		complexity = "|| Complexity: Simple  ||";
	else
		complexity = "|| Complexity: Complex ||";

	//temp_env_ = brightness;
	temp_env_ = brightness + complexity;
	if(previous_ != temp_env_)
	{
		previous_ = temp_env_;
		msg_env_.data = temp_env_.c_str();
		env_pub_.publish(msg_env_);
	}
}

}