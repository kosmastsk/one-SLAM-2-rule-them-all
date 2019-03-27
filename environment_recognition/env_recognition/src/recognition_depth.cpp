#include <env_recognition/recognition_depth.h>

namespace env_recognition
{ 

/*
*********************
*    Constructor    * 
*********************
*/

RecognitionDepth::RecognitionDepth (void)
{
	initParams();
	initValues();

	rgb_sub_ = n_.subscribe("/camera/depth/image_raw", 10, &RecognitionDepth::depthCallback, this);
	env_pub_ = n_.advertise<std_msgs::String>("/environment", 10);
}

/*
********************
*    Destructor    *  
********************
*/

RecognitionDepth::~RecognitionDepth (void)
{
	ROS_INFO("[RecognitionDepth] Node destroyed.");
}

/*
*****************************************
*    Initialize the necessary values    *  
*****************************************
*/

void RecognitionDepth::initValues (void)
{
	counter_msgs_ = 0;
	sum_edges_ = 0;
	temp_env_ = ""; 
	index_ = 0;
	samples_ = 0;

	for(int i; i<100; i++)
		list_edges_[i] = 0;
}

/*
***********************************
*    Initialize the parameters    *  
***********************************
*/

void RecognitionDepth::initParams (void)
{
	if (n_.hasParam(ros::this_node::getName() + "/threshold_complexity"))
	{
		n_.getParam(ros::this_node::getName() + "/threshold_complexity", threshold_complexity_);
	}  
	else 
	{
		ROS_WARN("[Parameters] Parameter threshold_complexity not found.\
			Using Default");
		threshold_complexity_ = 1.0;
	}
}

/*
************************************
*    Get data from depth camera    * 
************************************
*/

void RecognitionDepth::depthCallback (const sensor_msgs::Image::ConstPtr &msg_rgb)
{
    try
    {
    	ptr_ = cv_bridge::toCvCopy(msg_rgb, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
      	ROS_ERROR("cv_bridge exception: %s", e.what());
      	return;
    }
    
    // Depth -> CV_8U
    ptr_->image.convertTo(src_, CV_8U);

	//! Edges detection
	blur(src_, detected_edges_, Size(3,3));
    Canny(detected_edges_, detected_edges_, lowThreshold_, lowThreshold_*ratio_, kernel_size_);
    dst_ = Scalar::all(0);
    ptr_->image.copyTo(dst_, detected_edges_);

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

void RecognitionDepth::averageValues (void)
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

	//! Calculate average edges so far
	previous = list_edges_[index_];
	list_edges_[index_] = temp_edges_;
	sum_edges_ += list_edges_[index_] - previous;
	average_edges_ = sum_edges_/samples_;

	ROS_INFO("*[index_] %d", index_);
	ROS_INFO("*[samples_] %d", samples_);
	ROS_INFO("*[temp_edges_] %.2f%%", temp_edges_);
	ROS_INFO("*[average_edges_] %.2f%%", average_edges_);
	ROS_INFO("*************************************************");
	
	//! Raise the index
	index_++;
	caseCheck();
}

void RecognitionDepth::caseCheck (void)
{
	string complexity;
	if(average_edges_ < threshold_complexity_)
		complexity = "|| Depth Complexity: Simple  ||";
	else
		complexity = "|| Depth Complexity: Complex ||";

	temp_env_ = complexity;
	if(previous_ != temp_env_)
	{
		previous_ = temp_env_;
		msg_env_.data = temp_env_.c_str();
		env_pub_.publish(msg_env_);
	}
}

}