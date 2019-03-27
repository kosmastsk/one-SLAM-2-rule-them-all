#include <env_recognition/recognition_depth.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "recognition_depth");

	env_recognition::RecognitionDepth obj;

	ros::spin();
	return 0;
}