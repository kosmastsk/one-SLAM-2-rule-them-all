#include <env_recognition/recognition_rgb.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "recognition_rgb");

	env_recognition::RecognitionRgb obj;

	ros::spin();
	return 0;
}