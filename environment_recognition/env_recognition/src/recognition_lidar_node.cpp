#include <env_recognition/recognition_lidar.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "recognition_lidar");

	env_recognition::RecognitionLidar obj;

	ros::spin();
	return 0;
}