#include <ros/ros.h>

#include <nav2d_tfpub/RobotTFpub.h>

using namespace ros;

int main(int argc, char **argv)
{
	init(argc, argv, NODE_NAME);
	NodeHandle n;

	RobotTFpub robTf;
	
	Rate loopRate(40);
	while(ok())
	{
		spinOnce();
		loopRate.sleep();
	}
	return 0;	
}
