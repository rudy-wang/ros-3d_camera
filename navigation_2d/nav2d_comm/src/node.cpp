#include <ros/ros.h>
#include <nav2d_comm/RobotComm.h>

using namespace ros;

int main(int argc, char **argv)
{
	init(argc, argv, NODE_NAME);
	NodeHandle n;

	RobotComm bmsinfo(BMS_CONTROL);
	RobotComm pwrctrl(PWR_CONTROL);

	int updateBattery = 0;
	double lasttime = Time::now().toSec()-295;
	
	//Rate loopRate(0.003);
	Rate loopRate(1);
	while(ok())
	{
		if(Time::now().toSec() - lasttime >= 300)
		{
			updateBattery = bmsinfo.readBMS();
			lasttime = Time::now().toSec();
		}
		else
		{
			updateBattery = 0;
		}
		pwrctrl.readPWR(updateBattery);
		spinOnce();
		loopRate.sleep();
	}
	return 0;	
}
