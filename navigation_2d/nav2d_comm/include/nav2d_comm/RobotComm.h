#ifndef COMM_H
#define COMM_H

#define NODE_NAME     "communication"
#define PWR_CONTROL 0
#define BMS_CONTROL 1

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <string>
#include <unistd.h>     /*Unix標準函數定義*/
#include <linux/reboot.h>
#include <stdio.h>
#include <stdlib.h>
#include <linux/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>

class RobotComm
{
public:
	// Default Constructor & Destructor
	RobotComm(){}
	RobotComm(int type);
	~RobotComm();
	int readBMS();
	bool readPWR(int updateBattery);
	

private:
	ros::Publisher mBatteryPublisher;
	std::string mPortname;
	char mData[100];
	int mFile;
	int mCommType;

};

#endif
