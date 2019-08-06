#include <nav2d_comm/RobotComm.h>
#include <omp.h>
#include <errno.h>

#define BMS_REQUEST ":001000000ee9~"

RobotComm::RobotComm(int type)
{	
	ros::NodeHandle n;
	if(type==BMS_CONTROL)
	{
		n.param("bms_port", mPortname, std::string(""));
		mBatteryPublisher = n.advertise<std_msgs::Int8>("battery", 1);
	}
	else if(type==PWR_CONTROL)
	{
		n.param("pwrctrl_port", mPortname, std::string(""));
	}
	mCommType = type;
	/* Open the file descriptor in non-blocking mode */
	mFile = open(mPortname.c_str(), O_RDWR | O_NOCTTY | O_NDELAY );

	/* Set up the control structure */
	struct termios toptions;

	/* Get currently set options for the tty */
	tcgetattr(mFile, &toptions);

	/* Set custom options */

	/* 9600 baud */
	cfsetispeed(&toptions, (speed_t)B9600);
	cfsetospeed(&toptions, (speed_t)B9600);
	/* 8 bits, no parity, no stop bits */
	toptions.c_cflag &= ~PARENB;
	toptions.c_cflag &= ~CSTOPB;
	toptions.c_cflag &= ~CSIZE;
	toptions.c_cflag |= CS8;

	/* no hardware flow control */
	toptions.c_cflag &= ~CRTSCTS;
	/* enable receiver, ignore status lines */
	toptions.c_cflag |= CREAD | CLOCAL;

	toptions.c_lflag     =   0;          // no signaling chars, no echo, no canonical processing
	toptions.c_oflag     =   0;                  // no remapping, no delays
	/* wait for 1 characters to come in before read returns */
	/* WARNING! THIS CAUSES THE read() TO BLOCK UNTIL ALL */
	/* CHARACTERS HAVE COME IN! */
	toptions.c_cc[VMIN] = 0;
	/* no minimum time to wait before read returns */
	toptions.c_cc[VTIME] = 10;

	cfmakeraw(&toptions);

	/* Flush anything already in the serial buffer */
	tcflush(mFile, TCIFLUSH);

	/* commit the options */
	tcsetattr(mFile, TCSANOW, &toptions);
}

RobotComm::~RobotComm()
{
	memset(mData, '\0', sizeof(mData));
	close(mFile);
}
int RobotComm::readBMS()
{
	if(mCommType != BMS_CONTROL)
	{
		ROS_ERROR("Communication object doesn't support this function.");
		return 0;
	}
	int n;
	long int battery_now = 0;
	long int battery_full = 0;
	char buffer[100];

	write(mFile,BMS_REQUEST,strlen(BMS_REQUEST));
	usleep(100000);
	if((n = read(mFile,mData,sizeof(mData)))==30 && mData[3]=='9' && mData[4]=='0' && mData[n-1]=='~')
	{
		mData[n]='\0';

		memset(buffer, '\0', sizeof(buffer));
		memcpy(buffer, &mData[15], 4);
		battery_now = strtol(buffer,NULL,16);

		memset(buffer, '\0', sizeof(buffer));
		memcpy(buffer, &mData[19], 4);
		battery_full = strtol(buffer,NULL,16);

		std_msgs::Int8 rate;
		rate.data = (int)((battery_now*100)/battery_full);
		mBatteryPublisher.publish(rate);

		return rate.data;
	}
	mData[0]='\0';
	return 0;
}

bool RobotComm::readPWR(int updateBattery)
{
	bool trigger = false;
	if(mCommType != PWR_CONTROL)
	{
		ROS_ERROR("Communication object doesn't support this function.");
		return false;
	}
	int n;
	if(updateBattery > 0)
	{
		char rate[10];
		sprintf(rate, "b%d\n", updateBattery);
		write(mFile,rate,strlen(rate));
		trigger = true;
	}

	if((n = read(mFile,mData,sizeof(mData)))>0)
	{
		mData[n]='\0';
		if(n==3 && mData[0]=='o' && mData[1]=='f' && mData[2]=='f')
		{
			memset(mData, '\0', sizeof(mData));
			write(mFile,"p10",3);
			close(mFile);
			system("pkill -SIGINT -f 'roslaunch nav2d_run UAGV.launch'");
			system("shutdown -P now");
		}
		trigger = true;
	}
	mData[0]='\0';
	if(trigger) return true;
	return false;
}
