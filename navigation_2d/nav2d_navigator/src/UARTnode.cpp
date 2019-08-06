#include <unistd.h>     /*Unix標準函數定義*/
#include <stdio.h>
#include <stdlib.h>
#include <linux/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>

#include <ros/ros.h>

/* My Arduino is on /dev/ttyUSB0 */
char *portname = "/dev/ttyUSB0";
char buf[256];

void receiveLiftStatus(const std_msgs::UInt8::ConstPtr& msg)
{
	nav2d_navigator::LiftGoal goal;
	goal.cmd = false;
	goal.target_status = msg->data;
	gLiftClient->sendGoal(goal);
}

int main(int argc, char **argv)
{
	int fd;
	int n = 0, serialLines;
	/* Open the file descriptor in non-blocking mode */
	fd = open(portname, O_RDONLY | O_NOCTTY);
 
	/* Set up the control structure */
	struct termios toptions;
 
	/* Get currently set options for the tty */
	tcgetattr(fd, &toptions);
 
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
	toptions.c_cc[VTIME] = 20;

	cfmakeraw(&toptions);

	/* Flush anything already in the serial buffer */
	tcflush(fd, TCIFLUSH);

	/* commit the options */
	tcsetattr(fd, TCSANOW, &toptions);
 
	ros::init(argc, argv, "UART");
	ros::NodeHandle n;
	
	ros::Subscriber statusSubscriber = n.subscribe("liftStatus", 1, &receiveLiftStatus);

	gLiftClient = new LiftClient(NAV_LIFT_ACTION, true);
	gLiftClient->waitForServer();
	
	ros::spin();
	
	delete gLiftClient;
	return 0;
}
