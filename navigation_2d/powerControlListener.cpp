#include <unistd.h>     /*Unix標準函數定義*/
#include <linux/reboot.h>
#include <stdio.h>
#include <stdlib.h>
#include <linux/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>

/* My Arduino is on /dev/ttyUSB0 */
char *portname = "/dev/ttyUSB0";
char buf[256];
 
int main(int argc, char *argv[])
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
 
 /* read up to 128 bytes from the fd */
 while(1)
 {
  if((n = read(fd,buf,sizeof(buf)))>0)
  {
   buf[n+1]='\0';
   if(n==3&&buf[0]=='o'&&buf[1]=='f'&&buf[2]=='f')
   {
    //close(fd);
    //system("shutdown -r now");
   }
  std::cout << buf << std::endl;
  }
 }
 
 return 0;
}
