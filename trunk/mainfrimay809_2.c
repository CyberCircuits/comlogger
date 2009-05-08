//Arduino-serial Tod E. Kurt, tod@todbot.com http://todbot.com/blog/
 
#include <stdio.h> /* Standard input/output definitions */
#include <stdlib.h> 
#include <stdint.h> /* Standard types */
#include <string.h> /* String function definitions */
#include <unistd.h> /* UNIX standard function definitions */
#include <fcntl.h> /* File control definitions */
#include <errno.h> /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <sys/ioctl.h>
#include <getopt.h>

int serialport_init(const char* serialport, int baud);
int serialport_read_until(int fd, char* buf, char until);

int main() 
{
 int fd = 0;
 char serialport[256];
 int baudrate = B115200; // default
 char buf[256];

char *optarg = "/dev/ttyUSB1\0";
 strcpy(serialport,optarg);
 fd = serialport_init(optarg, baudrate);

//fd = serialport_init(optarg, baudrate);
		
usleep(2000 * 1000 ); // sleep milliseconds
serialport_read_until(fd, buf, '\n');
 printf("read: %s\n",buf);
	
 exit(EXIT_SUCCESS); 
} // end main
 
int serialport_read_until(int fd, char* buf, char until)
{
 char b[1];
 int i=0;
 do { 
 int n = read(fd, b, 1); // read a char at a time
 if( n==-1) return -1; // couldn't read
 if( n==0 ) {
 usleep( 10 * 1000 ); // wait 10 msec try again
 continue;
 }
 buf[i] = b[0]; i++;
 } while( b[0] != until );

 buf[i] = 0; // null terminate the string
 return 0;
}

// takes the string name of the serial port (e.g. "/dev/tty.usbserial","COM1")
// and a baud rate (bps) and connects to that port at that speed and 8N1.
// opens the port in fully raw mode so you can send binary data.
// returns valid fd, or -1 on error
int serialport_init(const char* serialport, int baud)
{
 struct termios toptions;
 int fd;
 
 //fprintf(stderr,"init_serialport: opening port %s @ %d bps\n",
 // serialport,baud);

//char *filename = "/dev/ttyUSB1\0";
	
 fd = open(serialport, O_RDWR | O_NOCTTY | O_NDELAY);
 if (fd == -1) {
 perror("init_serialport: Unable to open port ");
 return -1;
 }
 
 if (tcgetattr(fd, &toptions) < 0) {
 perror("init_serialport: Couldn't get term attributes");
 return -1;
 }
 speed_t brate = baud;
 cfsetispeed(&toptions, brate);
 cfsetospeed(&toptions, brate);

 // 8N1
 toptions.c_cflag &= ~PARENB;
 toptions.c_cflag &= ~CSTOPB;
 toptions.c_cflag &= ~CSIZE;
 toptions.c_cflag |= CS8;
 // no flow control
 toptions.c_cflag &= ~CRTSCTS;

 toptions.c_cflag |= CREAD | CLOCAL; // turn on READ & ignore ctrl lines
 toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl

 toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
 toptions.c_oflag &= ~OPOST; // make raw

 toptions.c_cc[VMIN] = 0;
 toptions.c_cc[VTIME] = 20;
 
 if( tcsetattr(fd, TCSANOW, &toptions) < 0) {
 perror("init_serialport: Couldn't set term attributes");
 return -1;
 }

 return fd;
}