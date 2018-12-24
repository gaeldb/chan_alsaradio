#include			<stdlib.h>
#include 			<unistd.h>
#include 			<stdio.h>
#include 			<termios.h>
#include 			<signal.h>
#include 			<sys/ioctl.h>
#include 			<sys/fcntl.h>
#include 			<stropts.h>

int					fd;
struct termios		termios_p;

void				handler(int signum)
{
	printf("Exit now (%i).\n", signum);
	close(fd);
	exit(0);
}

int 				main(void)
{
	char			c;
	int 			status;

	/* Catch SIGINT */
  	if (signal(SIGINT, handler) == SIG_IGN)
  		signal(SIGINT, SIG_IGN);

	/* Open port */
	if ((fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY)) == -1 )
	{
		perror("open");
		exit(-1);
	}
	
	/* Read current parameters */
	tcgetattr(fd,&termios_p);

	/* Set baudrate */
	cfsetispeed(&termios_p, B9600);
	cfsetospeed(&termios_p, B9600);

	/* Ignore BREAKs, parity error and disable flow control */
	termios_p.c_iflag &= ~(IXON | IXOFF | IXANY);
	termios_p.c_iflag |= IGNPAR;

	/* Nothing to be set for output... */
	termios_p.c_oflag = 0;

	/* Setting "8N1" controlling with RTS/CTS */
	termios_p.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
	termios_p.c_cflag |= CREAD;
	termios_p.c_cflag |= CS8;

	/* Raw input mode */
	termios_p.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

	/* Read caraters immediatly */
	termios_p.c_cc[VMIN] = 1;
	termios_p.c_cc[VTIME] = 0;

	/* Set new attributes*/
	tcsetattr(fd,TCSANOW,&termios_p);
	
	/* If needed, set DTR inactive. For exemple if PTT is controlled
	by RTS or DTR and PTT is always pushed during exec */
	if ((ioctl(fd, TIOCMGET, &status)) < 0)
    {
    	printf("Cannot get IO parameters");
        exit(-1);
    }
	/* set DTR active and RTS innactive */
	//status |= TIOCM_DTR;
    //status &= ~TIOCM_RTS;
    status |= TIOCM_RTS;
    status &= ~TIOCM_DTR;
	if ((ioctl(fd, TIOCMSET, &status)) < 0)
    {
    	printf("Cannot set IO parameters");
		exit(-1);
    }

	/* Dirty reading loop */
	while (1)
	{
		read(fd, &c, 1);
		//printf("%03u %02x %c\n",c&0xff,c&0xff,c);
		if (c == 0x03) // 0x03 = ETX
			printf("\n");
		else if (c == '\n')
			printf("\\n");
		else if (c == '\r')
			printf("\\r");
		else
			printf("%c", c);
	}
}