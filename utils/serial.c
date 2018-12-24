/* Your bible is now: https://www.cmrr.umn.edu/~strupp/serial.html */

#include			<stdlib.h>
#include 			<unistd.h>
#include 			<stdio.h>
#include 			<termios.h>
#include 			<signal.h>
#include 			<sys/ioctl.h>
#include 			<sys/fcntl.h>
#include			<sys/select.h>
#include			<sys/types.h>
#include			<sys/time.h>
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
	char			buf[256];
	char 			c;
	int 			i;
	int 			c_read;
	int 			status;
	int            	n;
	int           	fd;
	fd_set        	rdfs;
	struct timeval 	timeout;

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

	/* Reading loop */
	while (1)
	{
		/* Initialize the input set */
		FD_ZERO(&rdfs);
		FD_SET(fd, &rdfs);
		
		/* Initialize the timeout structure */
		timeout.tv_sec = 3;
		timeout.tv_usec = 0;

		/* Do the select */
		n = select(fd + 1, &rdfs, NULL, NULL, &timeout);
		if (n == -1)
		  perror("select failed");
		else if (n == 0)
		  printf("Select timeout\n");
		else
		{
			/* We have input */
		  	if (FD_ISSET(fd, &rdfs))
		  	{
		  		i = 0;
		  		while (42)
		  		{
					if (read(fd, &c, 1) < 0)
						perror("read failed");
					if (c == 0x03) // 0x03 = ETX
					{
						buf[i] = '\0';
						break;
					}
					if (c >= 0x06) // not a control char
					{
						buf[i] = c;
						i++;
					}
		  		}
		  		printf("%s\n", buf);

			}
		}
	}
}