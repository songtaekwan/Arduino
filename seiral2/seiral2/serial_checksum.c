#include <stdio.h>
#include <stdlib.h>


//multi thread
#include <pthread.h>

#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/poll.h>
#include <termios.h>                   // B115200, CS8 등 상수 정의
#include <fcntl.h>                     // O_RDWR , O_NOCTTY 등의 상수 정의
#include <time.h>
#include <math.h>
#include <errno.h> // Error integer and strerror() function
#include <unistd.h> // write(), read(), close()

#define _USE_MATH_DEFINES

typedef unsigned char BYTE;

#define DEG2RAD(x) (M_PI/180.0*(x) )
#define RAD2DEG(x) ((x)*180.0/M_PI)
#define RPM2RPS(x) ((x)/60) 
#define RPS2RPM(x) ((x)*60) 

union
{
    float data ;
    char  bytedata[4];
    
} m_robot_speed;

union
{
    short data ;
    char  bytedata[2];
    
} m_robot_angle, crc_check;

#define DEG2RAD(x) (M_PI/180.0*(x) )
#define RAD2DEG(x) ((x)*180.0/M_PI)
#define RPM2RPS(x) ((x)/60) 
#define RPS2RPM(x) ((x)*60) 



#define BAUDRATE   B115200
#define SERIAL_DEVICE   "/dev/ttyUSB0"  
//#define SERIAL_DEVICE   "/dev/ttyS0"  
//#define SERIAL_DEVICE   "/dev/ttyAMA0"  

static int uart_fd;
unsigned char protocal_test[12] ={0,};
unsigned char read_buf [20];

void write_serial(unsigned char *buf, int len)
{
   write(uart_fd, &buf[0], len);
} 

int init_serial_port(void)
{
  int serial_port = open(SERIAL_DEVICE, O_RDWR);

  // Create new termios struct, we call it 'tty' for convention
  struct termios tty;

  // Read in existing settings, and handle any error
  if(tcgetattr(serial_port, &tty) != 0) {
      printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
      return 1;
  }

  tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
  tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
  tty.c_cflag |= CS8; // 8 bits per byte (most common)
  tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo
  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
  // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

  tty.c_cc[VTIME] = 100;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 0;

  // Set in/out baud rate to be 9600
  cfsetispeed(&tty, BAUDRATE);
  cfsetospeed(&tty, BAUDRATE);

  // Save tty settings, also checking for error
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
      printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
      return -1;
  }
    
  else
  {
      return serial_port;
  } 
}

void *readserial_thread(void *pt)
{
    
    int num_bytes = -1;
    unsigned char insert_buf; 
    int cnt=0;

    while(1)
    { 
        while( (num_bytes = read(uart_fd, &insert_buf, 1)   ) > 0 )   
        {
            for(int i=0;i<13;i++)
			{
				read_buf[i]=read_buf[i+1];
				cnt++;
			}
			
			read_buf[13]=insert_buf;
			
			if(read_buf[cnt]=='*')
			{
				if(read_buf[cnt-8]=='#')
				{
				    if(read_buf[cnt-7]=='I')
				    {
						for(int i=cnt-7; i<cnt-2; i++)
						{
							crc_check.data += read_buf[i];
						}
						
						if(crc_check.bytedata[0] == read_buf[cnt-1] && crc_check.bytedata[1] == read_buf[cnt-2])
						{
							printf("correct! %d %d\n\n", read_buf[cnt-1], read_buf[cnt-2]);
							crc_check.data = 0;
						}
						else
						{
							printf("wrong\n\n");
							crc_check.data = 0;
					    }
					}
			    }
			}
			cnt=0;
       
			  // for(int i=0;i<10;i++)       printf("%c", read_buf[i]);
			  // printf("\n");
        }
    }
}  

/*
void send_serial_data(void)
{
    unsigned short protocal_crc16;
   
    
    protocal_test[0] = '#';
    protocal_test[1] = 's';
    
    protocal_test[2] = m_robot_angle.bytedata[0];
    protocal_test[3] = m_robot_angle.bytedata[1];
    
    protocal_test[4] = m_robot_speed.bytedata[0];
    protocal_test[5] = m_robot_speed.bytedata[1];
    protocal_test[6] = m_robot_speed.bytedata[2];
    protocal_test[7] = m_robot_speed.bytedata[3];
    
    protocal_test[8] = '*';
    
    //printf("protocal CRC16 %X \n", protocal_crc16);
   
    write_serial(protocal_test,9);   
}
*/

int main(void)
{
  uart_fd = init_serial_port();  
   
  pthread_t id_1;
  
  int ret1=pthread_create(&id_1,NULL,readserial_thread,NULL);    

  m_robot_angle.data = 0;
  m_robot_speed.data = 0.00;
        
  while(1)
  {
      m_robot_angle.data--;
      m_robot_speed.data -=0.01;    
     // send_serial_data();    
      
     // printf("%x %x  %x %x  %x %x\n",m_robot_angle.bytedata[0],m_robot_angle.bytedata[1], m_robot_speed.bytedata[0], m_robot_speed.bytedata[1],m_robot_speed.bytedata[2], m_robot_speed.bytedata[3] ); 
      sleep(2); 
  }
  close(uart_fd);
  return 0;
  
}
