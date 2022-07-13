
//*******************************************************************************
// *  Functions
// *******************************************************************************
// *
//*******************************************************************************
// *  INCLUDE #define POLYNORMIAL 0xA001FILES
//*******************************************************************************
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

//*******************************************************************************
// *  Defines
//*******************************************************************************

#define _USE_MATH_DEFINES
typedef unsigned char BYTE;
#define DEG2RAD(x) (M_PI/180.0*(x) )
#define RAD2DEG(x) ((x)*180.0/M_PI)
#define RPM2RPS(x) ((x)/60)
#define RPS2RPM(x) ((x)*60)

union
{
    float data;
    char  bytedata[4];
} data;

union
{
    short data ;
    char  bytedata[2];
} checksum, checksum2;


#define DEG2RAD(x) (M_PI/180.0*(x) )
#define RAD2DEG(x) ((x)*180.0/M_PI)
#define RPM2RPS(x) ((x)/60)
#define RPS2RPM(x) ((x)*60)
#define BAUDRATE   B9600
//#define SERIAL_DEVICE   "/dev/ttyUSB0"
#define SERIAL_DEVICE   "/dev/ttyS0"

static int uart_fd;
unsigned char protocal_test[9] = {0,};
unsigned char protocal_receive[9] = {0,};
unsigned char read_buf[20];

char cnt_rcv = 0;
short check = 0;


void write_serial(unsigned char *buf, int len)
{
	write(uart_fd, buf, len);
}




int init_serial_port(void)
{
  int serial_port = open("/dev/ttyUSB0", O_RDWR);
  if(serial_port < 0)
  {
	  printf("Error %i from open: %s\n", errno, strerror(errno));
      exit(1);
  }
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
    while(1)
    {
		
     	while( (num_bytes = read(uart_fd, &insert_buf, 1)   ) > 0 )	
       {
            printf("receive data : %x\n", insert_buf);
			
			//아두이노에 잘못된 데이터가 전송돼 wrong 메시지 받음
			if(insert_buf == 'w')
			{
				cnt_rcv = 0;
				printf("receive wrong\n");				
			}
			
			//정상 데이터 전송돼 동일 데이터 수신
			else
			{
				protocal_receive[cnt_rcv] = insert_buf; //0~8
			
				if(protocal_receive[cnt_rcv] == '*')
				{
					//수신받은 데이터의 형태가 맞을때 #F0000checksum1,2*
					if(protocal_receive[(cnt_rcv+1)%9] == '#' && protocal_receive[(cnt_rcv+2)%9] == 'F')
					{
						printf("packet\n");	
						for(int i = 2; i < 7; i++)
						{
							check += protocal_receive[(cnt_rcv + i)%9];
						}
						checksum2.data = check;
						checksum2.data &= 0xFF;
						checksum2.data = ~checksum2.data + 1;
						checksum2.data += check;
						checksum2.data &= 0xFF;
						check = 0;
						
						//체크썸 맞을 때 다음 데이터 송신
						if(checksum2.bytedata[0] == protocal_receive[cnt_rcv-1] && checksum2.bytedata[1] == protocal_receive[cnt_rcv-2])
						{
							cnt_rcv = 0;
							data.data += 0.01;
							printf("good\n");				
						}
						
						//체크썸 틀릴때 원래 데이터 송신
						else
						{
							cnt_rcv = 0;
							printf("bad\n");	
						}
					}
					//수신받은 데이터의 형태가 아닐때 #F0000checksum1,2*
					else
					{
						printf("trash\n");	
						cnt_rcv = 0;
					}
				}
				cnt_rcv++;
				cnt_rcv %= 9;
			}
        }
    }
}






void send_serial_data(void)
{
    unsigned short protocal_crc16;
	
    protocal_test[0] = '#';
    protocal_test[1] = 'F';
    protocal_test[2] = data.bytedata[3];
    protocal_test[3] = data.bytedata[2];
    protocal_test[4] = data.bytedata[1];
    protocal_test[5] = data.bytedata[0];
    
    for(int i = 1; i < 6; i++)
    {
		check += protocal_test[i];	
	}
	checksum.data = check;
	checksum.data &= 0xFF;
	checksum.data = ~checksum.data + 1;
	checksum.data += check;
	checksum.data &= 0xFF;
	check = 0;
	
    protocal_test[6] = checksum.bytedata[1];
    protocal_test[7] = checksum.bytedata[0];
    protocal_test[8] = '*';
    //printf("protocal CRC16 %X \n", protocal_crc16);
	
//	printf("data : %f\n", data.data);
	printf("checksum : %d %x %x \n\n", checksum.data, checksum.bytedata[0], checksum.bytedata[1]);
	
//	printf("start : %c %c\n", protocal_test[0] ,protocal_test[1] );
//	printf("data : %2x %2x %2x %2x \n", protocal_test[2], protocal_test[3], protocal_test[4], protocal_test[5]);
//	printf("checksum : %2X %2X\n", protocal_test[6], protocal_test[7]);
//	printf("end : %c\n\n\n", protocal_test[8]);
	
    write_serial(protocal_test, 9);
}

int main(void)
{
  uart_fd = init_serial_port();
  //memset(&read_buf, '\0', sizeof(read_buf));protocal_test[0] = '#';
  //printf("protocal CRC16 %X \n", protocal_crc1
  //unsigned char msg[] = { 'H', 'e', 'l', 'l', 'o', '\r' };
  //write(uart_fd , msg, sizeof(msg));
  // Normally you wouldn't do this memset() call, but since we will just receive
  // ASCII data for this example, we'll set everything to 0 so we can
  // call printf() easily.
  // Read bytes. The behaviour of read() (e.g. does it block?,
  // how long does it block for?) depends on the configuration
  // settings above, specifically VMIN and VTIME
  pthread_t id_1;
  int ret1=pthread_create(&id_1,NULL,readserial_thread,NULL);
  // Here we assume we received ASCII data, but you might be sending raw bytes (in that case, don't try and
  // print it to the screen like this!)
  checksum.data = 0;
  data.data = 0.01;
	

  send_serial_data();
  sleep(1);
  send_serial_data();
  sleep(1);
  send_serial_data();
  sleep(1);
  send_serial_data();
  sleep(1);
  send_serial_data();
  sleep(1);


      
  close(uart_fd);
  return 0;
}
