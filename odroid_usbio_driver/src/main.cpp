#include <stdio.h>
#include <iostream>
#include <string.h>
#include <stdlib.h>

#include <sys/types.h>
#include <sys/time.h>
#include <unistd.h>
#include "hidapi.h"
// external includes
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#define MAX_STR 65
#define TRUE 1
#define FALSE 0

unsigned char buf[MAX_STR];
int toggleLeds;
int prev_toggleLeds;
int potentiometerValue;
int pushbuttonStatus;
int getRval;
int pressed;
int prev_pressed;

hid_device* dev_open() {

	hid_device *device;

	device = hid_open(0x04D8, 0x003F, NULL);

	if(device) {
		printf("HID Device opened\n");
		hid_set_nonblocking(device, TRUE);
	} else
		printf("HID Device open failed\n");

	return device;
}

void closeDevice(hid_device *device) {
	hid_close(device);
}

void checkmSec(int msec) {
	struct timeval ts, te;
	long tdiff, ms;

	gettimeofday(&ts, NULL);

	while(1) {
		gettimeofday(&te, NULL);

		tdiff = (1000000*(te.tv_sec - ts.tv_sec)) + (te.tv_usec - ts.tv_usec);

		ms = tdiff / 1000;

		if(ms > msec)
			break;

	}
}

void rwUSB(hid_device *device) {

	unsigned char p1 = buf[10];
	unsigned char p2 = buf[11];
//	float volt = 0;
//	int i = 0;

	if(toggleLeds == TRUE) {
		
		toggleLeds = FALSE;

		buf[1] = 0x80;

		if (hid_write(device, buf, sizeof(buf)) == -1) {
			closeDevice(device);
			return;
		}

		buf[0] = 0x00;
		buf[1] = 0x37;
		memset((void*)&buf[2], 0x00, sizeof(buf) - 2);

		return ;
	}
	
	do {
		if(hid_write(device, buf, sizeof(buf)) == -1) {
			closeDevice(device);
			return ;
		}

		usleep(50000);

		if(hid_read(device, buf, sizeof(buf)) == -1) {
			closeDevice(device);
			return;
		}

		if(buf[0] == 0x37) {
			potentiometerValue = (buf[1]<<2) + ((buf[2]&0xF0)>>2);
			buf[1] = 0x81;
		
			printf("potentiometerValue = %d\n", potentiometerValue);
		}

		if(buf[0] == 0x81) {
//			printf("Pushbutton State : %s  %d\n", buf[1]?"Not Pressed":"Pressed",  buf[1]);
			prev_pressed = pressed;
			pressed = (int)buf[1];
//			if (!(int)buf[1]) toggleLeds = TRUE;
//			else printf("Pushbutton State : %c\n", buf[1]);
		}

		if(buf[0] == 0x98) {
			getRval = buf[1];

			printf("Got value (0x%x) at SFR address (0x%x)\n", getRval, p1);
		}

		if(buf[0] == 0x99) {
			getRval = buf[1];

			printf("Wrote (0x%x) at SFR address (0x%x)\n", getRval, p1);
		}

		if(buf[0] == 0x9A) {
			getRval = buf[1];

			printf("Got value (%d) at SFR address (0x%x) (%dbit)\n", getRval, p1, p2);
		}

		if(buf[0] == 0x9B) {
			getRval = buf[1];

			printf("Wrote (%d) at SFR address (0x%x) (%dbit)\n", getRval, p1, p2);
		}

	} while(0);
}

int main(int argc, char* argv[] ) {
	
 	sleep(15);

	ros::init(argc, argv, "gpio");
	ros::NodeHandle n;

	ros::Publisher pub = n.advertise<std_msgs::Bool>("camera_on", 1000);
	//int result = 0;
//	int i = 0;
	//unsigned char s = 0;
	//unsigned int inum[3] = {0, 0, 0};
	hid_device *device;

	device = dev_open();

	if(!device)
		return -1;

	pushbuttonStatus = FALSE;
	potentiometerValue = 0;
	toggleLeds = 0;
	pressed = 1;
	prev_pressed = 1;
	bool camera_on = false;

	
	//result = system("clear");

	ros::Time begin, end;
					
	while(ros::ok()) {
			
			/*printf("\n");
			printf("a. Toggle LED\n");
			printf("b. AN0/POT Value\n");
			printf("c. Button status\n");
			printf("d. Get Register\n");
			printf("e. Set Register\n");
			printf("f. Get Register Bit\n");
			printf("g. Set Register Bit\n");
			printf("q. Exit\n");
			printf("\n");

			result = scanf("%s", &s);

			buf[0] = 0x00;
			memset((void*)&buf[2], 0x00, sizeof(buf) - 2);

			switch(s) {
				case 'a':
					toggleLeds = TRUE;
					break;

				case 'b':
					buf[0] = 0x37;
					break;

				case 'c':
					buf[0] = 0x81;
					break;

				case 'd':
					printf("Input LSB a byte of SFR address to read : 0x");
					result = scanf("%x", inum);
					buf[0] = 0x98;
					buf[10] = inum[0];
					break;

				case 'e':
					printf("Input LSB a byte of SFR Address to write : 0x");
					result = scanf("%x", &inum[0]);
					printf("Input value : 0x");
					result = scanf("%x", &inum[1]);
					buf[0] = 0x99;
					buf[10] = inum[0];
					buf[11] = inum[1];
					break;

				case 'f':
					printf("Input LSB a byte of SFR address to read : 0x");
					result = scanf("%x", &inum[0]);
					printf("Which Bit(0 to 7) : ");
					result = scanf("%x", &inum[1]);
					buf[0] = 0x9A;
					buf[10] = inum[0];
					buf[11] = inum[1];
					break;

				case 'g':
					printf("Input LSB a byte of SFR address to write : 0x");
					result = scanf("%x", &inum[0]);
					printf("Which Bit(0 to 7) : ");
					result = scanf("%x", &inum[1]);
					printf("Input value(0 or 1) : ");
					result = scanf("%x", &inum[2]);
					buf[0] = 0x9B;
					buf[10] = inum[0];
					buf[11] = inum[1];
					buf[12] = inum[2];
					break;

				case 'q':
				default: 
					printf("Bye~~\n");
					closeDevice(device);
					return 0;
		}
		rwUSB(device);*/


//			result = scanf("%s", &s);


//			begin = ros::Time::now();

			buf[0] = 0x00;
			memset((void*)&buf[2], 0x00, sizeof(buf) - 2);

			if (pressed && !prev_pressed){
				toggleLeds = TRUE;
				prev_pressed = pressed;
				camera_on = !camera_on;
				std_msgs::Bool msg;
				msg.data = camera_on;
				pub.publish(msg);
				
			}
			else {
				buf[0] = 0x81;
			}

			rwUSB(device);

			usleep(48000);

//			end = ros::Time::now();
//			std::cout << (end-begin).toSec() << std::endl;
	}

	closeDevice(device);

	return 0;
}
