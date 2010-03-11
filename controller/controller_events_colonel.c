//
// controller_events.c
//
// This file contains all the user defined code for events triggered by
// a change of state. On the controller side, a change of state occurs
// when the joystick state changes. Or when the program starts and shuts
// down. See common/events.h for more complete information.
//
#define PI 3.1415926535897932384626433832

#include <math.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include "robot_comm.h"
#include "robot_log.h"
#include "joystick.h"
#include "events.h"
#include "profile.h"

int turbo = 0;

void on_init() {
    robot_event ev;
    ev.command = ROBOT_EVENT_CMD_START;
    ev.index = 0;
    ev.value = 0;

	log_string(-1, "Controller is initializing");
	send_event(&ev);
}

void on_shutdown() {
    robot_event ev;
    ev.command = ROBOT_EVENT_CMD_STOP;
    ev.index = 0;
    ev.value = 0;

	log_string(-1, "Controller is shutting down");
	send_event(&ev);
}

void on_button_up(robot_event *ev) {
	if(ev->index == CON_TURBO1 || ev->index == CON_TURBO2)
		turbo--;
	send_event(ev);
}

void on_button_down(robot_event *ev) {
	if(ev->index == CON_TURBO1 || ev->index == CON_TURBO2)
		turbo++;
	send_event(ev);
}

void on_axis_change(robot_event *ev) {
    
	 static int drive = 0, front = 0, rear = 0; //Motor values (for mechanum steering)
	 static unsigned char xAxis = 127, yAxis = 127, rAxis = 127; //x (lateral) y(forward) and r (rotational) axis values
	 static int xNew = 0, yNew = 0, rNew =0, rDrive = 0; // 0 centered axis values
	 int strafe;
	
	 robot_event new_ev;
    unsigned char axis = ev->index;
    unsigned char value = ev->value;
	 if(value == 255) value = 254;
	 if(value == 0) value = 1;
	 
	 send_event(ev);
    
	 if(axis == CON_XAXIS || axis == CON_YAXIS || axis == CON_RAXIS) {
        if(axis == CON_YAXIS)
            yAxis = value;
        if(axis == CON_XAXIS)
            xAxis = value;
        if(axis == CON_RAXIS)
            rAxis = -value;
        xNew = (int)xAxis - 127;
        yNew = (int)yAxis - 127;
        rNew = (int)rAxis - 127;
	rDrive = rNew/127*250;

        if(xNew > 0 && yNew >= 0)
		strafe = (int)((250*atan(yNew/xNew)/(.5*PI)-250)*(-1));
	else if (xNew < 0 && yNew >= 0)
		strafe = (int)((250*atan((-1)*yNew/xNew)/(.5*PI)-250));
	else if (xNew < 0 && yNew >= 0)
		strafe = (int)((250*atan(yNew/xNew)/(.5*PI)+250)*(-1));
	else if (xNew < 0 && yNew >= 0)
		strafe = (int)((250*atan((-1)*yNew/xNew)/(.5*PI)+250));
	else if (xNew == 0 && yNew >= 0)
		strafe = 500; //Translates to 1000 == 0, but thats later)
	else
		strafe = -500;

	if(strafe < 0)
		strafe = -500 - strafe;

	strafe += 500;
	front = strafe + rDrive;
	rear = strafe - rDrive;
	if(front > 999)
		front -= 1000;
	if(front < 0)
		front += 1000;
	if(rear > 999)
		rear -= 1000;
	if(rear < 0)
		rear += 1000;
	int temp = sqrt(pow((double)xNew,2.0)+pow((double)yNew,2.0)); 
	if(temp >= abs(rNew))
		drive = temp;
	else 
		drive = abs(rNew);
	if(drive > 127) drive = 127;
	if(drive < -127) drive = -127;
	drive += 128;

	new_ev.command = ROBOT_EVENT_MOTOR;
        new_ev.index = 0; new_ev.value = drive;
		send_event(&new_ev);

        new_ev.index = 1; new_ev.value = drive;
		send_event(&new_ev);

        new_ev.index = 6; new_ev.value = front;
		send_event(&new_ev);

        new_ev.index = 7; new_ev.value = rear;
		send_event(&new_ev);
	 }
}

void on_1hz_timer(robot_event *ev) {

}


void on_10hz_timer(robot_event *ev) {
	 robot_event ev1;
	 ev1.command = ROBOT_EVENT_CMD_NOOP;
	 ev1.index = 0;
	 ev1.value = 0;
	 send_event(&ev1);
}


void on_status_code(robot_event *ev) {
	switch(ev->command) {
		case ROBOT_EVENT_NET_STATUS_OK:
			log_string(-1, "Controller received STATUS_OK:%02X\n", ev->value);
			break;

		case ROBOT_EVENT_NET_STATUS_ERR:
			log_string(-1, "Controller received STATUS_ERR:%02X\n", ev->value);
			break;

		case ROBOT_EVENT_NET_STATUS_NOTICE:
			log_string(-1, "Controller received STATUS_NOTICE:%02X\n", ev->value);
			break;
		default:
			// unknown status datagram
			log_string(-1, "Controller reveived STATUS_%02X:%02X\n", ev->command, ev->value);
			break;
	}

}


void on_adc_change(robot_event *ev){
	log_string(-1, "ADC %02X value %02X", ev->index, ev->value);
}

void on_command_code(robot_event *ev) {
    robot_event send_ev;
	switch(ev->command) {
		case ROBOT_EVENT_CMD_NOOP:
            send_ev.command = ROBOT_EVENT_NET_STATUS_OK;
            send_ev.index = 0;
            send_ev.value = 0;

            send_event(&send_ev);
			break;

		case ROBOT_EVENT_CMD_START:
			log_string(-1, "Controller received CMD_START:%02X\n", ev->value);
			break;

		case ROBOT_EVENT_CMD_STOP:
			log_string(-1, "Controller received CMD_STOP:%02X\n", ev->value);
			break;

		case ROBOT_EVENT_CMD_REBOOT:
			
			log_string(-1, "Controller received CMD_REBOOT:%02X\n", ev->value);
			break;
		default:
			// unknown command code datagram
			log_string(-1, "Controller received CMD_%02X:%02X\n", ev->command, ev->value);
			break;
	}
}
on_read_variable(robot_event* ev){
}
on_set_variable(robot_event* ev){
}
