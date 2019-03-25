#include <wiringPi.h>
#include <softPwm.h>
#include <ncurses.h>
#include "Drive.h"
#include <math.h>

void Driver::init(){
	wiringPiSetup();

	pinMode(RW1, OUTPUT);
	pinMode(RW2, OUTPUT);
	pinMode(LW1, OUTPUT);
	pinMode(LW2, OUTPUT);
	softPwmCreate(RPWM, 0, 100);
	softPwmCreate(LPWM, 0, 100);
}

void Driver::stop(){
	digitalWrite(RW1, LOW);
	digitalWrite(RW2, LOW);
	digitalWrite(LW1, LOW);
	digitalWrite(LW2, LOW);

	softPwmWrite(RPWM, 0);
	softPwmWrite(LPWM, 0);
}

void Driver::setRWS(int RWS){
	rws = RWS;
	softPwmWrite(RPWM, rws);
}

void Driver::setLWS(int LWS){
	lws = LWS;
	softPwmWrite(LPWM, lws);
}


void Driver::forward(){
	init();

	digitalWrite(RW1, HIGH);
	digitalWrite(RW2, LOW);
	digitalWrite(LW1, HIGH);
	digitalWrite(LW2, LOW);

	softPwmWrite(RPWM, rws);
	softPwmWrite(LPWM, lws);
}

void Driver::reverse(){
	init();

	digitalWrite(RW1, LOW);
	digitalWrite(RW2, HIGH);
	digitalWrite(LW1, LOW);
	digitalWrite(LW2, HIGH);

	softPwmWrite(RPWM, rws);
	softPwmWrite(LPWM, lws);
}



void Driver::pivot_right(){
	init();

	digitalWrite(RW1, HIGH);
	digitalWrite(RW2, LOW);
	digitalWrite(LW1, LOW);
	digitalWrite(LW2, HIGH);

	softPwmWrite(RPWM, rws);
	softPwmWrite(LPWM, lws);
}

void Driver::pivot_left(){
	init();

	digitalWrite(RW1, LOW);
	digitalWrite(RW2, HIGH);
	digitalWrite(LW1, HIGH);
	digitalWrite(LW2, LOW);

	softPwmWrite(RPWM, rws);
	softPwmWrite(LPWM, lws);
}


void Driver::turn_correction(float error, float PID){ // PID has to be between 0 and 1.
	int rPivotSpeed, lPivotSpeed;
	if (abs(error) <= 0.1){
		rws = static_cast<int>(floor(62.5));
		lws = static_cast<int>(floor(62.5));
		forward();
	} else if (abs(error) < 12 && abs(error) > 0.1){
		if (error < 0){
			rws = static_cast<int>(floor(62.5 - 37.5*PID));
			lws = static_cast<int>(floor(62.5 + 37.5*PID));
			forward();
		} else if (error > 0){
			rws = static_cast<int>(floor(62.5 + 37.5*PID));
			lws = static_cast<int>(floor(62.5 - 37.5*PID));
			forward();
		  }
	  }
	  else {
		  if (error < 0){
			rws = static_cast<int>(floor(100 - 75.0f*PID));
			lws = static_cast<int>(floor(62.5 - 75.0f*PID));
			pivot_right();
		} else if (error > 0){
			rws = static_cast<int>(floor(62.5 - 75.0f*PID));
			lws = static_cast<int>(floor(62.5 - 75.0f*PID));
			pivot_left();
		  }
	  }
}


void Driver::manual(){
	initscr();
	cbreak();
	noecho();
	keypad(stdscr, TRUE);

	int speed = 50;
	int ch;
	int x = 0;

	do {
		ch = getch();
		mvaddch(0,x,ch);
		x++;
		switch(ch){
			case KEY_UP:
			case 'w':
				forward();
				break;
			case  KEY_DOWN:
			case 's':
				reverse();
				break;
			case KEY_RIGHT:
			case 'd':
				pivot_right();
				break;
			case KEY_LEFT:
			case 'a':
				pivot_left();
				break;
			case 'b':
				stop();
				break;
		}
	} while(ch != 'q');
	stop();
	endwin();
}


