#pragma once

#include <stdio.h>
#include <stdlib.h>


#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifdef _WIN32
#include <Windows.h>
#define delay(x)   ::Sleep(x)
#else
#include <unistd.h>

static inline void delay(_word_size_t ms){
    while (ms>=1000){
        usleep(1000*1000);
        ms-=1000;
    };
    if (ms!=0)
        usleep(ms*1000);
}

//WARNING! THE ABOVE FUNCTION CAN CRASH WITH WIRINGPI.h with DELAY BEWARE. 
#endif

rp::standalone::rplidar::RPlidarDriver* startLidar();
rp::standalone::rplidar::RPlidarDriver* startLidarExpress();
int lidarData(float dist[], float angl[], int quality[], rp::standalone::rplidar::RPlidarDriver * drv );
void stopLidar(rp::standalone::rplidar::RPlidarDriver * drv );
