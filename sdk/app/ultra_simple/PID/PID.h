#ifndef _PID_H_
#define _PID_H_
#include <wiringPi.h>
#include <iostream>
#include <cmath>
#include <math.h>
const float INTEGRAL_CUTOFF = 10; //Angle in which the integral terms integrates. 


class PIDImpl;
class PID
{
    public:
        // Kp -  proportional gain
        // Ki -  Integral gain
        // Kd -  derivative gain
        // max - maximum value of manipulated variable
        // min - minimum value of manipulated variable
        PID( double max, double min, double Kp, double Kd, double Ki );


        // Returns the manipulated variable given a setpoint and current process value
        double calculate( double setpoint, double pv );
        ~PID();

    private:
        PIDImpl *pimpl;
};

#endif
