
#ifndef _PID_SOURCE_
#define _PID_SOURCE_

#include <wiringPi.h>
#include <iostream>
#include <cmath>
#include <math.h>
#include "PID.h"

using namespace std;

class PIDImpl
{
    public:
        PIDImpl( double max, double min, double Kp, double Kd, double Ki );
        ~PIDImpl();
        double calculate( double setpoint, double pv );

    private:
        double prevTime;
        double _max;
        double _min;
        double _Kp;
        double _Kd;
        double _Ki;
        double _pre_error;
        double _integral;
};


PID::PID( double max, double min, double Kp, double Kd, double Ki )
{
    pimpl = new PIDImpl(max,min,Kp,Kd,Ki);
}
double PID::calculate( double setpoint, double pv )
{
    return pimpl->calculate(setpoint,pv);
}
PID::~PID() 
{
    delete pimpl;
}


/**
 * Implementation
 */
PIDImpl::PIDImpl( double max, double min, double Kp, double Kd, double Ki ) :
    prevTime(static_cast<double>(millis()) ),
    _max(max),
    _min(min),
    _Kp(Kp),
    _Kd(Kd),
    _Ki(Ki),
    _pre_error(0),
    _integral(0)
{}

double PIDImpl::calculate( double setpoint, double pv )
{
    double nowTime = static_cast<double>(millis()); // / 1000 if seconds are desired.
    // Calculate error
    double error = setpoint - pv;

    // Proportional term
    double Pout = _Kp * error;

    // Integral term
    if (abs(error) > INTEGRAL_CUTOFF){
        _integral = 0;
    }
    else if (abs(error) <= INTEGRAL_CUTOFF){
        _integral += error * (nowTime - prevTime);
    }
    double Iout = _Ki * _integral;

    // Derivative term
    double derivative = (error - _pre_error) / (nowTime - prevTime);
    double Dout = _Kd * derivative;

    // Calculate total output
    double output = Pout + Iout + Dout;

    // Restrict to max/min
    if( output > _max )
        output = _max;
    else if( output < _min )
        output = _min;

    // Save error to previous error
    _pre_error = error;
    prevTime = static_cast<double>(millis());
    return output;
}

PIDImpl::~PIDImpl()
{
}

#endif
