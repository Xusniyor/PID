/*
	19.06.2021
	https://github.com/Xusniyor
	Programmer:
	@Xusniyor
	@Uzbekistan
	@PID Regulator
*/
/*
The Circuit
                         Ci
                  |------| |--------------|
                  |           Rp          |
                  |----/\/\/\/\-----------|
                  |          Rd    Cd     |
           Rf     |----/\/\/\---| |-------|
Vin o----/\/\/\---|                       |
                  |    |\                 |
                  |    | \                |
                  |----|- \               | 
                       |   \              |
                       |    \-------------|---------o  Vout
                       |    /
                       |   /
                       |+ /
                   ----| /
                  |    |/
                  |
                  |
               ___|___ GND
                _____
                 ___
                  _
LEGEND:
  Vin is the input signal.
  Vout is the Output.
  Rp controls the propotional term ( P in PID) 
  Ci controls the Integral term ( I id PID)
  Rd and Cd controls the differential term ( D in PID)
  Rf is the gain control, which is common to all of the above controllers.
*/
#include "pid.h"

double PID(pidTypeDef *hpid, double setpoint, double newValue)
{
	// Calculate error
	double error = setpoint - newValue;

	// Proportional term
	double Pout = hpid->_Kp * error ;

	// Integral term
	hpid->_integral += error * hpid->_dt;
	double Iout = hpid->_Ki * hpid->_integral;

	// Derivative term
	double derivative = (error - hpid->_pre_error) / hpid->_dt;
	double Dout = hpid->_Kd * derivative;

	// Calculate total output
	double output = Pout + Iout + Dout; 

	// Restrict to max/min
	if( output > hpid->_max )
		output = hpid->_max;
	else if( output < hpid->_min )
		output = hpid->_min;

	// Save error to previous error
	hpid->_pre_error = error;

	return output;
}
