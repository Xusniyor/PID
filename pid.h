/*
	19.06.2021
	https://github.com/Xusniyor
	Programmer:
	@Xusniyor
	@Uzbekistan
	@PID Regulator
*/
#ifndef __PID_SOURCE_H_
#define __PID_SOURCE_H_
/*
	output = Kp * err + (Ki * int * dt) + (Kd * der /dt);

	Kp  = Proptional Constant.
	Ki  = Integral Constant.
	Kd  = Derivative Constant.
	err = Expected Output - Actual Output ie. error;
	int = int from previous loop + err; ( i.e. integral error )
	der = err - err from previous loop; ( i.e. differential error)
	dt  = execution time of loop.
*/
typedef struct
{
	double _dt;
	double _max; // max out.
	double _min; // min out.
	double _Kp;  // Proptional Constant.
	double _Kd;  // Derivative Constant.
	double _Ki;  // Integral Constant.
	double _pre_error;
	double _integral;
} pidTypeDef;

double PID(pidTypeDef *hpid, double setpoint, double newValue);

#endif /*__PID_SOURCE_H_*/
