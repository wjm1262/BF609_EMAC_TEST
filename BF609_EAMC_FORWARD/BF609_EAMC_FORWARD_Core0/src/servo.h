/*
 * servo.h
 *
 *  Created on: 2014-7-30
 *      Author: Administrator
 */

#ifndef SERVO_H_
#define SERVO_H_
typedef struct
{
	int maxOutput;
	int input;
	double output;
	double observedDrift;
	double kP;//比例因子
	double kI;//积分因子
	TimeInternal lastUpdate;
	bool runningMaxOutput;
	double dt;

} PIservo;

//PIservo PI={20000,0, 0,0,1.99915,2, 0,0,1 };
//PIservo PI={20000,0, 0,0,0.99915,0.03, 0,0,1 };//for EZ-KIT version
PIservo PI={20000,0, 0,0,0.99915,0.03, 0,0,1 };

/**
* \brief Struct used to average the offset from master
*
* The FIR filtering of the offset from master input is a simple, two-sample average
 */
typedef struct
{
	int  nsec_prev; //前一次
	int  y;          //当前输出
} offset_from_master_filter;

offset_from_master_filter  ofm_filt = {0,0};


double
runPIservo ( PIservo *servo, const int input )
{

	if ( servo->dt <= 0.0 )
		servo->dt = 1.0;

	servo->input = input;

	if ( servo->kP < 0.000001 )
		servo->kP = 0.000001;

	if ( servo->kI < 0.0000001 )
		servo->kI = 0;

	servo->observedDrift += servo->dt * ( ( input + 0.0 ) * servo->kI );

//	if ( servo->observedDrift >= servo->maxOutput )
//	{
//		servo->observedDrift = servo->maxOutput;
//		servo->runningMaxOutput = 1;
//	}
//
//	else if ( servo->observedDrift <= -servo->maxOutput )
//	{
//		servo->observedDrift = -servo->maxOutput;
//		servo->runningMaxOutput = 1;
//	}
//
//	else
//	{
//		servo->runningMaxOutput = 0;
//	}

	servo->output = ( servo->kP * ( input + 0.0 ) ) + servo->observedDrift;


	return -servo->output;
}

int
calcAdjAddend ( unsigned int addend, double adj, double dT )
{

	int  deltaAddend;
	double dAddend;

	dAddend= adj*addend/(dT*1000000000);

	deltaAddend  = (int) (dAddend+0.5);

	return deltaAddend;
}

#endif /* SERVO_H_ */
