#ifndef __PID_H
#define __PID_H

typedef struct 
{
	float Target_Val;
	float Output_Val;
	float Error;
	float LastError;
	float PrevError;
	float Integral;
	float Kp;
	float Ki;
	float Kd;
}PID;

void PID_Init(PID *pid);
float AddPID_Realize(PID *pid , float Actual_val , float Max_Pid_Value);
float PosPID_Realize(PID *pid , float Actual_val , float Max_Pid_Value);

#endif

