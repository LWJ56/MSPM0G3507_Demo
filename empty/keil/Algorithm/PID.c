#include "PID.h"



void PID_Init(PID *pid)
{
	
	pid->Target_Val=0;
	pid->Output_Val=0;
	pid->Error=0;
	pid->LastError=0;
	pid->PrevError=0;
	pid->Integral=0;
	pid->Kp=0.0;
	pid->Ki=0.0;
  pid->Kd=0.0;
}

float AddPID_Realize(PID *pid , float Actual_val , float Max_Pid_Value)
{
	pid->Error = pid->Target_Val - Actual_val;//�������
	if(pid->Error > 4096)
	{
		pid->Error = pid->Target_Val - (Actual_val + 8192);
	}
	else if(pid->Error < (-4096))
	{
		pid->Error = pid->Target_Val - Actual_val + 8192;
	}
	else
	{
		;
	}
	pid->Output_Val += pid->Kp * (pid->Error - pid->LastError)+
	                   pid->Ki * pid->Error +
	                   pid->Kd * (pid->Error- 2*pid->LastError + pid->PrevError);//��ʽ����
	
	//�������
  pid->PrevError = pid->LastError;
	pid->LastError = pid->Error;
	
	//�����޷�
	if(pid->Output_Val > Max_Pid_Value)
	{
		pid->Output_Val = Max_Pid_Value;
		return Max_Pid_Value;
	}
	else if(pid->Output_Val <(-1)*Max_Pid_Value)
	{
		pid->Output_Val = (-1)*Max_Pid_Value;
		return (-1)*Max_Pid_Value;
	}
	else
		return pid->Output_Val;
}

float PosPID_Realize(PID *pid , float Actual_val , float Max_Pid_Value)
{
	pid->Error = pid->Target_Val - Actual_val;//�������
	
	pid->Output_Val  = pid->Kp * pid->Error + pid->Kd * (pid->Error- pid->LastError);//��ʽ����
	pid->Output_Val += pid->Ki * pid->Error;//��ʽ����
	
	//�������
  pid->PrevError = pid->LastError;
	pid->LastError = pid->Error;
	
	//�����޷�
	if(pid->Output_Val > Max_Pid_Value)
	{
		pid->Output_Val = Max_Pid_Value;
		return Max_Pid_Value;
	}
	else if(pid->Output_Val <(-1)*Max_Pid_Value)
	{
		pid->Output_Val = (-1)*Max_Pid_Value;
		return (-1)*Max_Pid_Value;
	}
	else
		return pid->Output_Val;
}
