#include "main.h"

typedef struct{
  uint8_t flag;
  float Target[Motor];
  float Real_Target[Motor];
  float Target_Yaw;
  float Kp_turn;
  //float Kd_turn = 0.005f;
  float Angle_Error;
  //float Last_Angle_Error;
  float Turn_Out;
}PID;

void PID_Init(PID* pid);
void PID_Start(PID *pid);
void PID_Stop(PID *pid);
void PID_Run(PID *pid, float yaw);