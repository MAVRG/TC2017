#ifndef __PARAM__
#define __PARAM__

//units
#define TORQUE_UNIT (100000.0) //[Nm]を基準に何倍(t=0.001[Nm]*--)。
#define VOLT_UNIT (1000.0) //[V]を基準に (v = 0.001[V]*--)
#define METER_UNIT (1000.0) //[m]基準に
#define ANPERE_UNIT (1000.0) //[A]
#define AVEL_UNIT (1.0) //[rad/s]
#define PWM_MAX 255.0

//device parameters
#define COUNT_REV (500*4) //一回転何パルスか。
#define VOLT 24.0 //[V]電圧
#define CYCLE 0.001 //[s]制御周期
#define GEAR  (17.14) //
#define MOTOR_R  4.7 //[R]抵抗
#define MOTOR_TC  (0.049) //[Nm/A]トルク定数
#define MOTOR_VC  (1.0/0.00515) //[rpm/V]回転数定数 
//#define MOTOR_VTC (490*1000.0)  //[rpm/Nm]回転数トルク定数


void set_param(void);
int parameter_set(char param, char id, int value);
void motor_speed(double r, double l);

#endif 
