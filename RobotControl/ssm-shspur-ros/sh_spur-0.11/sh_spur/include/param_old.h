#ifndef __PARAM__
#define __PARAM__
/*
//units
#define TORQUE_UNIT (100000.0) //[Nm]を基準に何倍(t=0.001[Nm]*--)。
#define VOLT_UNIT (1000.0) //[V]を基準に (v = 0.001[V]*--)
#define METER_UNIT (1000.0) //[m]基準に
#define ANPERE_UNIT (1000.0) //[A]
#define AVEL_UNIT (1.0) //[rad/s]
#define PWM_MAX 255.0

//device parameters
#define COUNT_REV (1000*4) //一回転何パルスか。
#define VOLT 24.0 //[V]電圧
#define CYCLE 0.001 //[s]制御周期
#define GEAR  (17.14) //
#define MOTOR_R  4.7 //[R]抵抗
#define MOTOR_TC  (0.049) //[Nm/A]トルク定数
#define MOTOR_VC  (1.0/0.00515) //[rpm/V]回転数定数 
//回転数定数が小さいとトルクリミット時爆走するおそれあり
//#define MOTOR_VTC (490*1000.0)  //[rpm/Nm]回転数トルク定数

#define RADIUS_R 0.058243066  //[m]
#define RADIUS_L 0.058268628
#define TREAD 0.29782971   //[m]

#define CONTROL_CYCLE 0.02

#define MAX_VEL 0.5
#define MAX_W  1.57
#define MAX_ACC_V (MAX_VEL*CONTROL_CYCLE/1.0)
#define MAX_ACC_W (MAX_W*CONTROL_CYCLE/1.0)

#define L_C1 0.01
#define L_K1 800.0
#define L_K2 300.0
#define L_K3 200.0
#define L_DIST 0.4
*/

enum{
 TORQUE_UNIT = 0,
   VOLT_UNIT,
   METER_UNIT,
   ANPERE_UNIT,
   AVEL_UNIT,
   PWM_MAX,
   COUNT_REV,
   VOLT,
   CYCLE,
   GEAR,
   MOTOR_R,
   MOTOR_TC,
   MOTOR_VC,
   MOTOR_VTC,
   RADIUS_R,
   RADIUS_L,
   TREAD,
   CONTROL_CYCLE,
   MAX_VEL,
   MAX_W,
   MAX_ACC_V,
   MAX_ACC_W,
   L_C1,
   L_K1,
   L_K2,
   L_K3,
   L_DIST,
   GAIN_KP,
   GAIN_KI,
   TORQUE_MAX,
   TORQUE_NEWTON,
   TORQUE_VISCOS,
   INTEGRAL_MAX,
   GAIN_A,
   GAIN_B,
   GAIN_C,
   GAIN_D,
   GAIN_E,
   GAIN_F,
   TORQUE_OFFSET,
   MASS
};

#define PARAM_NAME {"TORQUE_UNIT","VOLT_UNIT", "METER_UNIT", "ANPERE_UNIT","AVEL_UNIT","PWM_MAX","COUNT_REV","VOLT","CYCLE", "GEAR","MOTOR_R","MOTOR_TC","MOTOR_VC","MOTOR_VTC","RADIUS_R","RADIUS_L","TREAD","CONTROL_CYCLE","MAX_VEL", "MAX_W","MAX_ACC_V","MAX_ACC_W","L_C1","L_K1","L_K2","L_K3","L_DIST","GAIN_KP","GAIN_KI","TORQUE_MAX","TORQUE_NEWTON","TORQUE_VISCOS","INTEGRAL_MAX","GAIN_A","GAIN_B","GAIN_C","GAIN_D","GAIN_E","GAIN_F", "TORQUE_OFFSET","MASS"}

#define PARAM_NUM 41

#define OPTION_SHOW_ODOMETRY 1
#define OPTION_PARAM_CONTROL 2

#define DEFAULT_PARAMETER_FILE "robot.param"

#define GRAVITY 9.81
#define SIGN(x)	((x < 0) ? -1 : 1)

int arg_analyse(int argc, char* argv[]);
int set_param(char *filename);
void set_param_motor(void);
void set_param_velocity(void);
int parameter_set(char param, char id, int value);
void motor_speed(double r, double l);
void robot_speed(double v, double w);
int robot_speed_smooth(double v, double w);
int now_speed(double *v, double *w);
void motor_free(void);
void motor_servo(void);
#endif 
