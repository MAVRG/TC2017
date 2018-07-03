#ifndef __PARAM__
#define __PARAM__

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

#define PARAM_NAME {"TORQUE_UNIT","VOLT_UNIT", "METER_UNIT", "ANPERE_UNIT","AVEL_UNIT","PWM_MAX","COUNT_REV","VOLT","CYCLE", "GEAR","MOTOR_R","MOTOR_TC","MOTOR_VC","MOTOR_VTC","RADIUS_R","RADIUS_L","TREAD","CONTROL_CYCLE","MAX_VEL", "MAX_W","MAX_ACC_V","MAX_ACC_W","L_C1","L_K1","L_K2","L_K3","L_DIST","GAIN_KP","GAIN_KI","TORQUE_MAX","TORQUE_NEWTON","TORQUE_VISCOS","INTEGRAL_MAX","GAIN_A","GAIN_B","GAIN_C","GAIN_D","GAIN_E","GAIN_F","TORQUE_OFFSET","MASS"}

#define PARAM_COMMENT {"[Integer Nm/Nm]","[Integer V/V]", "[Integer m/m]", "[Integer A/A]","[Integer rad/s / rad/s]","(28000000/DUTY)","[Counts/rev]","[V]","[s]", "[in/out]","[ohm]","[Nm/A]","[rpm/V]","","[m]","[m]","[m]","[s]","[m/s]", "[rad/s]","[m/ss]","[rad/ss]","","","","","","PI control parameter Kp","PI control parameter Ki","[Nm]","[Nm]","[Nm/(count/1ms)]","","PWS parameter A","PWS parameter B","PWS parameter C","PWS parameter D","PWS parameter E","PWS parameter F","[Nm]","[kg]"}

#define PARAM_NUM 41

#define OPTION_SHOW_ODOMETRY 1
#define OPTION_PARAM_CONTROL 2
#define DEFAULT_PARAMETER_FILE "./robot.param"
//#define DEFAULT_PARAMETER_FILE "/usr/local/share/sh_spur/robot.param"
#define DEFAULT_DEVICE_NAME "/dev/ttyUSB0"

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
void motor_stop(void);
void motor_free(void);
void motor_servo(void);
#endif 
