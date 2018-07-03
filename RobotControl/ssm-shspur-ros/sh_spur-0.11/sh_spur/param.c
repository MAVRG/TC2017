/*high level I/O*/
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <strings.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>

/*low level I/O*/
#include <fcntl.h>
#include <sys/time.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>

/*serial*/
#include <sys/termios.h>

/*ボディパラメータ*/
#include <sh_vel_parameters.h>

/*SSM 用*/
//#include <ssm_common.h>
//#include <Spur_odometry.h>

/*sh_spur用*/
#include <serial.h>
#include <param.h>
#include <control.h>
#include <command.h>

/*ライブラリ用*/
#include <SHSpur.h>



extern double before_v, before_w;/*直前の速度*/
extern char g_parameter_filename[];
extern char g_device_name[];
extern double g_P[];
extern Odometry g_odometry;
extern int g_option;
extern char g_state[];
extern int g_run_mode;

extern double g_cnt2rad;
extern double g_2wradius;

int is_character(int c);
int is_number(int c);

/**/
int arg_analyse(int argc, char* argv[]){
  int i;

  g_option = 0;  

  strcpy(g_parameter_filename, DEFAULT_PARAMETER_FILE);
  strcpy(g_device_name, DEFAULT_DEVICE_NAME);

  for(i = 1; i < argc; i++){
    if(!strcmp(argv[i], "--show_odometry") || !strcmp(argv[i],"-o")){
      g_option |= OPTION_SHOW_ODOMETRY;
    }else if(!strcmp(argv[i], "--param") || !strcmp(argv[i], "-p")){
      i++;
      if(i < argc){
	strcpy(g_parameter_filename, argv[i]);
      }
    }else if(!strcmp(argv[i], "--device") || !strcmp(argv[i], "-d")){
      i++;
      if(i < argc){
	strcpy(g_device_name, argv[i]);
      }
    }else if(!strcmp(argv[i], "--control") || !strcmp(argv[i], "-c")){
      g_option |= OPTION_PARAM_CONTROL;
      g_state[0] = DISABLE;
      g_state[1] = DISABLE;
      g_state[2] = DISABLE;
      g_state[3] = DISABLE;
    }
  }
  return 0;
}


int is_character(int c){
  if(c >= 'A' && c <='Z')return 1;
  if(c >= 'a' && c <='z')return 1;
  if(c == '_')return 1;
  return 0;
}


int is_number(int c){
  if(c >= '0' && c <='9')return 1;
  if(c == '-')return 1;
  if(c == '.')return 1;
  return 0;
}

/**/
int set_param(char *filename){
  FILE* paramfile;
  char  param_names[PARAM_NUM][20] = PARAM_NAME;
  char name[20],value_str[100];
  int i,c;
  //  double value;
  int str_wp;
  int read_state;

  /**/
  paramfile = fopen(g_parameter_filename, "r");
  if(!paramfile){
    printf("File [%s] is not exist.\n",g_parameter_filename);
    return 0;
  }
  for(i = 0; i < PARAM_NUM; i++){
    g_P[i] = 0;
  }

  read_state = 0;
  str_wp = 0;
  while((c = getc(paramfile)) != EOF){
    switch(read_state){
    case 0:/**/
      if(c == '#'){
	read_state = 1;
      }
      if(is_character(c)){
	name[0] = c;
	read_state = 2;
	str_wp = 1;
      }
      break;
    case 1:/*comment*/
      if(c == '\n'){
	read_state = 0;
      }
      break;
    case 2:/*name*/
      name[str_wp] = c;
      if(!(is_character(c)||is_number(c)) ){
	name[str_wp] = 0;
	read_state = 3;
	str_wp = 0;
      }else{
	str_wp++;
      }
      break;
    case 3:/*value */
      if(is_number(c)){
	str_wp = 0;
	value_str[str_wp] = c;
	str_wp++;
	read_state = 4;
      }
      break;
    case 4:/*value */
      value_str[str_wp] = c;
      if(!is_number(c)){
	value_str[str_wp] = 0;
	read_state = 0;
	//	printf("%s\n",name);
	for(i = 0; i < PARAM_NUM; i++){
	  if(!strcmp(name, param_names[i])){
	    g_P[i] = strtod(value_str,0);
	    printf("%s %f\n",name,g_P[i]);
	    break;
	  }
	} 
      }else{
	str_wp++;
      }
    }
  }

  /*パラメータの入力*/
  //  parameter_set(PARAM_free,0,0);   

  /*モータのパラメータ*/ 
  set_param_motor();
  usleep(100000);

  /*速度制御パラメータ*/
  set_param_velocity();
  usleep(100000);

  g_2wradius = g_P[RADIUS_R]+g_P[RADIUS_L];
  robot_speed(0,0);

  g_odometry.x = 0;
  g_odometry.y = 0;
  g_odometry.theta = 0;
  g_run_mode = RUN_STOP;
  /*パラメータを有効にする*/
  g_state[SHSPUR_PARAM_MOTOR]    = ENABLE;
  g_state[SHSPUR_PARAM_VELOCITY] = ENABLE;
  g_state[SHSPUR_PARAM_BODY]     = ENABLE;
  g_state[SHSPUR_PARAM_TRACKING] = ENABLE;

  printf("mass=%f\n",g_P[MASS]);
  return 0;
}

void set_param_motor(void){
  /*モータのパラメータ*/ 
  parameter_set(PARAM_p_ki,0,
		(double)(65536.0*g_P[PWM_MAX]*g_P[MOTOR_R]/
			 (g_P[TORQUE_UNIT]*g_P[MOTOR_TC]*g_P[VOLT])));
  parameter_set(PARAM_p_ki,1,
		(double)(65536.0*g_P[PWM_MAX]*g_P[MOTOR_R]/
			 (g_P[TORQUE_UNIT]*g_P[MOTOR_TC]*g_P[VOLT])));
  parameter_set(PARAM_p_kv,0,
		(double)(65536.0*g_P[PWM_MAX]*60.0/
			 (g_P[MOTOR_VC]*g_P[VOLT]*g_P[COUNT_REV]*g_P[CYCLE])));
  parameter_set(PARAM_p_kv,1,
		(double)(65536.0*g_P[PWM_MAX]*60.0/
			 (g_P[MOTOR_VC]*g_P[VOLT]*g_P[COUNT_REV]*g_P[CYCLE])));    
  /*摩擦補償*/
  parameter_set(PARAM_p_fr_plus, 0,  g_P[TORQUE_NEWTON]*g_P[TORQUE_UNIT]);
  parameter_set(PARAM_p_fr_plus, 1,  g_P[TORQUE_NEWTON]*g_P[TORQUE_UNIT]);
  parameter_set(PARAM_p_fr_wplus, 0, g_P[TORQUE_VISCOS]*g_P[TORQUE_UNIT]);
  parameter_set(PARAM_p_fr_wplus, 1, g_P[TORQUE_VISCOS]*g_P[TORQUE_UNIT]);
  parameter_set(PARAM_p_fr_minus, 0, g_P[TORQUE_NEWTON]*g_P[TORQUE_UNIT]);
  parameter_set(PARAM_p_fr_minus, 1, g_P[TORQUE_NEWTON]*g_P[TORQUE_UNIT]);
  parameter_set(PARAM_p_fr_wminus, 0,g_P[TORQUE_VISCOS]*g_P[TORQUE_UNIT]);
  parameter_set(PARAM_p_fr_wminus, 1,g_P[TORQUE_VISCOS]*g_P[TORQUE_UNIT]); 
  parameter_set(PARAM_p_toq_offset,0,g_P[TORQUE_OFFSET]*g_P[TORQUE_UNIT]); 
  parameter_set(PARAM_p_toq_offset,1,g_P[TORQUE_OFFSET]*g_P[TORQUE_UNIT]); 

  parameter_set(PARAM_pwm_max,0, g_P[PWM_MAX]);  
  parameter_set(PARAM_pwm_max,1, g_P[PWM_MAX]);  
  parameter_set(PARAM_pwm_min,0,-g_P[PWM_MAX]);  
  parameter_set(PARAM_pwm_min,1,-g_P[PWM_MAX]);  

  parameter_set(PARAM_toq_max,0, g_P[TORQUE_MAX]*g_P[TORQUE_UNIT]);  
  parameter_set(PARAM_toq_max,1, g_P[TORQUE_MAX]*g_P[TORQUE_UNIT]);
  parameter_set(PARAM_toq_min,0,-g_P[TORQUE_MAX]*g_P[TORQUE_UNIT]);    
  parameter_set(PARAM_toq_min,1,-g_P[TORQUE_MAX]*g_P[TORQUE_UNIT]);

  g_cnt2rad  = g_P[GEAR]*g_P[COUNT_REV]*g_P[CYCLE]/(2*M_PI);
}

void set_param_velocity(void){
  /*ウォッチドックタイマの設定*/
  parameter_set(PARAM_watch_dog_limit,0,300);     

  /*FF制御パラメータ*/
  parameter_set(PARAM_p_A, 0, g_P[GAIN_A]);
  parameter_set(PARAM_p_B, 0, g_P[GAIN_B]);
  parameter_set(PARAM_p_C, 0, g_P[GAIN_C]);
  parameter_set(PARAM_p_D, 0, g_P[GAIN_D]);
  parameter_set(PARAM_p_E, 0, g_P[GAIN_E]);
  parameter_set(PARAM_p_F, 0, g_P[GAIN_F]);
  /*PI制御のパラメータ*/
  parameter_set(PARAM_p_pi_kp, 0, g_P[GAIN_KP]);  
  parameter_set(PARAM_p_pi_kp, 1, g_P[GAIN_KP]);
  parameter_set(PARAM_p_pi_ki, 0, g_P[GAIN_KI]);  
  parameter_set(PARAM_p_pi_ki, 1, g_P[GAIN_KI]);
  /*各種制限*/
  parameter_set(PARAM_int_max,0, g_P[INTEGRAL_MAX]); 
  parameter_set(PARAM_int_max,1, g_P[INTEGRAL_MAX]);     
  parameter_set(PARAM_int_min,0,-g_P[INTEGRAL_MAX]);     
  parameter_set(PARAM_int_min,1,-g_P[INTEGRAL_MAX]); 
  


  parameter_set(PARAM_w_ref,0,0);     
  parameter_set(PARAM_w_ref,1,0);
  parameter_set(PARAM_servo,0,SERVO_LEVEL_VELOCITY);   
}


void motor_stop(void){
  parameter_set(PARAM_servo, 0, SERVO_LEVEL_COUNTER);   
}

void motor_free(void){
  parameter_set(PARAM_servo, 0, SERVO_LEVEL_TORQUE);   
}

void motor_servo(void){
  parameter_set(PARAM_servo, 0, SERVO_LEVEL_VELOCITY);   
}

/*[rad/s]*/
void motor_speed(double r, double l){
  int ir,il;

  ir =(double)( r * g_cnt2rad);  
  il =(double)( l * g_cnt2rad);  

  parameter_set(PARAM_w_ref,0,ir);     
  parameter_set(PARAM_w_ref,1,il);   

  //printf("%f %f\n",r,l);
}

/*m/s rad/s*/
void robot_speed(double v, double w){
  double wr, wl;

  before_v = v;
  before_w = w;

  wr=2.0*v/(g_2wradius)-g_P[TREAD]*w/(g_2wradius);
  wl=2.0*v/(g_2wradius)+g_P[TREAD]*w/(g_2wradius);
  
  motor_speed(wr,wl);
}

/**/
int robot_speed_smooth(double v, double w){
  int limit;
  
  limit = 15;

  if(v > g_P[MAX_VEL]){
    v = g_P[MAX_VEL];
  }else if(v < -g_P[MAX_VEL]){
    v = -g_P[MAX_VEL];
  }else{
    limit -=1;
  }

  if(v > before_v + g_P[MAX_ACC_V]){
    v= before_v+g_P[MAX_ACC_V];
  }else if(v < before_v - g_P[MAX_ACC_V]){
    v= before_v-g_P[MAX_ACC_V];
  }else{
    limit-= 2;
  }


  if(w > g_P[MAX_W]){
    w = g_P[MAX_W];
  }else if(w < -g_P[MAX_W]){
    w = -g_P[MAX_W];
  }else{
    limit -=4;
  }

  if(w > before_w + g_P[MAX_ACC_W]){
    w= before_w+g_P[MAX_ACC_W];
  }else if(w < before_w - g_P[MAX_ACC_W]){
    w= before_w-g_P[MAX_ACC_W];
  }else{
    limit -= 8;
  }


  robot_speed(v,w);
  return limit;
}

int now_speed(double *v, double *w){
  *v = before_v;
  *w = before_w;
  return 0;
}

