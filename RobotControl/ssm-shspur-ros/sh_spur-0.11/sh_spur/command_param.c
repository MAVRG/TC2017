/*high level I/O*/
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <strings.h>
#include <pthread.h>

/*low level I/O*/
#include <fcntl.h>
#include <sys/time.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ipc.h>
#include <sys/msg.h>

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
#include <CoordinateSystem2D.h>


/*pthread mutex用*/
extern pthread_mutex_t mutex;

extern double g_spur_x;
extern double g_spur_y;
extern double g_spur_theta;
extern double g_spur_v;
extern double g_spur_w;
extern double g_spur_d;
extern int g_run_mode;
extern Odometry g_odometry;
extern CSptr g_GL, g_LC, g_BS, g_FS;

extern char g_state[];
extern double g_P[];
int g_do_quit;
extern double g_cnt2rad;
extern double g_2wradius;


void param_set_com(int cs, double *data){
  if(cs >= 0 && cs < PARAM_NUM)
    g_P[cs] = data[0];
}


void param_state_com(int cs, double *data){
  if(cs >= 0 && cs < 5)
    g_state[cs] = data[0];
  
  if(cs == SHSPUR_PARAM_MOTOR && data[0] == ENABLE){
    set_param_motor();
    printf("motor enable\n");
  }
  if(cs == SHSPUR_PARAM_VELOCITY && data[0] == ENABLE){
    set_param_velocity();
    printf("vel enable\n");
  }
  if(cs == SHSPUR_PARAM_BODY && data[0] == ENABLE){
    g_2wradius = g_P[RADIUS_R]+g_P[RADIUS_L];
    robot_speed(0,0);
    g_odometry.x = 0;
    g_odometry.y = 0;
    g_odometry.theta = 0;
    printf("body enable\n");
  }
  if(cs == SHSPUR_PARAM_TRACKING && data[0] == ENABLE){
    printf("tracking enable\n");
  }
  if(cs == SHSPUR_PARAM_GRAVITY && data[0] == ENABLE){
   
    printf("gravity compensation enable\n");
  }

}
