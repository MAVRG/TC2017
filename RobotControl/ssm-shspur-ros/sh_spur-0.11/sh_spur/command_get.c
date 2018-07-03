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
extern CSptr g_GL, g_LC, g_BS, g_FS, g_BL;
extern double g_spur_x;
extern double g_spur_y;
extern double g_spur_theta;
extern double g_spur_v;
extern double g_spur_w;
extern double g_spur_d;
extern int g_run_mode;
extern Odometry g_odometry;
extern char g_state[];
extern double g_P[];
int g_do_quit;
extern double g_cnt2rad;
extern double g_2wradius;

void get_pos_com(int cs, double *data, double *resdata){
  double x,y,theta;
  x = g_odometry.x;
  y = g_odometry.y; 
  theta = g_odometry.theta;

  switch(cs){
    case CS_GL:
      CS_recursive_trans(g_GL,g_BS,&x,&y,&theta);
      break;
    case CS_LC:
      CS_recursive_trans(g_LC,g_BS,&x,&y,&theta);
      break;
    case CS_BS:
      //    CS_recursive_trans(g_GL,g_BS,&x,&y,&theta);
      break;
    case CS_BL:
      CS_recursive_trans(g_BL,g_BS,&x,&y,&theta);
      break;
  }
  resdata[0] = x;
  resdata[1] = y;
  resdata[2] = theta;
  resdata[3] = g_odometry.time;

  printf("get  %f %f %f\n",x,y,theta);
}

void get_vel_com(int cs, double *data, double *resdata){

  resdata[0] = g_odometry.v;
  resdata[1] = g_odometry.w;
  resdata[2] = g_odometry.time;

  //  printf("getvel  %f %f %f\n",);
}

int near_pos_com(int cs, double *data, double *resdata){
  double x,y,theta,cx,cy;
  double dist;

  x = g_odometry.x;
  y = g_odometry.y; 
  theta = g_odometry.theta;

  switch(cs){
  case CS_GL:
    CS_recursive_trans(g_GL,g_BS,&x,&y,&theta);
    break;
  case CS_LC:
    CS_recursive_trans(g_LC,g_BS,&x,&y,&theta);
    break;
  case CS_BS:
    //    CS_recursive_trans(g_GL,g_BS,&x,&y,&theta);
    break;
  case CS_BL:
    CS_recursive_trans(g_BL,g_BS,&x,&y,&theta);
    break;
  }
  cx = data[0];
  cy = data[1];
  dist = sqrt((cx-x)*(cx-x)+(cy-y)*(cy-y));
  resdata[0] = dist;

  if(dist < data[2])return 1;
  else return 0;
}

int near_ang_com(int cs, double *data, double *resdata){
  double x,y,theta;
  double dist;

  x = g_odometry.x;
  y = g_odometry.y; 
  theta = g_odometry.theta;

  switch(cs){
    case CS_GL:
      CS_recursive_trans(g_GL,g_BS,&x,&y,&theta);
      break;
    case CS_LC:
      CS_recursive_trans(g_LC,g_BS,&x,&y,&theta);
      break;
    case CS_BS:
      //    CS_recursive_trans(g_GL,g_BS,&x,&y,&theta);
      break;
    case CS_BL:
      CS_recursive_trans(g_BL,g_BS,&x,&y,&theta);
      break;

  }
  dist = theta-data[0];
  while(dist > M_PI)dist-=2*M_PI;
  while(dist <-M_PI)dist+=2*M_PI;
  resdata[0] = dist;
  if(fabs(dist) < data[1])return 1; 
  else return 0;
}

int over_line_com(int cs, double *data, double *resdata){
  double x,y,theta;
  double dist;

  x = g_odometry.x;
  y = g_odometry.y; 
  theta = g_odometry.theta;

  switch(cs){
    case CS_GL:
      CS_recursive_trans(g_GL,g_BS,&x,&y,&theta);
      break;
    case CS_LC:
      CS_recursive_trans(g_LC,g_BS,&x,&y,&theta);
      break;
    case CS_BS:
      //    CS_recursive_trans(g_GL,g_BS,&x,&y,&theta);
      break;
    case CS_BL:
      CS_recursive_trans(g_BL,g_BS,&x,&y,&theta);
      break;
  }

  dist = (x-data[0])*cos(data[2]) + (y-data[1])*sin(data[2]);

  resdata[0] = dist;
  if(dist > 0)return 1; 
  else return 0;
}
