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
extern CSptr g_GL, g_LC, g_BS, g_FS, g_BL;

extern char g_state[];
extern double g_P[];
int g_do_quit;
extern double g_cnt2rad;
extern double g_2wradius;

void line_com(int cs, double *data){
  double x,y,theta;
  
  x = data[0];
  y = data[1];
  theta = data[2];
  switch(cs){
  case CS_GL:
    break;
  case CS_LC:
    CS_recursive_trans(g_GL, g_LC, &x, &y, &theta );
    break;
  case CS_FS:
    CS_recursive_trans(g_GL, g_FS, &x, &y, &theta );
    break;
  case CS_BS:
    CS_recursive_trans(g_GL, g_BS, &x, &y, &theta );
    break;
  case CS_BL:
    CS_recursive_trans(g_GL, g_BL, &x, &y, &theta );
    break;
  }
  g_spur_x     = x;
  g_spur_y     = y;
  g_spur_theta = theta;
  g_run_mode   = RUN_LINEFOLLOW;
}



void circle_com(int cs, double *data){
  double x,y,theta;
  
  x = data[0];
  y = data[1];
  theta = 0;
  switch(cs){
  case CS_GL:
    break;
  case CS_LC:
    CS_recursive_trans(g_GL, g_LC, &x, &y, &theta );
    break;
  case CS_FS:
    CS_recursive_trans(g_GL, g_FS, &x, &y, &theta );
    break;
  case CS_BS:
    CS_recursive_trans(g_GL, g_BS, &x, &y, &theta );
    break;
  case CS_BL:
    CS_recursive_trans(g_GL, g_BL, &x, &y, &theta );
    break;
  }

  g_spur_x     = x;
  g_spur_y     = y;
  g_spur_d     = data[2];
  g_run_mode   = RUN_CIRCLEFOLLOW;
}

void spin_com(int cs, double *data){
  double x,y,theta;
  
  x = 0;
  y = 0;
  theta = data[0];
  switch(cs){
  case CS_GL:
    break;
  case CS_LC:
    CS_recursive_trans(g_GL, g_LC, &x, &y, &theta );
    break;
  case CS_FS:
    CS_recursive_trans(g_GL, g_FS, &x, &y, &theta );
    break;
  case CS_BS:
    CS_recursive_trans(g_GL, g_BS, &x, &y, &theta );
    break;
  case CS_BL:
    CS_recursive_trans(g_GL, g_BL, &x, &y, &theta );
    break;
  }

  g_spur_theta = theta;
  g_run_mode   = RUN_SPIN;
}

void stop_com(double *data){
  g_run_mode = RUN_STOP;
}

void free_com(double *data){
  g_run_mode = RUN_FREE;
}

void vel_com(double *data){
  g_spur_v    = data[0];
  g_spur_w    = data[1];
  g_run_mode  = RUN_VEL;
}

void wheel_vel_com(double *data){
  g_spur_v    = data[0];
  g_spur_w    = data[1];
  g_run_mode  = RUN_WHEEL_VEL;
}
