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
/*傾き*/
extern double g_spur_tilt;
extern double g_spur_dir;

extern int g_run_mode;
extern Odometry g_odometry;
extern char g_state[];
extern double g_P[];
int g_do_quit;
extern double g_cnt2rad;
extern double g_2wradius;

void set_pos_com(int cs, double *data){
  double x,y,theta;
  x = data[0];
  y = data[1];
  theta = data[2];

  CS_turn_base(&x, &y, &theta);

  if(cs == CS_GL){
    CS_recursive_trans(g_BS,g_FS, &x, &y, &theta);
    CS_set(g_GL, x,y,theta);
  }else if(cs == CS_LC){
    CS_recursive_trans(g_GL,g_FS, &x, &y, &theta);
    CS_set(g_LC, x,y,theta);
  }else if(cs == CS_BL){
    CS_recursive_trans(g_BS,g_FS, &x, &y, &theta);
    CS_set(g_BL, x,y,theta);
  }
}

void set_GL_on_GL_com(double *data){
  double x,y,theta;
  x = data[0];
  y = data[1];
  theta = data[2];

  CS_recursive_trans(g_BS, g_GL, &x, &y, &theta);
  CS_set(g_GL, x, y, theta);
}

void set_adjust_com(int cs, double *data){
  double x,y,theta;

  x = data[0];
  y = data[1];
  theta = data[2];
  switch(cs){
  case CS_GL:
    break;
  case CS_LC:
    CS_recursive_trans(g_GL, g_LC, &x, &y, &theta);
    break;
  case CS_FS:
    CS_recursive_trans(g_GL, g_FS, &x, &y, &theta);
    break;
  case CS_BS:
    CS_recursive_trans(g_GL, g_BS, &x, &y, &theta);
    break;
  case CS_BL:
    CS_recursive_trans(g_GL, g_BL, &x, &y, &theta);
    break;
  }

  /*ロボット(FS)がGL上のx,y,thetaに見えるとするとき*/
  /*FSからGLがどこに見えるか(GL->FS => FS->GL)*/
  CS_turn_base(&x,&y,&theta);
  /*それはBS上のどこか*/
  CS_recursive_trans(g_BS,g_FS,&x,&y,&theta);
  /*GLをセット*/
  CS_set(g_GL, x, y, theta);
   


}

void set_vel_com(double *data){
  g_spur_v     = data[0];
}

void set_ang_vel_com(double *data){
  g_spur_w     = data[0];
}


void set_tilt_com(int cs, double *data){
  double x,y,theta;

  x = 0; y = 0; theta = data[0];

  switch(cs){
  case CS_GL:
    break;
  case CS_LC:
    CS_recursive_trans(g_GL, g_LC, &x, &y, &theta);
    break;
  case CS_FS:
    CS_recursive_trans(g_GL, g_FS, &x, &y, &theta);
    break;
  case CS_BS:
    CS_recursive_trans(g_GL, g_BS, &x, &y, &theta);
    break;
  case CS_BL:
    CS_recursive_trans(g_GL, g_BL, &x, &y, &theta);
    break;
  }

  g_spur_dir  = theta;
  g_spur_tilt = data[1];
}
