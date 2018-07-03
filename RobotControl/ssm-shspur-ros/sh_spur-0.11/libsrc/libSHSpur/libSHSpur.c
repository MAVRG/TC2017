/*
  SH Spurとの通信ライブラリ
 Communication Library for SH Spur
*/
#include<unistd.h>
#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>

#include <SHSpur.h>

static int pid;
static int msq_id;


/*poetとのメッセージ通信を開始する*/
int SHSpur_init(void){

  /*メッセージ・キューのオープン*/
  msq_id = msgget(SHSPUR_MSQ_KEY, 0666);

  /*内部データの初期化*/
  pid = 0x07fff & getpid();

  return 1;
}

/**/
int SHSpur_line(int cs, double x, double y, double theta)
{
  SHSpur_msg msg;

  msg.msg_type = SHSPUR_MSG_CMD;
  msg.pid  = pid;
  msg.type = SHSPUR_LINE;
  msg.data[0] = x;
  msg.data[1] = y;
  msg.data[2] = theta;
  msg.cs = cs;

  if(msgsnd(msq_id,&msg,SHSPUR_MSG_SIZE , 0) < 0){
    /*error*/
  }   
  return 1;
}
/**/
int SHSpur_circle(int cs, double x, double y, double r)
{
  SHSpur_msg msg;

  msg.msg_type = SHSPUR_MSG_CMD;
  msg.pid  = pid;
  msg.type = SHSPUR_CIRCLE;
  msg.data[0] = x;
  msg.data[1] = y;
  msg.data[2] = r;
  msg.cs = cs;
  if(msgsnd(msq_id,&msg,SHSPUR_MSG_SIZE , 0) < 0){
    /*error*/
  }
   
  return 1;
}

/**/
int SHSpur_spin(int cs, double theta)
{
  SHSpur_msg msg;

  msg.msg_type = SHSPUR_MSG_CMD;
  msg.pid  = pid;
  msg.type = SHSPUR_SPIN;
  msg.data[0] = theta;
  msg.cs = cs;
  if(msgsnd(msq_id,&msg,SHSPUR_MSG_SIZE , 0) < 0){
    /*error*/
  }
   
  return 1;
}

/**/
int SHSpur_stop(void)
{
  SHSpur_msg msg;

  msg.msg_type = SHSPUR_MSG_CMD;
  msg.pid  = pid;
  msg.type = SHSPUR_STOP;
 
  if(msgsnd(msq_id,&msg,SHSPUR_MSG_SIZE , 0) < 0){
    /*error*/
  }
   
  return 1;
}
int SHSpur_free(void)
{
  SHSpur_msg msg;

  msg.msg_type = SHSPUR_MSG_CMD;
  msg.pid  = pid;
  msg.type = SHSPUR_FREE;
 
  if(msgsnd(msq_id,&msg,SHSPUR_MSG_SIZE , 0) < 0){
    /*error*/
  }
   
  return 1;
}

int SHSpur_set_pos(int cs, double x, double y, double theta){
  SHSpur_msg msg;
  
  msg.msg_type = SHSPUR_MSG_CMD;
  msg.pid  = pid;
  msg.type = SHSPUR_SET_POS;
  msg.data[0] = x;
  msg.data[1] = y;
  msg.data[2] = theta;
  msg.cs = cs;

  if(msgsnd(msq_id,&msg,SHSPUR_MSG_SIZE , 0) < 0){
    /*error*/
  }   
  return 1;
}

int SHSpur_adjust_pos(int cs, double x, double y, double theta){
  SHSpur_msg msg;
  
  msg.msg_type = SHSPUR_MSG_CMD;
  msg.pid  = pid;
  msg.type = SHSPUR_ADJUST;
  msg.data[0] = x;
  msg.data[1] = y;
  msg.data[2] = theta;
  msg.cs = cs;

  if(msgsnd(msq_id,&msg,SHSPUR_MSG_SIZE , 0) < 0){
    /*error*/
  }   
  return 1;
}

int SHSpur_set_vel(double v){
  SHSpur_msg msg;

  msg.msg_type = SHSPUR_MSG_CMD;
  msg.pid  = pid;
  msg.type = SHSPUR_SET_VEL;
  msg.data[0] = v;

  if(msgsnd(msq_id,&msg,SHSPUR_MSG_SIZE , 0) < 0){
    /*error*/
  }
   
  return 1;
}
int SHSpur_set_angvel(double w){
  SHSpur_msg msg;

  msg.msg_type = SHSPUR_MSG_CMD;
  msg.pid  = pid;
  msg.type = SHSPUR_SET_ANGVEL;
  msg.data[0] = w;

  if(msgsnd(msq_id,&msg,SHSPUR_MSG_SIZE , 0) < 0){
    /*error*/
  }
   
  return 1;
}

/**/
double SHSpur_get_pos(int cs, double *x, double *y, double *theta)
{
  SHSpur_msg msg;
  int len;
  double time;

  msg.msg_type = SHSPUR_MSG_CMD;
  msg.pid  = pid;
  msg.type = SHSPUR_GET_POS; 
  msg.cs = cs;
  if(msgsnd(msq_id,&msg,SHSPUR_MSG_SIZE , 0) < 0){
    /*error*/
  }
  
  /*指定のコマンド受け取り*/
  len = msgrcv(msq_id,&msg, SHSPUR_MSG_SIZE , pid ,0);
  if(len < 0){
    /*receive error*/
    return 0;
  }

  *x     = msg.data[0];
  *y     = msg.data[1];
  *theta = msg.data[2];
  time = msg.data[3];
  return time;
}

/**/
double SHSpur_get_vel(double *v, double *w)
{
  SHSpur_msg msg;
  int len;
  double time;

  msg.msg_type = SHSPUR_MSG_CMD;
  msg.pid  = pid;
  msg.type = SHSPUR_GET_VEL; 
  msg.cs = 0;
  if(msgsnd(msq_id,&msg,SHSPUR_MSG_SIZE , 0) < 0){
    /*error*/
  }
  
  /*指定のコマンド受け取り*/
  len = msgrcv(msq_id,&msg, SHSPUR_MSG_SIZE , pid ,0);
  if(len < 0){
    /*receive error*/
    return 0;
  }

  *v     = msg.data[0];
  *w     = msg.data[1];
  time = msg.data[2];
  return time;
}

/**/
int SHSpur_parameter_set(int param_id,double value){
  SHSpur_msg msg;

  msg.msg_type = SHSPUR_MSG_CMD;
  msg.pid  = pid;
  msg.type = SHSPUR_PARAM_SET;
  msg.cs = param_id;
  msg.data[0] = value;

  if(msgsnd(msq_id,&msg,SHSPUR_MSG_SIZE , 0) < 0){
    /*error*/
  }
   
  return 1;
}

int SHSpur_parameter_state(int param_id, int state){
  SHSpur_msg msg;

  msg.msg_type = SHSPUR_MSG_CMD;
  msg.pid  = pid;
  msg.type = SHSPUR_PARAM_STATE;
  msg.cs = param_id;
  msg.data[0] = state;

  if(msgsnd(msq_id,&msg,SHSPUR_MSG_SIZE , 0) < 0){
    /*error*/
  }
   
  return 1;
}

int SHSpur_vel(double v, double w){
  SHSpur_msg msg;

  msg.msg_type = SHSPUR_MSG_CMD;
  msg.pid  = pid;
  msg.type = SHSPUR_VEL;
  msg.data[0] = v;
  msg.data[1] = w;

  if(msgsnd(msq_id,&msg,SHSPUR_MSG_SIZE , 0) < 0){
    /*error*/
  }
   
  return 1;
}


int SHSpur_wheel_vel(double r, double l){
  SHSpur_msg msg;

  msg.msg_type = SHSPUR_MSG_CMD;
  msg.pid  = pid;
  msg.type = SHSPUR_WHEEL_VEL;
  msg.data[0] = r;
  msg.data[1] = l;

  if(msgsnd(msq_id,&msg,SHSPUR_MSG_SIZE , 0) < 0){
    /*error*/
  }
   
  return 1;
}

int SHSpur_tilt(int cs, double dir, double tilt){
  SHSpur_msg msg;

  msg.msg_type = SHSPUR_MSG_CMD;
  msg.pid  = pid;
  msg.type = SHSPUR_SET_TILT;
  msg.data[0] = dir;
  msg.data[1] = tilt;
  msg.cs = cs;

  if(msgsnd(msq_id,&msg,SHSPUR_MSG_SIZE , 0) < 0){
    /*error*/
  }
   
  return 1;
}

int SHSpur_near_pos(int cs, double x, double y, double r){
  SHSpur_msg msg;
  int len;

  msg.msg_type = SHSPUR_MSG_CMD;
  msg.pid  = pid;
  msg.type = SHSPUR_NEAR_POS; 
  msg.data[0] = x;
  msg.data[1] = y;
  msg.data[2] = r;
  msg.cs = cs;
  if(msgsnd(msq_id,&msg,SHSPUR_MSG_SIZE , 0) < 0){
    /*error*/
  }
  
  /*指定のコマンド受け取り*/
  len = msgrcv(msq_id,&msg, SHSPUR_MSG_SIZE , pid ,0);
  if(len < 0){
    /*receive error*/
    return 0;
  }

  return msg.cs;
}
int SHSpur_near_ang(int cs, double th, double d){
  SHSpur_msg msg;
  int len;

  msg.msg_type = SHSPUR_MSG_CMD;
  msg.pid  = pid;
  msg.type = SHSPUR_NEAR_ANG; 
  msg.data[0] = th;
  msg.data[1] = d;
  msg.cs = cs;
  if(msgsnd(msq_id,&msg,SHSPUR_MSG_SIZE , 0) < 0){
    /*error*/
  }
  
  /*指定のコマンド受け取り*/
  len = msgrcv(msq_id,&msg, SHSPUR_MSG_SIZE , pid ,0);
  if(len < 0){
    /*receive error*/
    return 0;
  }

  return msg.cs;
}
int SHSpur_over_line(int cs, double x, double y, double theta){
  SHSpur_msg msg;
  int len;

  msg.msg_type = SHSPUR_MSG_CMD;
  msg.pid  = pid;
  msg.type = SHSPUR_OVER_LINE; 
  msg.data[0] = x;
  msg.data[1] = y;
  msg.data[2] = theta;
  msg.cs = cs;
  if(msgsnd(msq_id,&msg,SHSPUR_MSG_SIZE , 0) < 0){
    /*error*/
  }
  
  /*指定のコマンド受け取り*/
  len = msgrcv(msq_id,&msg, SHSPUR_MSG_SIZE , pid ,0);
  if(len < 0){
    /*receive error*/
    return 0;
  }

  return msg.cs;
}
