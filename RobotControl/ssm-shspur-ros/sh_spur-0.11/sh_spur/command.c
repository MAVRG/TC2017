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
extern double g_spur_dir;
extern double g_spur_tilt;

extern int g_run_mode;
extern Odometry g_odometry;
extern char g_state[];
extern double g_P[];
int g_do_quit;
extern double g_cnt2rad;
extern double g_2wradius;


/*コマンド受信*/
void command(void){
  SHSpur_msg  msg, res_msg;
  int msq_id, len;
  char param_name[PARAM_NUM][30] = PARAM_NAME;
  int before_run_mode;/*直前の走行モード*/

  printf("command anlyser\n");

  /*initialize message queue*/
  msq_id = msgget(SHSPUR_MSQ_KEY, 0666|IPC_CREAT);
  if(msq_id<0){
    /*queue_error*/
    printf("hoge\n");
  }

  g_spur_x = 0;
  g_spur_y = 0;
  g_spur_theta = 0;
  g_spur_v = 0;
  g_spur_w = 0;
  g_spur_d = 0;
  g_spur_tilt = 0;
  g_spur_dir = 0;
  g_run_mode = RUN_FREE;
  before_run_mode = -10000;

  g_do_quit = 0;
  printf("message come on.\n");  

  while(!g_do_quit){
    /*1コマンド取得*/    
    len = msgrcv(msq_id, &msg ,SHSPUR_MSG_SIZE , SHSPUR_MSG_CMD ,0);
    printf("one command received\n");

    pthread_mutex_lock(&mutex);
    /*コマンド解析・返信*/
    switch(msg.type){/*関数*/  
      /*--------command_set.c--------*/
    case SHSPUR_SET_POS:
      set_pos_com(msg.cs, msg.data);
      printf("set pos %f %f %f\n",msg.data[0], msg.data[1], msg.data[2]);
      break;
    case SHSPUR_SET_GL_GL:
      set_GL_on_GL_com(msg.data);  
      printf("GL on GL  %f %f %f\n",msg.data[0], msg.data[1], msg.data[2]);
      break;
    case SHSPUR_ADJUST:
      set_adjust_com(msg.cs, msg.data);
      printf("adjust  %f %f %f\n",msg.data[0], msg.data[1], msg.data[2]);
      break;
    case SHSPUR_SET_VEL:
      set_vel_com(msg.data);   printf("vel %f\n",g_spur_v);
      break;
    case SHSPUR_SET_ANGVEL: 
      set_ang_vel_com(msg.data);  printf("w %f\n",g_spur_w);
      break; 
    case SHSPUR_SET_TILT: 
      set_tilt_com(msg.cs, msg.data);  
      printf("tilt %f %f\n",g_spur_dir,g_spur_tilt);
      break;
      /*--------command_run.c----------*/
    case SHSPUR_LINE:
      line_com(msg.cs, msg.data);
      printf("line %f %f %f\n",g_spur_x,g_spur_y, g_spur_theta);
      break;
    case SHSPUR_CIRCLE:
      circle_com(msg.cs, msg.data);  
      printf("circle %f %f %f\n",g_spur_x,g_spur_y, g_spur_d);
      break;
    case SHSPUR_SPIN:
      spin_com(msg.cs, msg.data);  
      printf("spin %f",g_spur_theta);
      break;
    case SHSPUR_STOP:
      stop_com(msg.data);     printf("stop\n");
      break;
    case SHSPUR_FREE:
      free_com(msg.data);     printf("free\n");
      //      motor_free();
      break;
    case SHSPUR_VEL:
      vel_com(msg.data);      printf("vel %f %f\n",g_spur_v,g_spur_w);
      break;
    case SHSPUR_WHEEL_VEL:
      wheel_vel_com(msg.data);printf("wheelvel %f %f\n",g_spur_v,g_spur_w);
      break;

      /*----------command_get.c------------------*/
    case SHSPUR_GET_POS:
      get_pos_com(msg.cs, msg.data, res_msg.data);  
      message_return(msq_id, msg.pid, &res_msg);
      printf("get %f %f %f\n",res_msg.data[0],res_msg.data[1],res_msg.data[2]);
      break;
    case SHSPUR_GET_VEL:
      get_vel_com(msg.cs, msg.data, res_msg.data);  
      message_return(msq_id, msg.pid, &res_msg);
      printf("getvel %f %f\n",res_msg.data[0],res_msg.data[1]);
      break;
    case SHSPUR_NEAR_POS:
      res_msg.cs = near_pos_com(msg.cs, msg.data, res_msg.data);  
      message_return(msq_id, msg.pid, &res_msg);
      printf("get %f %f %f\n",res_msg.data[0],res_msg.data[1],res_msg.data[2]);
      break;
    case SHSPUR_NEAR_ANG:
      res_msg.cs = near_ang_com(msg.cs, msg.data, res_msg.data);  
      message_return(msq_id, msg.pid, &res_msg);
      printf("get %f %f %f\n",res_msg.data[0],res_msg.data[1],res_msg.data[2]);
      break;
    case SHSPUR_OVER_LINE:
      res_msg.cs = over_line_com(msg.cs, msg.data, res_msg.data);  
      message_return(msq_id, msg.pid, &res_msg);
      printf("get %f %f %f\n",res_msg.data[0],res_msg.data[1],res_msg.data[2]);
      break;
      /*-------------command_param.c---------------*/

    case SHSPUR_PARAM_SET:
      param_set_com(msg.cs, msg.data);
      printf("param_set %s %f\n",param_name[msg.cs],msg.data[0]); 
      break;
    
    case SHSPUR_PARAM_STATE:
      param_state_com(msg.cs, msg.data);
      break;

    default :
      //      printf("ss\n");
      break;
    }
    
    /*走行モードに変化があったか*/
    if(g_run_mode != before_run_mode){
      if(g_run_mode == RUN_FREE){/*フリーになった*/
	motor_free();
	printf("free mode\n");
      }
      if(before_run_mode == RUN_FREE){
	motor_servo();
	printf("servo mode %d\n",g_run_mode);
      }
      printf("change\n");
    }
    before_run_mode = g_run_mode;
    
    pthread_mutex_unlock(&mutex);
  }
  printf("command analyser stop.\n");  

}


/*メッセージを返す*/
void message_return(int msq_id, long retpid, SHSpur_msg* res_msg){
  res_msg->type = 0; 
  res_msg->msg_type = retpid;
  res_msg->pid = 0;
  msgsnd(msq_id, res_msg, SHSPUR_MSG_SIZE, 0);
}
