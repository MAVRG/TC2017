#include<stdio.h>
#include<SHSpur.h>
#include<unistd.h>
#include<math.h>
#include<fcntl.h>
#include <sys/termios.h>
#include<ssm_common.h>
#include "key_ssm.h"

#define DEV_NAME "/dev/input/js0"
#define MAX_VEL 0.6
#define MAX_ANGVEL 1.0
#define MAX_IR 20

int joy_fd;
int joystick_open(void);

int joystick_open(void){

  if(( joy_fd = open(DEV_NAME, O_RDONLY, 0)) == -1){
    fprintf(stderr,"Can't open %s\n",DEV_NAME);
    return 0;
  }

  tcflush(joy_fd,TCIOFLUSH);
  return 1;
}

int main(int argc, char* argv[]){
  double v,w,ir;
  int changed,is_free;
  char data [16];
  char do_control;
 YTime key_time;
  Key_Data key_data;
  SSM_sid key_sid;
  int     key_tid;

  if(!initSSM())return 0;
  if((key_sid = createSSM_time("keyboard",0,sizeof(Key_Data),10,0.1) ) <= 0){
    errSSM();
    printf("\nerr\n");
    return 0;
  }

  SHSpur_init();
  if(!joystick_open())return 0;
  v = 0;
  w = 0;
  ir = 0;
  changed = 1;
  do_control = 1;
  is_free = 1;
  while(do_control){
    if(read(joy_fd,data,8)>0){
      data[8] = 0;
      printf("%d %d %d %d %d %d %d %d\n",
	     data[0],data[1],data[2],data[3],
	     data[4],data[5],data[6],data[7]);

      if(data[4] ==1 && data[5]== 0 && data[6]==1){
	key_data.key = '0' + data[7];
	key_data.state = 1;

	writeSSM(key_sid, (char*)&key_data, gettimeSSM());   
	printf(">%c \n", key_data.key);
      }

      if(is_free){
	if(data[4] ==1 && data[5]== 0 && data[6]==1 && data[7]==0){
	  printf("Button3\n") ;
	  is_free = 0;
	  //SHSpur_servo();
	  SHSpur_vel(0,0);
	  //  do_control = 0;
	}
      }else{    
	if(data[6]==2 && (data[7]==0 ||data[7]==3 )){
	  printf("side %d\n",data[5]) ;
	  if(v < 0)w = MAX_ANGVEL*(double)data[5]/128.0;
	  else w = -MAX_ANGVEL*(double)data[5]/128.0;	
	  changed = 1;
	}
	if(data[6]==2 && data[7]==1){
	  printf("front %d\n",data[5]) ;
	  v = -MAX_VEL*(double)data[5]/128.0;
	  changed = 1;	  
	}
	if(data[4] ==1 && data[5]== 0 && data[6]==1 && data[7]==0){
	  printf("Button3\n") ;
	  is_free = 1;
	  //  do_control = 0;
	  SHSpur_free();
	}
	if(changed){
	  SHSpur_vel(v,w);
	  changed = 0;
	}
      }
    }
  }
  SHSpur_free();

  return 0;
}
