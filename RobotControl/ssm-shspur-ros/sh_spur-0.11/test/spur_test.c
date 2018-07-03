#include<stdio.h>
#include<unistd.h>
#include<math.h>

#include<SHSpur.h>

int main(int argc, char* argv[]){
  double x,y,theta;
  /*初期化*/
  SHSpur_init();
  
  SHSpur_set_vel(0.3);
  SHSpur_set_angvel(1.0);
  
  SHSpur_set_pos_GL(0, 0, 0);
  /*走行*/
  //SHSpur_line_GL(1,0,M_PI/2);
  SHSpur_line_GL(0,0.5,0);
  sleep(10);
  SHSpur_stop();
  return 0;

  while(!SHSpur_near_pos_GL(0.5 ,0 ,0.2))usleep(20000);
  //   SHSpur_spin_GL(M_PI/2);
  //X
  // while(!SHSpur_near_ang_GL(M_PI/2 ,0.1))usleep(50000);
  SHSpur_stop();
  sleep(4);
  SHSpur_free();
  while(1){
    SHSpur_get_pos_GL(&x,&y,&theta);
    printf("%f %f %f\n",x,y,theta*180.0/M_PI);
    usleep(100000);
  }
  return 0;
}
