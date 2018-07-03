#include<stdio.h>
#include<SHSpur.h>
#include<unistd.h>
#include<math.h>


int main(int argc, char* argv[]){
  SHSpur_init();

  SHSpur_set_vel(0.1);

  SHSpur_stop();
  usleep(100000);
  SHSpur_free();
  return 0;

}
