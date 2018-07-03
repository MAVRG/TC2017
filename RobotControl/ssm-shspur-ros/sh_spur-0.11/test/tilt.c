#include<stdio.h>
#include<math.h>

int main(int argc, char* argv[]){

  double tilt,theta,dir;

  dir = 0;//M_PI/2.0;
  tilt= M_PI/4;
  for(theta = 0;theta < M_PI*2.0;theta+=M_PI/20.0){
    printf("%f %f\n",theta, 180.0*atan(cos(theta-dir)*tan(tilt))/M_PI );
  }

}
