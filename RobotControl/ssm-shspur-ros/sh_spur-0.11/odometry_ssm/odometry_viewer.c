/*-----------------------------------------
 *-----------------------------------------*/

/*-------------include files-----------------*/
#include <unistd.h>
#include<ssm_common.h>
#include"Spur_odometry.h"
#include"OdometrySSM.h"

/*low level I/O*/
#include <fcntl.h>

/*high level I/O*/
#include <stdio.h>


int  main(int argc ,char *argv[])
{
  SSM_sid odm_bs_sid,odm_gl_sid;
  FILE *traj_file1, *traj_file2, *gnuplot;
  Spur_Odometry odm;
  double time,ox,oy,otheta,x,y,theta;
  int i;  

  initSSM();
  //odm_bs_sid = openSSM("spur_odometry", 0, 0);
  odm_gl_sid = openSSM("spur_odometry", 0, 0);

  gnuplot = popen("/usr/bin/gnuplot","w");
  //traj_file1 =fopen("odometry","w"); 
  traj_file2 =fopen("global","w"); 
 
  readSSM(odm_gl_sid, (char*)&odm, &time, -1); 
  ox = odm.x;
  oy = odm.y;
  otheta = odm.theta;
 
 //  fprintf(traj_file2,"%f %f %f\n",odm.x, odm.y, odm.theta);
  for(i = 0;i < 1200;i++){/*約30秒間*/
    /*オドメトリ表示*/
    // readSSM(odm_bs_sid, (char*)&odm, &time, -1); 
    //fprintf(traj_file1,"%f %f %f\n",odm.x, odm.y, odm.theta);
    /*グローバル*/
    readSSM(odm_gl_sid, (char*)&odm, &time, -1); 
    x = (odm.x-ox)*cos(-otheta)-(odm.y-oy)*sin(-otheta);
    y = (odm.x-ox)*sin(-otheta)+(odm.y-oy)*cos(-otheta);
    theta = odm.theta-otheta;
    printf("%f %f %f\n",x, y, theta);
    
    //fflush(traj_file1);
//    fflush(traj_file2);
    
//    fprintf(gnuplot,"plot 'global' w l\n");
//    fflush(gnuplot);
  
    usleep(100000);
  }

  //fclose(traj_file1);
  fclose(traj_file2);
  pclose(gnuplot);   
  
  return 1;
} 
