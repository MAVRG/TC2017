/*
 *  Spur odometry data header file for SSM
 *
 */


#ifndef __SPURODOMETRY__
#define __SPURODOMETRY__

#define SNAME_ODOMETRY  "spur_odometry" 

typedef struct{
  double x;
  double y;
  double theta;
  double v;
  double w;
}Spur_Odometry;

#endif
