#ifndef __CONTROL__
#define __CONTROL__

#include<CoordinateSystem2D.h>

typedef struct _odometry *OdometryPtr;
typedef struct _odometry{
  double x;
  double y;
  double theta;
  double v;
  double w;
  double time;
}Odometry;

double circle_follow(OdometryPtr odm,double x,double y,double radius,double v_max);
double line_follow(OdometryPtr odm,double x,double y,double theta,double v_max);
int to_point(OdometryPtr odm, double x, double y, double theta,double max_vel);
double dist_pos(OdometryPtr odm, double x, double y);
double regurator(double d, double q, double r,  double v_max);
double trans_q(double theta);
double spin(OdometryPtr odm, double theta, double w_max);
void odometry(OdometryPtr xp, short cnt1, short cnt2,double dt);
void odm_logging(OdometryPtr ,double, double);
int odm_read(OdometryPtr odm, double *v, double *w);
void coordinate_synchronize(void);
void cs_odometry(CSptr target_cs, OdometryPtr odm, OdometryPtr dst_odm);
double gravity_compensation(Odometry* odm);
void run_control(void);


#endif
