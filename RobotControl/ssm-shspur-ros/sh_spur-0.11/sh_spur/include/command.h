#ifndef __COMMAND__
#define __COMMAND__
#include <SHSpur.h>

enum{
  RUN_FREE = 0,
    RUN_STOP,
    RUN_LINEFOLLOW,
    RUN_TO_POINT,
    RUN_CIRCLEFOLLOW,
    RUN_SPIN,
    RUN_VEL,
    RUN_WHEEL_VEL
};

void command(void);
void message_return(int msq_id, long retpid, SHSpur_msg* res_msg);

/*command_run.c*/
void line_com(int cs, double *data);
void circle_com(int cs, double *data);
void spin_com(int cs, double *data);
void stop_com(double *data);
void free_com(double *data);
void vel_com(double *data);
void wheel_vel_com(double *data);

/*command_set.c*/
void set_pos_com(int cs, double *data);
void set_GL_on_GL_com(double *data);
void set_adjust_com(int cs, double *data);
void set_vel_com(double *data);
void set_ang_vel_com(double *data);
void set_tilt_com(int cs, double *data);

/*command_get.c*/
void get_pos_com(int cs, double *data, double *resdata);
void get_vel_com(int cs, double *data, double *resdata);
int near_pos_com(int cs, double *data, double *resdata);
int near_ang_com(int cs, double *data, double *resdata);
int over_line_com(int cs, double *data, double *resdata);

/*command_param.c*/
void param_set_com(int cs, double *data);
void param_state_com(int cs, double *data);


#endif
