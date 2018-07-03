#ifndef __LIBSHSPUR__
#define __LIBSHSPUR__

#ifdef __cplusplus
extern "C"{
#endif

#define SHSPUR_MSQ_KEY  0x7045
#define SHSPUR_MSG_CMD 1

#define CS_BS 0
#define CS_GL 1
#define CS_LC 2
#define CS_FS 3
#define CS_BL 4

enum{
  SHSPUR_FREE = 10,
    SHSPUR_SERVO,
    SHSPUR_LINE,
    SHSPUR_CIRCLE,
    SHSPUR_SET_VEL,
    SHSPUR_SET_ANGVEL,
    SHSPUR_GET_POS,
    SHSPUR_SET_POS,
    SHSPUR_SET_GL_GL,
    SHSPUR_ADJUST,
    SHSPUR_STOP,
    SHSPUR_VEL,
    SHSPUR_SET_TILT,
    SHSPUR_NEAR_POS,
    SHSPUR_NEAR_ANG,
    SHSPUR_OVER_LINE,
    SHSPUR_SPIN,
    SHSPUR_WHEEL_VEL,
    SHSPUR_GET_VEL,
    //
    SHSPUR_PARAM_SET = 100,
    SHSPUR_PARAM_STATE

};

/*パラメータの有効・無効*/
enum{
  SHSPUR_PARAM_MOTOR = 0,
    SHSPUR_PARAM_VELOCITY,
    SHSPUR_PARAM_BODY,
    SHSPUR_PARAM_TRACKING,
    SHSPUR_PARAM_GRAVITY
};

#define ENABLE 1
#define DISABLE 0

typedef struct{
  long   msg_type;
  long   pid;
  int    type;
  int    cs;
  double data[4];
}SHSpur_msg;

#define SHSPUR_MSG_SIZE (sizeof(SHSpur_msg) -sizeof(long))


/*SHSpurコマンド集*/
#define SHSpur_line_GL(x,y,th)    SHSpur_line(CS_GL, x,y,th)
#define SHSpur_line_LC(x,y,th)    SHSpur_line(CS_LC, x,y,th)
#define SHSpur_line_FS(x,y,th)    SHSpur_line(CS_FS, x,y,th)
#define SHSpur_line_BS(x,y,th)    SHSpur_line(CS_BS, x,y,th)
#define SHSpur_line_BL(x,y,th)    SHSpur_line(CS_BL, x,y,th)

#define SHSpur_circle_GL(x,y,d)    SHSpur_circle(CS_GL, x,y,d)
#define SHSpur_circle_LC(x,y,d)    SHSpur_circle(CS_LC, x,y,d)
#define SHSpur_circle_FS(x,y,d)    SHSpur_circle(CS_FS, x,y,d)
#define SHSpur_circle_BS(x,y,d)    SHSpur_circle(CS_BS, x,y,d)
#define SHSpur_circle_BL(x,y,d)    SHSpur_circle(CS_BL, x,y,d)

#define SHSpur_spin_GL(th)    SHSpur_spin(CS_GL, th)
#define SHSpur_spin_LC(th)    SHSpur_spin(CS_LC, th)
#define SHSpur_spin_FS(th)    SHSpur_spin(CS_FS, th)
#define SHSpur_spin_BS(th)    SHSpur_spin(CS_BS, th)
#define SHSpur_spin_BL(th)    SHSpur_spin(CS_BL, th)

#define SHSpur_set_pos_GL(x,y,th)    SHSpur_set_pos(CS_GL, x,y,th)
#define SHSpur_set_pos_LC(x,y,th)    SHSpur_set_pos(CS_LC, x,y,th)
#define SHSpur_set_pos_BL(x,y,th)    SHSpur_set_pos(CS_BL, x,y,th)

#define SHSpur_adjust_pos_GL(x,y,th)    SHSpur_adjust_pos(CS_GL, x,y,th)
#define SHSpur_adjust_pos_LC(x,y,th)    SHSpur_adjust_pos(CS_LC, x,y,th)
#define SHSpur_adjust_pos_FS(x,y,th)    SHSpur_adjust_pos(CS_FS, x,y,th)
#define SHSpur_adjust_pos_BS(x,y,th)    SHSpur_adjust_pos(CS_BS, x,y,th)
#define SHSpur_adjust_pos_BL(x,y,th)    SHSpur_adjust_pos(CS_BL, x,y,th)

#define SHSpur_get_pos_GL(x,y,th)    SHSpur_get_pos(CS_GL, x,y,th)
#define SHSpur_get_pos_LC(x,y,th)    SHSpur_get_pos(CS_LC, x,y,th)
#define SHSpur_get_pos_FS(x,y,th)    SHSpur_get_pos(CS_FS, x,y,th)
#define SHSpur_get_pos_BS(x,y,th)    SHSpur_get_pos(CS_BS, x,y,th)
#define SHSpur_get_pos_BL(x,y,th)    SHSpur_get_pos(CS_BL, x,y,th)

#define SHSpur_near_pos_GL(x,y,r)   SHSpur_near_pos(CS_GL,x,y,r)
#define SHSpur_near_pos_LC(x,y,r)   SHSpur_near_pos(CS_LC,x,y,r)
#define SHSpur_near_pos_BS(x,y,r)   SHSpur_near_pos(CS_BS,x,y,r)
#define SHSpur_near_pos_BL(x,y,r)   SHSpur_near_pos(CS_BL,x,y,r)

#define SHSpur_near_ang_GL(th,d)   SHSpur_near_ang(CS_GL,th,d)
#define SHSpur_near_ang_LC(th,d)   SHSpur_near_ang(CS_LC,th,d)
#define SHSpur_near_ang_BS(th,d)   SHSpur_near_ang(CS_BS,th,d)
#define SHSpur_near_ang_BL(th,d)   SHSpur_near_ang(CS_BL,th,d)

#define SHSpur_over_line_GL(x,y,th)   SHSpur_over_line(CS_GL,x,y,th)
#define SHSpur_over_line_LC(x,y,th)   SHSpur_over_line(CS_LC,x,y,th)
#define SHSpur_over_line_BS(x,y,th)   SHSpur_over_line(CS_BS,x,y,th)
#define SHSpur_over_line_BL(x,y,th)   SHSpur_over_line(CS_BL,x,y,th)

#define SHSpur_tilt_GL(d,t)          SHSpur_tilt(CS_GL,d,t)
#define SHSpur_tilt_LC(d,t)          SHSpur_tilt(CS_LC,d,t)
#define SHSpur_tilt_FS(d,t)          SHSpur_tilt(CS_FS,d,t)
#define SHSpur_tilt_BS(d,t)          SHSpur_tilt(CS_BS,d,t)
#define SHSpur_tilt_BL(d,t)          SHSpur_tilt(CS_BL,d,t)


int SHSpur_init(void);
int SHSpur_stop(void);
int SHSpur_free(void);
int SHSpur_line(int cs, double x, double y, double theta);
int SHSpur_circle(int cs, double x, double y, double r);
int SHSpur_spin(int cs, double theta);
int SHSpur_set_pos(int cs, double x, double y, double theta);
int SHSpur_adjust_pos(int cs, double x, double y, double theta);
int SHSpur_set_vel(double v);
int SHSpur_set_angvel(double w);
double SHSpur_get_pos(int cs, double *x, double *y, double *theta);
double SHSpur_get_vel(double *v, double *w);
int SHSpur_near_pos(int cs, double x, double y, double r);
int SHSpur_near_ang(int cs, double th, double d);
int SHSpur_over_line(int cs, double x, double y, double theta);

int SHSpur_vel(double v, double w);
int SHSpur_wheel_vel(double r, double l);

int SHSpur_tilt(int cs, double dir, double tilt);

/*裏コマンド集*/
int SHSpur_parameter_set(int param_id, double value);
int SHSpur_parameter_state(int state_id,int state);

#ifdef __cplusplus
  }
#endif

#endif
