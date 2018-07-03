#ifndef __LIBSHSPUR__
#define __LIBSHSPUR__


#define SHSPUR_MSQ_KEY  0x7045

#define SHSPUR_MSG_CMD 1

enum{
  SHSPUR_FREE = 10,
    SHSPUR_SERVO,
    SHSPUR_LINE,
    SHSPUR_CIRCLE,
    SHSPUR_SET_VEL,
    SHSPUR_SET_ANGVEL,
    SHSPUR_GET_POS
};

typedef struct{
  long   msg_type;
  long   pid;
  int    type;
  double data[12];
}SHSpur_msg;

#define SHSPUR_MSG_SIZE (sizeof(SHSpur_msg) -sizeof(long))

/*SHSpurコマンド集*/
int SHSpur_init(void);
int SHSpur_line(double x, double y, double theta);
int SHSpur_circle(double x, double y, double r);
int SHSpur_set_vel(double v);
int SHSpur_set_angvel(double w);
double SHSpur_get_pos(double *x, double *y, double *theta);
#endif
