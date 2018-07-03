#ifndef __PWS_MOTOR__
#define __PWS_MOTOR__


typedef struct _pws_motor *PWSMotorPtr;
typedef struct _pws_motor{
  int counter1;
  int counter2;
  int pwm1;
  int pwm2;
}PWSMotor;

#endif
