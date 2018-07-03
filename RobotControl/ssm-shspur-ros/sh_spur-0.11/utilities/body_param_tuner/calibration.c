/*-----------------------------------------
 * body parameter calibration program
 *                   by  TKU
 *history
 *
 *memo:
 *2006/1/20
 *-----------------------------------------*/


/*-------------include files-----------------*/
#include <sys/types.h>
#include <sys/stat.h>

/*low level I/O*/
#include <fcntl.h>
#include <unistd.h>
#include <strings.h>

/*serial*/
#include <sys/termios.h>

/*high level I/O*/
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <math.h>

#include <SHSpur.h>
#include <ssm_common.h>
#include <sensor/PWS_Motor.h>

/*---------------defines--------------------*/
//#define DEBUG
#define PI 3.141592654

#define GEAR (5.4*45.0/16.0)
#define ENCODER 2048.0

/*------------global variables------------------*/

pthread_mutex_t mutex;
char do_count;
int enc_r,enc_l;

/*------------------------------------------------
 *                 functions
 *------------------------------------------------*/

void encoder_thread(void){
   int tid;
   SSM_sid motor_sid;
   PWSMotor motor_dat;
   double time;

   do_count = 1;

   enc_r = 0;
   enc_l = 0;   
   initSSM();
   motor_sid = openSSM("pws_motor",0,0);
   tid = readSSM(motor_sid, (char*)&motor_dat, &time, -1);
   while(do_count){
     if(readSSM(motor_sid, (char*)&motor_dat, &time, tid)<=0){
       usleep(100000);
     }

     enc_r+= motor_dat.counter1;
     enc_l+= motor_dat.counter2;
    
     tid++;
   }
   return ;
}

void get_encoder(int *l, int *r){
  
  pthread_mutex_lock(&mutex);  
  *l = enc_l;
  *r = enc_r;
  pthread_mutex_unlock(&mutex);
  
}


/*************************************************************

****************************************************************/
int  main(int argc ,char *argv[])
{
   double k,r,len,tread,n;
   pthread_t Encoder_thread;
   int s_r,s_l, e_r,e_l;   
   pthread_create(&Encoder_thread,NULL,(void*)encoder_thread,NULL);

   /*脱力*/
   SHSpur_free();
   
   printf("Body parameter calibration program.\n\n");
   printf("1: Rl:Rr の推定をします。\n");
   printf("　ロボットの背中を平な壁にぴったりくっつけてください。\n");
   printf("　enterキーで開始。\n");
   getchar();
   get_encoder(&s_l, &s_r);
   printf("   ロボットを大きいループを描く様に動かして、.\n");
   printf("   元の位置に戻り、再び背中をぴったり付けてください。.\n");
   printf("   この際、ロボットが一回転しないようにしてください。.\n");
   printf("　enterキーで終了。");
   getchar();
   get_encoder(&e_l, &e_r);
   k = ((double)e_r-(double)s_r)/((double)e_l-(double)s_l);
   printf("Rl:Rr = %f:1\n",k);
   
   
   printf("Rを推定します。\n");
   printf("まっすぐ走らせます。\n");
   printf("距離はどれくらいにしますか？[mm]\n");
   printf(">");
   scanf("%lf",&len);
   printf("len = %f\n",len);
   printf("スタート位置に着いたらenter\n");
   getchar();
   
   getchar();
   get_encoder(&s_l, &s_r);
   printf("ゴールに着いたらenter\n");
   getchar();
   get_encoder(&e_l, &e_r);
   
   r = len*ENCODER*GEAR/
     (PI*(  ((double)e_r-(double)s_r)+
	    k*((double)e_l-(double)s_l))  ); 
   
    printf("Rr = %f, Rl = %f\n",r,r*k);

    printf("トレッドの計算をします。\n");
    printf("何周回しますか？\n");
    printf(">");
    scanf("%lf",&n);
    getchar();
    printf("　ロボットの背中を平な壁にぴったりくっつけてください。\n");
    printf("　enterキーで開始。\n");
    getchar();
    get_encoder(&s_l, &s_r);
    printf("   ロボットを移動させ、%f周回して、\n",n);
    printf("   元の位置に戻り、再び背中をぴったり付けてください。.\n");
    printf("　enterキーで終了。");
    getchar();
    get_encoder(&e_l, &e_r);
    tread =  (r*(  ((double)e_r-(double)s_r)-
	         k*((double)e_l-(double)s_l) ))/(ENCODER*GEAR*n);   
    printf("tread = %f \n",tread);
    do_count = 0;    
    pthread_join(Encoder_thread, NULL);
   
    printf("done.\n");
    return 0;
}
