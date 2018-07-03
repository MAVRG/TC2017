/*
  センサ共有メモリ管理共通設定
  12/13 Eijiro TAKEUCHI
  ・ライブラリ用ディレクトリに移動
*/


#ifndef __SSM_COMMON__
#define __SSM_COMMON__

#ifdef __cplusplus
extern "C" {
#endif

#include "ymbc_time.h"



/*---------------------------defines------------------------------------  */

/*共有メモリアクセス用キー*/
#define SHM_KEY 0x3292
/*メッセージキューアクセス用キー*/
#define MSQ_KEY 0x3292

#define SHM_NUM 10


/*MessageCommand type*/
#define MSQ_CMD  1000
#define MSQ_RES  1001
#define MSQ_RES_MAX 2000
#define MCinit   0
#define MCcreate 1
#define MCopen   2
#define MCclose  3
#define MCgettid 4
#define MCsensor_list_info 5
#define MCsensor_list_num 6
#define MCsensor_list_name 7
#define MCsensor_property_set 8
#define MCsensor_property_get 9

#define MCres 31

#define MCread 0x20
#define MCwrite 0x40
#define MCexclusive 0x80

#define SSM_SNAME_MAX 32   /*センサ名の最大長さ（8n）*/

#define SSM_MARGIN 1       /*データアクセスタイミングの余裕
			     （書き込み中のデータを読まないようにするため）*/
#define SSM_TID_SP 0       /*tidの下限*/

#define createSSM_time(n,u,s,l,c)    createSSM(n,u,s,l,c)
/*---------------------------typedefs------------------------------------  */
typedef struct{
  int data;
  int num;
}shm_type;

typedef struct{
  char* data;
  double time;
  int id;
}ssm_data;


/*SSMのヘッダ*/
typedef struct{
  int   tid_top;   //最新のTID(rp = tid_top wp = tid_top+1 )
  int   num;       //履歴数
  int   size;      //データサイズ
  int   table_size;//TID検索テーブルのサイズ
  double cycle;    //データの入力される周期（最低値）
  int   data_off;  //データまでのオフセット
  int   times_off; //時刻データまでのオフセット
  int   table_off; //TID検索テーブルまでのオフセット
}ssm_header;


/*SSMコマンドメッセージ*/
typedef struct{
  long   msg_type;
  long   res_type;/*返信用*/
  int    cmd_type;
  char   name[SSM_SNAME_MAX];
  int    suid;
  int    ssize;
  int    hsize;
  YTime  time; 
}ssm_msg;


typedef struct{
  double ssmtime_settime;
  double ssmtime_offset;
  double ssmtime_speed;
}SSM_common_data;

/*SIDは実はただのアドレス*/
typedef char* SSM_sid;

#define SSM_MSG_SIZE  (sizeof(ssm_msg) - sizeof(long) )
//#define SSM_MSG_SIZE  (sizeof(ssm_msg))
/*-------------------------global variables--------------------------------  */

/*-------------------------function prototypes----------------------------  */
void errSSM(void);
void del_msg(void);
int initSSM(void);
SSM_sid createSSM(char *name,int sensor_uid,int ssm_size,YTime life,YTime cycle);

SSM_sid openSSM(char *name,int sensor_uid,char open_mode);
int getTID(SSM_sid sid,YTime ytime);
int getTID_time(ssm_header *shm_p, YTime ytime );
int setTID_time(ssm_header *shm_p, YTime ytime ,int tid);
int readSSM(SSM_sid sid,char *data,YTime *ytime,int tid);
int readSSM_time(SSM_sid sid, char *data, YTime ytime, YTime* ret_time);
int writeSSM(SSM_sid sid,char *data,YTime ytime);
int writeSSM_time(SSM_sid sid,char *data,YTime ytime);
int damp(SSM_sid sid,int mode,int num);

int getSSM_num(void);
int getSSM_name(int n, char *name, int *suid, int *size);
int getSSM_info(char *name, int suid, 
		int *size, int *num, double *cycle, int *property_size);
double gettimeSSM(void);
double settimeSSM(double offset_time,double speed);
int  set_propertySSM(char *name,int sensor_uid,char* data, int size);
int  get_propertySSM(char *name,int sensor_uid,char* data);

#ifdef __cplusplus
  }
#endif

#endif
