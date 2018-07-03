/*
  センサ共有メモリ管理共通設定
  12/13 Eijiro TAKEUCHI
  ・ライブラリ用ディレクトリに移動
*/


#ifndef __SSM_LOG__
#define __SSM_LOG__

#ifdef __cplusplus
extern "C" {
#endif

#include "ymbc_time.h"
#include <ssm_common.h>
#define MAX_LOG_NUM 20
#define END_OF_FILE -1

/*---------------------------defines------------------------------------  */

  void initSSMlog(void);
/*-------------------------function prototypes----------------------------  */
  SSM_sid openSSMlog(char *name, int* id);
/*------------------ read sensor data -----------------*/
  int proceedSSMlog(int i);
  int proceedSSMlog_time(double time);

 
#ifdef __cplusplus
  }
#endif

#endif
