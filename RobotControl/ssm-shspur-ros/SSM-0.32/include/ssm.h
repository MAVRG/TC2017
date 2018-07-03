/*
  センサ共有メモリ管理設定
  12/13 eto
*/

#ifndef __SSM_MANAGER__
#define __SSM_MANAGER__

#include "ymbc_time.h"
#include "ssm_common.h"


/*---------------------------defines------------------------------*/



/*---------------------------typedefs------------------------------*/
typedef struct ssm_list *SSM_ListPtr;
typedef struct ssm_list{
  char name[SSM_SNAME_MAX];/*センサの名前*/
  int  suid;           /*センサ固有のID*/
  int  shm_id;         /*ブロックのある共有メモリのID*/
  char* shm_ptr;       /*ブロックのある共有メモリのアドレス(Manager内のみ有効)*/
  int   shm_offset;    /*共有メモリ内でブロックのある場所*/
  int   size;          /*ブロックのサイズ*/
  ssm_header *header;  /*ブロックのヘッダ*/
  SSM_ListPtr next;    /*次のリストへ*/
  char* property;      /*プロパティデータへのポインタ*/
  int   property_size;  /*プロパティデータの大きさ*/
}SSM_List;


/*----------------------global variables--------------------------*/


/*----------------------function prototypes-----------------------*/
int alloc_ssm_block(int ssize,int hsize,YTime cycle, char **shm_h,int *offset);
int alloc_ssm_common_data(void);
SSM_List* add_ssm_list(char *name,int suid,int ssize,int hsize,YTime cycle);
void free_ssm_list(SSM_List* ssmp);
int ssm_init(void);
SSM_List* serch_SSM_List(char* name,int suid);
int msq_loop(void);
SSM_List* get_nth_SSM_List(int n);
int get_num_SSM_List(void);
long get_receive_id(void);

#endif
