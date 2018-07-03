/*
 * ssm_manager.c - 共有メモリ・管理プログラム
 *
 */

#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <ctype.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <sys/msg.h>
#include "ssm_common.h"
#include "ssm.h"


#define VERSION "0.32"

int       msq_id=-1;   /*メッセージキューのID*/
key_t     shm_key_num = 0;/*共有メモリの数*/
SSM_List* ssm_top = 0;
pid_t     my_pid;      /*自分のプロセスID*/
int       verbosity_mode = 0;/*メッセージ表示*/

int shmid_ssm_common_data;
SSM_common_data *shm_ssm_common_data;

void escape_road(void);
struct sigaction sa_sigint;
static void emergency(int);
static void emergency(int sig){
  fprintf(stderr,"program stop [%d]\n",sig);  
  fprintf(stderr,"finalize...\n");  
  
  /*終了処理*/
  free_ssm_list(ssm_top);
  ssm_top = 0;
  fprintf(stderr,"- all allocated shared memory released\n");  
  shmdt((char*)shm_ssm_common_data);/*デタッチ*/
  shmctl(shmid_ssm_common_data, IPC_RMID, 0);/*削除*/
  while(1){
    fprintf(stderr,"please hit ctrl+c\n");
    sleep(1);
  };
}

void escape_road(void){
 sa_sigint.sa_handler = emergency;
 sa_sigint.sa_flags = SA_RESETHAND|SA_NODEFER;
 sa_sigint.sa_restorer = 0;

 if (sigaction(SIGINT, &sa_sigint, NULL) < 0) {
   perror("sigaction");
   exit(1);
 }
}


void print_list(SSM_List* p)
{
  while(p){
    printf("name : %s\n",p->name);
    printf("ID: %d  offset: %d  size: %d  address: %ld \n   |\n",p->shm_id,p->shm_offset,p->size,(long)p->shm_ptr);
    p=p->next;
  }
}

/*宛先作成*/
long get_receive_id(void){
  static long id;
  if(id < MSQ_RES)id = MSQ_RES;
  
  id++;
  if(id > MSQ_RES_MAX)id = MSQ_RES;
  return id;
}

/*SSMの共通データ*/
int alloc_ssm_common_data(void){
 /*共有メモリ領域をげと*/
  if((shmid_ssm_common_data =
      shmget(SHM_KEY, sizeof(SSM_common_data), IPC_CREAT | 0666)) < 0)
    return 0;
  shm_key_num++;

  /*あたっち*/
  if((shm_ssm_common_data = 
      (SSM_common_data*)shmat(shmid_ssm_common_data, 0, 0)) < 
     (SSM_common_data *)0) {
    return 0;
  }

  shm_ssm_common_data->ssmtime_offset = 0;
  shm_ssm_common_data->ssmtime_settime = 0;
  shm_ssm_common_data->ssmtime_speed = 1;

  return 1;
}

/*データサイズ ssize,履歴数hsizeの共有メモリの領域を得る*/
/*現在は単に要求毎に共有メモリ領域を確保するのみ        */
/*将来的にはメモリ管理と似たような機構を入れる？        */
int alloc_ssm_block(int ssize, int hsize,YTime cycle, char **shm_h, int *offset)
{
  int s_id,i; /*  */

  /*共有メモリ領域をげと*/
  if((s_id = shmget(SHM_KEY+shm_key_num,
		    sizeof(ssm_header)+(ssize+sizeof(YTime)+sizeof(int))*hsize,
		    IPC_CREAT | 0666)) < 0)return 0;
  shm_key_num++;

  /*あたっち*/
  if((*shm_h = shmat(s_id, 0, 0)) < (char *)0) {
    return 0;
  }

  /*０で初期化？*/
  for(i = 0; i < (int)(sizeof(ssm_header)+(ssize+sizeof(YTime)+sizeof(int))*hsize); i++)
    (*shm_h)[i] = 0;

  /*現状では新たに得た領域なので、offsetは０*/
  *offset = 0;
  if(cycle <= 0)cycle = 1;

  /*ssm_header 初期化*/
  ((ssm_header*)*shm_h)->tid_top = SSM_TID_SP-1;  /*初期位置*/ 
  ((ssm_header*)*shm_h)->size = ssize;            /*データサイズ*/
  ((ssm_header*)*shm_h)->num = hsize;             /*履歴数*/
  ((ssm_header*)*shm_h)->table_size = hsize;      /*テーブルサイズ*/
  ((ssm_header*)*shm_h)->cycle = cycle;           /*データ最小サイクル*/
  ((ssm_header*)*shm_h)->data_off = sizeof(ssm_header);  /*データの先頭アドレス*/
  ((ssm_header*)*shm_h)->times_off = sizeof(ssm_header) + ssize*hsize;/*時刻の先頭アドレス*/
  ((ssm_header*)*shm_h)->table_off =sizeof(ssm_header) + (ssize+sizeof(YTime))*hsize;/*time tableの先頭アドレス*/

  return s_id;
}


/*管理用リストを最後尾に作る*/
SSM_List* add_ssm_list(char *name,
		   int suid,
		   int ssize,
		   int hsize,
		   YTime cycle)
{
  SSM_List *p,*q;
 
  p = (SSM_List*)malloc(sizeof(SSM_List));
  if(!p)return 0;
  strcpy(p->name,name);
  p->suid = suid;
  p->shm_id = alloc_ssm_block(ssize,hsize,cycle,&(p->shm_ptr),&(p->shm_offset));
  p->size =sizeof(ssm_header)+(ssize+sizeof(YTime)+sizeof(int))*hsize;
  p->header = (ssm_header*)(p->shm_ptr);
  p->next = 0;
  p->property = 0;
  p->property_size = 0;
  /*リストの最後にpを追加*/
  if(!ssm_top){
    ssm_top = p;
  }else{
    q = ssm_top;
    while(q->next){
      q = q->next;
    }
    q->next = p;
  }
  return p;
}

void free_ssm_list(SSM_List* ssmp){
  if(ssmp){
    if(ssmp->next)free_ssm_list(ssmp->next);
    if(ssmp->shm_ptr){
      shmdt(ssmp->shm_ptr);/*デタッチ*/
      shmctl(ssmp->shm_id, IPC_RMID, 0);/*削除*/
      printf("%s detached\n",ssmp->name);
    }/*
      ssmp->next = 0;
      
      if(ssmp->header)free(ssmp->header);
      if(ssmp->property)free(ssmp->property);
      free(ssmp);
    */
  }
}

void clean_shm(void){
  int i; 
  int s_id;

  printf("delete_all_sensor\n");
  
  for(i=0;i<40;i++){  
    if((s_id = shmget(SHM_KEY+i, 1, IPC_CREAT | 0666)) > 0){
      //shmdt(ssmp->shm_ptr);/*デタッチ*/
      shmctl(s_id, IPC_RMID, 0);/*削除*/
    }
  }
}


/*SSMの初期化*/
int ssm_init(void)
{
  /*メッセージキューのオープン*/
  if((msq_id = msgget(MSQ_KEY, IPC_CREAT | 0666)) < 0) {
    return -1;
  }
  
  my_pid = getpid();
  ssm_top = 0;
  
  //共通共有情報の場所を確保
  alloc_ssm_common_data();  

  printf("Message queue ready.\n");
  printf("Msg queue ID = %d \n",msq_id);
  //  printf("PID = %d\n",my_pid);
  return 1;
}


/*名前か、固有IDからセンサを検索する*/
/*名前優先で、同じ名前の物があれば固有IDから判断*/

SSM_List* search_SSM_List(char* name,int suid)
{
  SSM_List *p,*pn,*pni,*pi;

  p = ssm_top;

  pn = 0;pni = 0;pi = 0;
  while(p){
    if(strcmp(p->name,name) == 0){
      /*名前が同じ*/
      pn = p;
      if(p->suid == suid){
	/*名前も同じ*/
	pni = p;
      }
    }
    if(p->suid == suid){
      /*suid が同じ*/
      pi = p;
      //break;
    }
    p = p->next;
  }

  if(pni)return pni;/*名前とIDが一致*/
  //if(pn)return pn;  /*名前が一致*/
  //if(pi)return pi;  /*IDが一致*/
  return 0;
}

/*センサリストに登録されている数を取得する*/
int get_num_SSM_List(void)
{
  SSM_List* p;
  int num;

  p = ssm_top;
  num = 0;  
  while(p){
    num++;
    p = p->next;
  }

  return num;
}

/*センサリストからn番めのセンサのアドレスを取得する*/
SSM_List* get_nth_SSM_List(int n)
{
  SSM_List* p;
  p = ssm_top;
  
  while(p){
    n--;
    if(n < 0)return p;
    p = p->next;
  }

  p = 0;
  return p;
}


/*メッセージのやりとり*/
int msq_loop(void)
{
  ssm_msg msg;
  SSM_List *slist;
  int len,num;
  
  while(1){
    /*受信（待ち）*/
    if(verbosity_mode)printf("msg_ready\n");
    len = msgrcv(msq_id,&msg,SSM_MSG_SIZE,MSQ_CMD,0);
    if(len < 0)return 0;
     if(verbosity_mode)printf("msg_get\n");
    /*それぞれのコマンドへの応答*/
    switch((msg.cmd_type)&0x1f){
    case MCinit:
      msg.msg_type = msg.res_type;/*返信*/
      msg.cmd_type = MCres;
      msg.suid  = shmid_ssm_common_data;
      msg.ssize = 0;
      /*送信*/
      if((msgsnd(msq_id, &msg, SSM_MSG_SIZE, 0)) < 0)return 0;
      
       if(verbosity_mode)printf("init");
      break;
    case MCcreate:/*センサのメモリ確保*/
       if(verbosity_mode)printf("message:create!\n");

      /*リストの検索*/
      slist = search_SSM_List(msg.name, msg.suid);

      /*同じ物が無かったら*/
      if(!slist){   if(verbosity_mode)printf("   |   :add\n");
	/*リストに追加*/
	slist=add_ssm_list(msg.name, msg.suid, msg.ssize, msg.hsize, msg.time);
      }else{
	 if(verbosity_mode)printf("   |   :exist\n");
      }
      msg.msg_type = msg.res_type;/*返信*/
      msg.cmd_type = MCres;
      msg.suid  = slist->shm_id;
      msg.ssize = slist->shm_offset;
      /*送信*/
      if((msgsnd(msq_id, &msg, SSM_MSG_SIZE, 0)) < 0)return 0;
      
       if(verbosity_mode)print_list(ssm_top);
       if(verbosity_mode)printf("   |   :received shm_id%d offset%d\n",msg.suid,msg.ssize);
      break;

    case MCopen:/*センサのオープン*/
       if(verbosity_mode)printf("message:open!\n");
      slist = search_SSM_List(msg.name,msg.suid);
      if(slist){ if(verbosity_mode)printf("   |   :exist\n");
	msg.msg_type = msg.res_type;/*返信*/
	msg.cmd_type = MCres;
	msg.suid = slist->shm_id;
	msg.ssize = slist->shm_offset;
      }else{ if(verbosity_mode)printf("   |   :not found\n");
	msg.msg_type = msg.res_type;/*返信*/
	msg.cmd_type = MCres;
	msg.suid = -1;
	msg.ssize = 0;
      }
       /*送信*/
      if((msgsnd(msq_id, &msg, SSM_MSG_SIZE, 0)) < 0)return 0;
       if(verbosity_mode)printf("   |   :received shm_id%d offset%d\n",msg.suid,msg.ssize);
      break;
      
    case MCclose:
      break;
      
    case MCgettid:
      break;

    case MCsensor_list_num:
      /*センサのリストに登録されている数を取得する*/
      num = get_num_SSM_List();
      msg.msg_type = msg.res_type;/*返信*/
      msg.cmd_type = MCres;
      msg.suid = num;

      /*送信*/
      if((msgsnd(msq_id, &msg, SSM_MSG_SIZE, 0)) < 0)return 0;
       if(verbosity_mode)printf("   |   :sensor num %d\n",msg.suid);
      break;

    case MCsensor_list_name:
      /*センサリストからn番めのものの情報を取得する*/
      num = msg.suid;
      slist = get_nth_SSM_List(num);
      if(slist){ if(verbosity_mode)printf("   |   :exist\n");
	msg.msg_type = msg.res_type;/*返信*/
	msg.cmd_type = MCres;
	msg.suid     = slist->suid;/*ID*/ 
	msg.ssize    = slist->header->size;/*データサイズ*/
	//msg.hsize    = slist->table_size;/*記憶数*/
	//msg.time     = slist->cycle;/*平均周期*/
	strcpy(msg.name, slist->name);/*名前*/
      }else{ if(verbosity_mode)printf("   |   :not found\n");
	msg.msg_type = msg.res_type;/*返信*/
	msg.cmd_type = MCres;
	msg.suid = -1;
	msg.ssize = -1;
      }
      /*送信*/
      if((msgsnd(msq_id, &msg, SSM_MSG_SIZE, 0)) < 0)return 0;
       if(verbosity_mode)printf("   |   :received num%d = %s[%d]\n",num,msg.name,msg.suid);
      break;

    case MCsensor_list_info:
      /*名前で指定したセンサの情報を取得する*/
      if(verbosity_mode)printf("message:read_setting!\n");

      slist = search_SSM_List(msg.name, msg.suid);
      if(slist){ if(verbosity_mode)printf("   |   :exist\n");
	msg.msg_type = msg.res_type;/*返信*/
	msg.cmd_type = MCres;
	msg.ssize    = slist->header->size; /*データサイズ*/
	msg.suid     = slist->property_size; /*property size*/
	msg.time     = slist->header->cycle;/*平均周期*/
	msg.hsize    = slist->header->num;  /*履歴数*/
      }else{ if(verbosity_mode)printf("   |   :not found\n");
	msg.msg_type = msg.res_type;/*返信*/
	msg.cmd_type = MCres;
	msg.suid  = -1;
	msg.ssize = -1;
      }
       /*送信*/
      if((msgsnd(msq_id, &msg, SSM_MSG_SIZE, 0)) < 0)return 0;
       if(verbosity_mode)printf("   |   :received \n");
      break;

    case MCsensor_property_set:
      if(verbosity_mode)printf("message:property_set\n");

      slist = search_SSM_List(msg.name, msg.suid);
      if(slist){
	if(verbosity_mode)printf("   |   :exist\n");
	if(!slist->property){/*場所がなければ作る*/
	  slist->property = malloc(msg.ssize+sizeof(long));
	  slist->property_size = msg.ssize;
	}
	if(slist->property){/*場所がとれてれば書き込む*/ 
	  /*受信準備OKの返信*/
	  msg.msg_type = msg.res_type;/*返信*/
	  msg.res_type = get_receive_id();/*宛先はこちら*/
	  msg.cmd_type = MCres;
	  msg.ssize    = slist->property_size; /*データサイズ*/
	  if((msgsnd(msq_id, &msg, SSM_MSG_SIZE, 0)) < 0)return 0;
	  /*データを読み込む*/
	  msgrcv(msq_id, (char*)slist->property,
		 slist->property_size, msg.res_type, 0);
	}else{
	  if(verbosity_mode)printf("   |   :mem error\n");
	  msg.msg_type = msg.res_type;/*返信*/
	  msg.cmd_type = MCres;
	  msg.ssize    = 0;
	  if((msgsnd(msq_id, &msg, SSM_MSG_SIZE, 0)) < 0)return 0;	
	}
      }else{
	if(verbosity_mode)printf("   |   :not found\n");
	msg.msg_type = msg.res_type;/*返信*/
	msg.cmd_type = MCres;
	msg.ssize = 0;
	if((msgsnd(msq_id, &msg, SSM_MSG_SIZE, 0)) < 0)return 0;	  
      }
      break;

    case MCsensor_property_get:
      if(verbosity_mode)printf("message:propeerty_get\n");

      slist = search_SSM_List(msg.name, msg.suid);
      if(slist){
	if(verbosity_mode)printf("   |   :exist\n");
	if(slist->property && slist->property_size){/*場所がとれてれば書き込む*/ 
	  /*受信準備OKの返信*/
	  msg.msg_type = msg.res_type;/*返信*/
	  msg.cmd_type = MCres;
	  msg.ssize    = slist->property_size; /*データサイズ*/
	  if((msgsnd(msq_id, &msg, SSM_MSG_SIZE, 0)) < 0)return 0;
	  
	  /*データを送る*/
	  *((long*)slist->property) = msg.res_type;/*返信*/
	  msgsnd(msq_id, (char*)slist->property,
		 slist->property_size, 0);
	}else{
	  if(verbosity_mode)printf("   |   :mem error\n");
	  msg.msg_type = msg.res_type;/*返信*/
	  msg.cmd_type = MCres;
	  msg.ssize    = 0;
	  if((msgsnd(msq_id, &msg, SSM_MSG_SIZE, 0)) < 0)return 0;	
	}
      }else{
	if(verbosity_mode)printf("   |   :not found\n");
	msg.msg_type = msg.res_type;/*返信*/
	msg.cmd_type = MCres;
	msg.ssize = 0;
	if((msgsnd(msq_id, &msg, SSM_MSG_SIZE, 0)) < 0)return 0;	  
      }
      break;
    default:
      break;
    }
  }

  return 1;
}


int main(int argc,char* argv[])
{
  printf("\n");
  printf(" --------------------------------------------\n");
  printf(" SSM(Sensor Sharing Manager).  \n");
  printf(" Ver. %s \n",VERSION);
  printf(" ------------------------------------------- \n\n");
  
  /*避難口の用意*/
  escape_road();
  
  ssm_top = 0;
   
  ssm_init();
  clean_shm();
  msq_loop();
  
  return 0;
}

