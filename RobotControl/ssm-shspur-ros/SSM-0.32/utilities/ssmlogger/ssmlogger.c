#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include "ymbc_time.h"
#include "ssm_common.h"

#define OPECODE 1
#define OPERAND 8

#define OPERAND_NAME 9
#define OPERAND_SUID 10
#define OPERAND_SIZE 11
#define OPERAND_LOGNAME 12

char g_name[100];
char g_logname[100];
int g_suid;
int g_size;
FILE *log_file;

void escape_road(void);
struct sigaction sa_sigint;
static void emergency(int);
static void emergency(int sig){
  fprintf(stderr,"program stop [%d]\n",sig);  
  fprintf(stderr,"finalize...\n");  
  
  /*終了処理*/
  if(log_file)fclose(log_file); 
  fprintf(stderr,"-file close\n");  
    
  while(1){
    fprintf(stderr,"please hit ctrl+c\n");
    sleep(1);
  };
}

void escape_road(void){
 sa_sigint.sa_handler = emergency;
 sa_sigint.sa_flags = SA_RESETHAND|SA_NODEFER;
 //sa_sigint.sa_mask = 0;
 sa_sigint.sa_restorer = 0;

 if (sigaction(SIGINT, &sa_sigint, NULL) < 0) {
   perror("sigaction");
   exit(1);
 }
}

int arg_analysis(int argc, char* argv[])
{
  int i;
  i = 1;

  while(i < argc){
    if(argv[i][0] == '-'){
      if(strcmp(argv[i]+1, "help")==0 || strcmp(argv[i]+1, "h")==0){
	printf("ssmlogger -name <sensor name> -suid <suid> -size <data size> -log <logname> \n");
	printf("ssmlogger -n <sensor name>    -i <suid>    -s <data size>    -l <logname> \n");
	i+=1;
      }else if(i+1 < argc){
	if(strcmp(argv[i]+1, "name")==0 || strcmp(argv[i]+1, "n")==0){
	  strcpy(g_name,argv[i+1]);
	  printf("name[%s]\n",g_name);
	  i+=2;
	}else if(strcmp(argv[i]+1, "suid")==0 || strcmp(argv[i]+1, "i")==0){
	  g_suid=strtol(argv[i+1],0,10);
	  printf("suid[%d]\n",g_suid);
	  i+=2;
	}else if(strcmp(argv[i]+1, "size")==0 || strcmp(argv[i]+1, "s")==0){
	  g_size=strtol(argv[i],0,10);
	  printf("size[%d]\n",g_size);
	  i+=2;
	}else if(strcmp(argv[i]+1, "log")==0||strcmp(argv[i]+1, "l")==0){
	  strcpy(g_logname,argv[i+1]);
	  printf("logname[%s]\n",g_logname);
	  i+=2;
	}else{
	  /*err*/
	  printf("argument error.\n");
	  return -1;
	}
      }else{
        /*err*/
	printf("argument error.\n");
	return -1;
      }
    }else{
      strcpy(g_logname,argv[i]);
      printf("logname[%s]\n",g_logname);
      i++;
    }
  }
  return 0;
}

int main(int argc,char* argv[])
{
  int tid,size,num,property_size;
  SSM_sid sid;
  char *buf, *prop_buf, fname[30];
  double ytime,cycle,wait;
  int count, fnum;
  long long int total;

  escape_road();
  strcpy(g_name, "sensor");
  strcpy(g_logname, "log.ssm");
  g_suid = 0;
  g_size = 4;
  if(arg_analysis(argc, argv)<0){
    printf("quit\n");
    return 0;
  }

  if(!initSSM())return 0;
  sid = openSSM(g_name, g_suid, 0);
  if(sid == 0){
    printf("Such sensor is not registerd.\n");
    return 0;
  }

  if(getSSM_info(g_name, g_suid, &size, &num, &cycle, &property_size)<0){
    printf("?\n");
    return 0;
  }

  printf("size = %d byte property = %d byte\n",size, property_size);
  if(!(buf = malloc(size))){
    printf("malloc error.\n");
    return 0;
  }
  if(!(prop_buf = malloc(property_size))){
    printf("property malloc error.\n");
    return 0;
  }

  /*logging*/
  log_file = fopen(g_logname, "w");
  
  tid = readSSM(sid, buf, &ytime, -1); /*get newest one data*/

  fprintf(log_file, "%s %d %d %d %lf %lf %d\n",
	  g_name, g_suid, size, num, cycle, ytime, property_size);
  printf("%s %d %d %d %lf %lf %d\n", 
	 g_name, g_suid, size, num, cycle, ytime, property_size);
  /*property*/
  if(property_size > 0){
    get_propertySSM(g_name,  g_suid, prop_buf);
    fwrite(prop_buf, property_size, 1, log_file);
  }

  wait = 0.2*1000000.0;
  count = 0;
  total = 0;
  fnum  = 0;
  //  if(wait > (cycle*tnum)*0.3)wait =(cycle*tnum)*0.3; 
  while(1){
    while(readSSM(sid, buf,&ytime,tid) <= 0){
      if(count)printf("get %s %d [%d]\n",g_name, tid, count);
      count = 0;
      
      /**/
      if((total > 1024*1024*1024) && 0){
	total = 0;
	fnum++;
	fclose(log_file);
	sprintf(fname, "%s_%d", g_logname, fnum);
	log_file = fopen(fname, "w");
	printf("split %s\n",fname);
	fprintf(log_file, "%s %d %d %d %lf %lf %d\n",
		g_name, g_suid, size, num, cycle, ytime, property_size);
	/*property*/
	if(property_size > 0){
	  get_propertySSM(g_name, g_suid, prop_buf);
	  fwrite(prop_buf, property_size, 1, log_file);
	}
      }else{
	usleep(wait);
      }   
    }

    fwrite(&ytime, sizeof(double), 1, log_file);
    fwrite(buf, size, 1, log_file);
    
    total += sizeof(double) + size;
    count++;
    tid++;
    
  }
  fclose(log_file);
  free(buf);  
  free(prop_buf);
  return 0;
}
