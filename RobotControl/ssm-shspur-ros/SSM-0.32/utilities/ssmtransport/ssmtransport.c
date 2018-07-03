#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <sys/times.h>
#include <sys/time.h>
#include <errno.h>
#include <time.h>
#include<sys/socket.h>
#include<sys/types.h>
#include<sys/stat.h>
#include<netinet/in.h>
#include<arpa/inet.h>
#include<sys/termios.h>
#include "ymbc_time.h"
#include <ssm_common.h>
#include "ssmtransport.h"
#include <pthread.h>

int sockfd,infd;
struct sockaddr_in srv;

pthread_mutex_t mutex;

char g_server_ip[16];
char g_send_sensors_file[100];

/*サーバ側オープン*/
int open_tcpip(int port_num)
{
  socklen_t socklen;
  
  if((sockfd = socket(PF_INET, SOCK_STREAM, 0)) < 0){
    printf("socket err.\n");
    return 0; 
  }
  memset(&srv, 0 ,sizeof(srv));
  srv.sin_family = AF_INET;
  srv.sin_port = htons(port_num);
  
  socklen = sizeof(srv);
  if((bind(sockfd, (struct sockaddr *)&srv, socklen)) < 0){
    printf("bind");
    return 0;
  }  
  if(listen(sockfd,5) < 0){
    printf("listen");
    return 0;
  }
  /*connect*/
  puts("TCP/IP socket available");
  printf("\tport %d\n", ntohs(srv.sin_port));
  printf("\taddr %s\n",inet_ntoa(srv.sin_addr));
  if((infd = accept(sockfd, (struct sockaddr *)&srv,&socklen )) >= 0)
    puts("new connection granted.");
  return 1;  
}

int open_tcpip_cli(char* ip_address, int port_num)
{
  socklen_t socklen;
  struct sockaddr_in cli;
  
  if((infd = socket(PF_INET, SOCK_STREAM, 0)) < 0){
    printf("socket\n");
    return 0;
  }
  memset(&cli, 0 ,sizeof(cli));
  cli.sin_family = AF_INET;
  cli.sin_port = htons(port_num);
  if(!(inet_aton(ip_address,&cli.sin_addr))){
    printf("inet_aton\n");
    return 0;
  }
  socklen = sizeof(cli);
  if(connect(infd, (struct sockaddr *)&cli, socklen)){
    printf("connect error\n");
    return 0;
  }

  /*connect*/
  puts("connected to TCP/IP socket");
  printf("\tport %d\n", ntohs(cli.sin_port));
  printf("\taddr %s\n",inet_ntoa(cli.sin_addr));
  return 1;
}

/*新しいセンサを登録する*/
int regist_new_sensor(SSMT_newsensor *new_sensor, RegSensPtr sensor ,int num){
  sprintf(sensor->name, "%s",new_sensor->name); 
  sensor->suid = new_sensor->suid;
  if((sensor->sid = createSSM_time(new_sensor->name, new_sensor->suid, 
			       new_sensor->size, new_sensor->time,
			       new_sensor->cycle))<=0)return 0;
  sensor->size = new_sensor->size;
  sensor->max_num = new_sensor->time/new_sensor->cycle;
  sensor->property_size = new_sensor->property_size;
  sensor->data = malloc(new_sensor->size);
 
  if(new_sensor->property_size)
    sensor->property = malloc(new_sensor->property_size);
  else
    sensor->property = 0;
  
  sensor->id = num;
  
  if(!sensor->data)return 0;
  return 1;
}


/*TCP/IPで送信*/
int send_sensor_data(RegSensPtr sensor){
  int rp,size;
  SSMT_header header;

  /*ヘッダ送信*/
  header.head = SSMT_HEAD;
  header.type = SSMT_DATA;
  header.id   = sensor->id; 
  header.size = sensor->size;
  header.time = sensor->time;
  write(infd, &header, sizeof(SSMT_header));

  /*データ送信*/
  rp = 0;
  while(rp < sensor->size){
    size = 1000;
    if(size > sensor->size - rp)size = sensor->size-rp;
    rp += write(infd,sensor->data + rp, size);
    //    printf(">"); 
  }
  return 1;
}

int send_newsensor(RegSensPtr sensor, char* name, int suid){
  static int id;
  SSMT_newsensor new_sensor;
  SSMT_header header;
  int num;

  sprintf(new_sensor.name, "%s",name);
  new_sensor.suid = suid;

  if((sensor->sid = openSSM(name, suid, 0)) <= 0)return 0;
  if(getSSM_info(name,suid, &new_sensor.size, &num, &new_sensor.cycle, &new_sensor.property_size)<0)return 0;
    
  new_sensor.time = (double)num*new_sensor.cycle;
  /*ヘッダ送信*/
  header.head = SSMT_HEAD;
  header.type = SSMT_NEW;
  header.id   = 0; 
  header.size = sizeof(SSMT_newsensor);
  write(infd, &header, sizeof(SSMT_header));
  write(infd, &new_sensor, sizeof(SSMT_newsensor));
  /*プロパティ送信*/
  if(new_sensor.property_size >0){
    sensor->property = malloc(new_sensor.property_size);    
    printf("get property\n");
    get_propertySSM(name, suid, (char*)sensor->property);
    header.head = SSMT_HEAD;
    header.type = SSMT_PROPERTY;
    header.id   = suid; 
    header.size = new_sensor.property_size;
    write(infd, &header, sizeof(SSMT_header));
    write(infd, (char*)sensor->property, new_sensor.property_size);
  }else{
    sensor->property = 0;
  }
  sensor->id = id;
  sensor->suid = new_sensor.suid;
  sensor->max_num = num;
  sensor->size = new_sensor.size;
  sensor->property_size = new_sensor.property_size;
  sprintf(sensor->name,"%s",name);
  sensor->data = malloc(sensor->size);

  sensor->tid = readSSM(sensor->sid, sensor->data, &sensor->time,-1);
  if(sensor->tid<=0)sensor->tid=1;  
  id++;
  return 1;
}

/*新しいデータがあったら送信する*/
int ssm2tcp(char *filename){
  RegisterdSensor out_sensor[20];
  int out_sensor_num =0;
  int updated;
  char name[30];
  int suid,i,tid;
  FILE *send_file;

  /*センサの登録処理*/
  send_file = fopen(filename,"r");
  if(!send_file){
    printf("File [%s] does not exist.",filename);
    return 0;
  }

  //pre regist
  out_sensor_num=0;
  while(fscanf(send_file, "%s %d",
	       out_sensor[out_sensor_num].name,
	       &(out_sensor[out_sensor_num].suid)) != EOF){
    out_sensor[out_sensor_num].sid=0;
    out_sensor[out_sensor_num].time=gettimeSSM()-2;
    out_sensor_num++;
  }
  fclose(send_file);

  /*センサデータの送信処理*/
  while(1){
    updated = 0;
    for(i = 0;i< out_sensor_num;i++){
      if(out_sensor[i].sid==0){

	if(out_sensor[i].time +1< gettimeSSM()){//check
	  out_sensor[i].time = gettimeSSM();
	  if(send_newsensor(&out_sensor[i], 
			    out_sensor[i].name,
			    out_sensor[i].suid)){
	    	    printf("Sensor %s[%d] registerd.\n",
		   out_sensor[i].name,
		   out_sensor[i].suid);
	  }else{
	    out_sensor[i].sid = 0;
	  }
	}    
	continue;
      }
      tid=readSSM(out_sensor[i].sid,out_sensor[i].data,&out_sensor[i].time,-1);
      //printf("%d %d\n",tid,out_sensor[i].tid);
      if(tid - out_sensor[i].tid > out_sensor[i].max_num/2){
	printf("skip %s\n",out_sensor[i].name);
	out_sensor[i].tid= tid-out_sensor[i].max_num/2;
      }else{
	if(readSSM(out_sensor[i].sid,  out_sensor[i].data, 
		 &out_sensor[i].time, out_sensor[i].tid)>0){
	out_sensor[i].tid++;
	
	send_sensor_data(&out_sensor[i]); /*送信する*/      
	updated = 1;
	printf("send %s %d [%d]\n",out_sensor[i].name,out_sensor[i].tid-1,tid);
	}
      }

    }
    if(!updated)usleep(10000);
  }
}

/*TCP/IPのメッセージを解釈して、SSMに入力する*/
void tcp2ssm(void)
{
  int len;
  RegisterdSensor in_sensor[20];
  int in_sensor_num= 0;
  SSMT_header header;
  SSMT_newsensor  new_sensor;
 
  while(1){
    /*ヘッダ読み込み*/
    len = 0;
    while(len < sizeof(SSMT_header))
      len += read(infd,((char*)&header)+len,sizeof(SSMT_header)-len);
    /*header check*/
    if(header.head != SSMT_HEAD){
      printf("header_error\n");
      return;
    }
    
    /*それぞれのタイプに応じて読み込み*/
    switch(header.type){
    case SSMT_NEW:
      len = 0;
      while(len < sizeof(SSMT_newsensor))
	len += read(infd,((char*)&new_sensor)+len,sizeof(SSMT_newsensor)-len);
      if(!regist_new_sensor(&new_sensor, &in_sensor[in_sensor_num], in_sensor_num))return;
      printf("sensor %s[%d] registerd.\n",in_sensor[in_sensor_num].name,
	     in_sensor[in_sensor_num].suid);
      in_sensor_num++;
      break;
    case SSMT_PROPERTY:
      len = 0;
      while(len < header.size)
	len += read(infd,
		    ((char*)in_sensor[header.id].property)+len,
		    header.size-len);
      set_propertySSM(in_sensor[header.id].name,
		      in_sensor[header.id].suid,
		      in_sensor[header.id].property,
		      in_sensor[header.id].property_size);
      printf("set property\n");       
      break;
    case SSMT_DATA:
      len = 0;
      while(len < header.size)
	len+=read(infd,((char*)in_sensor[header.id].data)+len,header.size-len);
      writeSSM(in_sensor[header.id].sid,
	       in_sensor[header.id].data,
	       header.time);
      //printf("one data\n");
      break;
    default:
      break;
    }
  }
}



int arg_analysis(int argc, char* argv[])
{
  int i;
  i = 1;

  while(i < argc){
    if(argv[i][0] == '-'){      
      if(strcmp(argv[i]+1, "help")==0 || strcmp(argv[i]+1, "h")==0){
	printf("ssmtransport <server IP>   client\n");
	printf("ssmtransport               server\n");
	printf("Options \n");
	printf("-sensors, -s:  send sensor file\n");
	i+=1;
      }else if(i+1 < argc){
	if(strcmp(argv[i]+1, "sensors")==0 || strcmp(argv[i]+1, "s")==0){
	  strcpy(g_send_sensors_file, argv[i+1]);
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
      strcpy(g_server_ip, argv[i]);
      printf("server_ip is %s\n",g_server_ip);
      i++;
    }
  }

  return 0;
}

int main(int argc, char* argv[])
{
  pthread_t receive_thread;
  
  /*initialize*/
  pthread_mutex_init(&mutex,NULL);

  initSSM();

  g_server_ip[0]=0;
  sprintf(g_send_sensors_file,"send_sensors");
  //  settimeSSM(0,0); 
 
  arg_analysis(argc,argv);


  if(g_server_ip[0] != 0){
    printf("client\n");
    if(!open_tcpip_cli(g_server_ip, 50000)){
      printf("bind error\n");
      return 0;  
    }
  }else{
    printf("server\n");
    if(!open_tcpip(50000)){
      printf("bind error\n");
      return 0;  
    }
  }

  printf("hoge\n");
  if(pthread_create(&receive_thread,NULL,(void*)tcp2ssm, NULL) != 0){
    /*エラー処理*/
  }


  ssm2tcp(g_send_sensors_file);

  pthread_join(receive_thread,NULL);

  return 0;
}
