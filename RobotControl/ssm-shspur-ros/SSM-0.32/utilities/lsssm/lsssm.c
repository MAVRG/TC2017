#include <stdio.h>
#include <string.h>
#include "ymbc_time.h"
#include "ssm_common.h"

int main(int argc,char* argv[])
{
  int  sid_search,i,j;
  char name[100];
  int  suid, size, num, property_size;
  double cycle;
  SSM_sid sid;
  double top_time,time;
  int top_tid,tid;
  int total_byte;

  if(!initSSM())return 0;

  sid_search = 0;
  top_time = gettimeSSM();
  total_byte = 0;
  printf("%f\n",top_time);

  printf("No.|    sensor name     | id |  size  |  num |life[s]|cycle[s]| prop |count/s|\n");
  printf("---+--------------------+----+--------+------+-------+--------+------|-------|\n");
  while(getSSM_name(sid_search , name, &suid, &size)>=0){
    if(getSSM_info(name, suid, &size, &num, &cycle, &property_size)<0){
      printf("SSM read error.\n");
      return 0;
    }

    sid = openSSM(name, suid, 0);

    tid = top_tid = readSSM(sid,0,&time,-1);
    while(readSSM(sid,0,&time,tid)>=0){
      if(top_time-time >  2){
	break;
      }
      if(top_time-time <= 1)top_tid = tid;
      if(top_time-time < -1)break;
           
      tid--;
    }
    total_byte+=size*(top_tid-tid);
    
    i = strlen(name);
    if(i >= 20)i = 0;
    else i = 20-i;
    j = i/2;
    i = i-j;
    printf("%02d |",sid_search);
    for(;i>0;i--)printf(" ");
    printf("%.20s",name);
    for(;j>0;j--)printf(" ");
    printf("| %2d |",suid);
    if(size<1000)printf("%7d ",size);
    else if(size < 1000*1024) printf("%7.2fK",(double)size/1024.0);
    else printf("%7.2fM",(double)size/(1024.0*1024.0));
    printf("| %4d | %5.1f | %6.3f | %4d | %5d |\n",num,cycle*num, cycle, property_size,top_tid-tid);


    sid_search++;
  }
  printf("---+--------------------+----+--------+------+-------+--------+------+-------|\n");

  printf("Total %d sensors. Total %6.2fMB/s\n",sid_search,(double)total_byte/(1024*1024.0));

  return 0;
}
