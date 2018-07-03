#ifndef __SH_SPUR__
#define __SH_SPUR__

double get_time(void);
/*CSの初期化*/
void init_coordinate_systems(void);
/*すれっどの初期化*/
void init_thread(pthread_t* thread);
/*SSMの初期化*/
void init_SSM(void);

#endif
