/*high level I/O*/
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <strings.h>
#include <pthread.h>

/*low level I/O*/
#include <fcntl.h>
#include <sys/time.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>

/*serial*/
#include <sys/termios.h>

/*ボディパラメータ*/
#include <sh_vel_parameters.h>

/*SSM 用*/
#ifdef SSM_ENABLE
#include <ssm_common.h>
#include <Spur_odometry.h>
#include <PWS_Motor.h>
#endif

/*sh_spur用*/
#include <serial.h>
#include <param.h>
#include <control.h>
#include <command.h>

/*ライブラリ用*/
#include <SHSpur.h>
#include <CoordinateSystem2D.h>

#ifdef SSM_ENABLE
extern SSM_sid g_odm_bs_sid,g_odm_sid,g_motor_sid, g_odm_adj_sid;
extern int g_ssm_enable;
extern int g_ssm_adj_enable;
#endif

/*pthread mutex用*/
extern pthread_mutex_t mutex;
/**/
extern Odometry g_odometry;
extern CSptr g_GL,g_LC,g_BS,g_FS;
extern int g_option;
extern char g_state[];
extern int g_run_mode;
extern double g_P[];
/**/
extern double g_spur_x;
extern double g_spur_y;
extern double g_spur_theta;
extern double g_spur_v;
extern double g_spur_w;
extern double g_spur_d;
/*傾き*/
extern double g_spur_tilt;
extern double g_spur_dir;


/*オドメトリ計算*/
void odometry(OdometryPtr xp, short cnt1, short cnt2,double dt){
  double v,w, wr, wl;
  
  /*角速度計算*/
  wr = 2.0*3.141592654*
    ((double)cnt2)/(g_P[COUNT_REV]*g_P[GEAR]*dt);
  wl = 2.0*3.141592654*
    ((double)cnt1)/(g_P[COUNT_REV]*g_P[GEAR]*dt);

  /*キネマティクス計算*/
  v = g_P[RADIUS_R]*wr/2.0  + g_P[RADIUS_L]*wl/2.0;
  w = g_P[RADIUS_R]*wr/g_P[TREAD]- g_P[RADIUS_L]*wl/g_P[TREAD];
  
  /*オドメトリ計算*/
  xp->x =  xp->x + v*cos(xp->theta)*dt;
  xp->y =  xp->y + v*sin(xp->theta)*dt;
  xp->theta = xp->theta+w* dt;    
  xp->time = gettimeSSM();
  xp->v = v;
  xp->w = w;
  /*-PI< <PIに調整*/
  //if(xp->theta <-M_PI)xp->theta += 2*M_PI;
  //if(xp->theta > M_PI)xp->theta -= 2*M_PI;

  /*FS座標系セット*/
  CS_set(g_FS, xp->x, xp->y,xp->theta );
}

/*-PI < theta < PIに調整する*/
double trans_q(double theta){
  while(theta > M_PI)theta -= 2.0*M_PI;
  while(theta < -M_PI)theta += 2.0*M_PI;
  return theta;
}

/*円弧追従*/
double circle_follow(OdometryPtr odm, double x, double y, double radius,
		   double v_max){
  double d,q,r,ang;
  
  r = sqrt((odm->x -x)*(odm->x -x) +(odm->y -y)*(odm->y -y));
  
  ang = atan2((odm->y - y), (odm->x - x));
  ang = trans_q(ang);

  // レギュレータ問題に変換
  d = fabs(radius) - r;
  q = trans_q(odm->theta - (ang + SIGN(radius) * (M_PI / 2.0)));
  
  return regurator(d, q, radius, v_max);
}

/*直線追従*/
double line_follow(OdometryPtr odm, double x, double y, double theta,
		   double v_max){
  double d,q;
  
  d = -(odm->x-x)*sin(theta) + (odm->y-y)*cos(theta);
  q = odm->theta - theta;
  q = trans_q(q);

  return regurator(d, q, 100, v_max);
}

/*軌跡追従レギュレータ*/
double regurator(double d, double q, double r,  double v_max){
  double nv,nw;
  double v,w;
  double cd;

  now_speed(&nv,&nw);  
  
  v = v_max - SIGN(v_max) * g_P[L_C1] * fabs(nw);
  
  cd = d;
  if(cd > g_P[L_DIST])cd = g_P[L_DIST];
  if(cd < -g_P[L_DIST])cd = -g_P[L_DIST];
  w = nw - g_P[CONTROL_CYCLE]*
    (SIGN(r)*SIGN(nv) * g_P[L_K1]*cd + g_P[L_K2]*q + g_P[L_K3]*nw);
  
  /*FF*/
  if(fabs(r)>0.1 )
    w += 2*nv / r ; 

  v = v_max;

  robot_speed_smooth(v, w);  
  return d;
}

/*回転*/
double spin(OdometryPtr odm, double theta, double w_max){
  double q, w_limit;
  double w;
  
  q = theta - odm->theta;
  q = trans_q(q);
  
  /*停止するのに限界の速度を計算*/
  w_limit = sqrt(0.8*2*g_P[MAX_ACC_W]*fabs(q)*g_P[CONTROL_CYCLE]/g_P[CYCLE]);

  if(w_max < w_limit){
    w = SIGN(q)*w_max;
  }else{
    w = SIGN(q)*w_limit;
    if(fabs(w) < M_PI/90.0)w = M_PI/90.0*SIGN(q);
  }
  
  robot_speed_smooth(0, w);  
  return fabs(q);
}

/*点までの距離*/
double dist_pos(OdometryPtr odm, double x, double y){
  double r;
  r = sqrt((odm->x -x)*(odm->x -x)+(odm->y -y)*(odm->y -y));
  
  return r;
}

/*直線の端点まで移動し止まる*/
int to_point(OdometryPtr odm, double x, double y, double theta,double max_vel){
  double dist,a ;
  double vel;
  int over;

  dist = dist_pos(odm, x, y);

  a = (odm->x -x)*cos(theta) + (odm->y-y)*sin(theta);
  over = 0;
  if(a > 0){
    vel = 0;
    over = 3;
  }else if(dist > max_vel){
    vel = max_vel;
  }else if(dist > 0.05){
    over = 1;
    vel = dist;  
  }else{
    over = 2;
    vel = dist;
  }

  line_follow(odm, x,y,theta,vel);
  return over;
}

/*Odometry型データの座標系を変換*/
void cs_odometry(CSptr target_cs, OdometryPtr odm, OdometryPtr dst_odm){
  double x,y,theta;
  x = odm->x;
  y = odm->y;
  theta = odm->theta;

  CS_recursive_trans(target_cs, g_BS, &x, &y, &theta );

  dst_odm->x = x;
  dst_odm->y = y;
  dst_odm->theta = theta;
  dst_odm->time = odm->time;
}

/*オドメトリ修正情報との融合*/
void coordinate_synchronize(void){
#ifdef SSM_ENABLE
  static double before_time;
  double now_time,time;
  Odometry bs_odometry, adj_odometry, target_pos;
  CoordinateSystem bs_cs, adj_cs;
  int tid;

  if(g_ssm_enable){/*20ms毎？*/
    if(!g_ssm_adj_enable){/*SSMcheck*/
      now_time = gettimeSSM();
      if(now_time > before_time+1){
	g_odm_adj_sid = openSSM("spur_global",1,0);
	if(g_odm_adj_sid > 0){//openできた
	  g_ssm_adj_enable = 1;
	  printf("find adjust information.\n");
	}else{
	  g_ssm_adj_enable = 0;
	}
	before_time = now_time;
      }
    }else{
      /*パラメータの変更がおこらないようにブロック*/
      pthread_mutex_lock(&mutex);

      //最新の修正位置
      if((tid = readSSM(g_odm_adj_sid, (char*)&adj_odometry,&now_time,-1))>=0){
	//同時刻のGL座標
	if(now_time > gettimeSSM()-1){
	  if((tid=
	      readSSM_time(g_odm_bs_sid,(char*)&bs_odometry,now_time,&time))>=0){
	    //時間が1秒以内（止まっていない）で、データがあるなら実行
	    /*座標系作成*/
	    bs_cs.x = bs_odometry.x;
	    bs_cs.y = bs_odometry.y;
	    bs_cs.theta = bs_odometry.theta;
	    
	    adj_cs.x = adj_odometry.x;
	    adj_cs.y = adj_odometry.y;
	    adj_cs.theta = adj_odometry.theta;
	   
	    /*修正後の位置を計算*/
	    target_pos = g_odometry;/*最新の位置*/
	    /*現在位置との微妙な差を計算*/
	    trans_cs(&bs_cs,&target_pos.x,&target_pos.y, &target_pos.theta);
	    /*微妙な差を付け加える*/
	    inv_trans_cs(&adj_cs,&target_pos.x,&target_pos.y,&target_pos.theta);
	    /*ロボット(FS)がGL上のx,y,thetaに見えるとするとき*/
	    /*FSからGLがどこに見えるか(GL->FS => FS->GL)*/
	    CS_turn_base(&target_pos.x,&target_pos.y,&target_pos.theta);
	    /*それはBS上のどこか*/
	    CS_recursive_trans(g_BS, g_FS,
			       &target_pos.x,&target_pos.y,&target_pos.theta);
	    /*GLをセット*/
	    CS_set(g_GL, target_pos.x, target_pos.y, target_pos.theta);
	  }
	}
      }
      /*保護終わり*/
      pthread_mutex_unlock(&mutex);

    }

  }
#endif
}

/*重力補償*/
double gravity_compensation(Odometry* odm){
  double tilt,f;
 
  /*傾きを計算*/
  tilt = atan(cos(odm->theta-g_spur_dir)*tan(g_spur_tilt));
  /*力を計算*/
  f = g_P[MASS]*GRAVITY*sin(tilt);
  /*[N]=[kg]*[m/ss]*tilt*/
  /*トルクを計算*/
  g_P[TORQUE_OFFSET] = f * g_P[RADIUS_R]/g_P[GEAR];
  /*[Nm]              [N]* [m]          /[in/out] */
  printf("%f %f\n",f,g_P[TORQUE_OFFSET]*g_P[TORQUE_UNIT]/2.0);
  parameter_set(PARAM_p_toq_offset,0,g_P[TORQUE_OFFSET]*g_P[TORQUE_UNIT]/2.0); 
  parameter_set(PARAM_p_toq_offset,1,g_P[TORQUE_OFFSET]*g_P[TORQUE_UNIT]/2.0); 
  return tilt;
} 

/*追従軌跡に応じた処理*/
void run_control(void){
  static double before_time;
  double now_time;
  Odometry gl_odometry;

  now_time = gettimeSSM();

  if(now_time > before_time+0.02 &&
     g_state[SHSPUR_PARAM_MOTOR]    == ENABLE  &&
     g_state[SHSPUR_PARAM_VELOCITY] == ENABLE){/*20ms毎？*/

    coordinate_synchronize();
    /*パラメータの変更がおこらないようにブロック*/
    pthread_mutex_lock(&mutex);
   
    cs_odometry(g_GL, &g_odometry, &gl_odometry);

    before_time = now_time;//+=0.02;
    
    /*重力補償*/
    if(g_state[SHSPUR_PARAM_GRAVITY] == ENABLE){
      gravity_compensation(&gl_odometry);      
    }

    /*走行状態に応じた処理*/
    switch(g_run_mode){
    case RUN_FREE:
      robot_speed_smooth(0,0);
      break;
    case RUN_STOP://ストップする（スピードを0にする）
      robot_speed_smooth(0,0);
      break;
    case RUN_WHEEL_VEL://速度直接指定
      motor_speed(g_spur_v, g_spur_w);
      break;
    case RUN_VEL://速度角速度指定
      if(g_state[SHSPUR_PARAM_BODY] == ENABLE)
	robot_speed_smooth(g_spur_v, g_spur_w);
      break;
    case RUN_LINEFOLLOW: //直線追従
      if(g_state[SHSPUR_PARAM_BODY] == ENABLE && 
	 g_state[SHSPUR_PARAM_TRACKING] == ENABLE)
	line_follow(&gl_odometry, g_spur_x, g_spur_y, g_spur_theta,g_spur_v);
      break;
    case RUN_TO_POINT: //短辺への移動
      if(g_state[SHSPUR_PARAM_BODY] == ENABLE && 
	 g_state[SHSPUR_PARAM_TRACKING] == ENABLE)
	to_point(&gl_odometry, g_spur_x, g_spur_y, g_spur_theta, g_spur_v);
      break;
    case RUN_CIRCLEFOLLOW: //円弧追従
      if(g_state[SHSPUR_PARAM_BODY] == ENABLE && 
	 g_state[SHSPUR_PARAM_TRACKING] == ENABLE)
	circle_follow(&gl_odometry, g_spur_x, g_spur_y, g_spur_d, g_spur_v);
      break;
    case RUN_SPIN://回転
      if(g_state[SHSPUR_PARAM_BODY] == ENABLE && 
	 g_state[SHSPUR_PARAM_TRACKING] == ENABLE)
	spin(&gl_odometry, g_spur_theta, g_spur_w);
      break;
    }
    /*保護終わり*/
    pthread_mutex_unlock(&mutex);

  }

}
