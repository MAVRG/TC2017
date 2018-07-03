#include <ros/ros.h>
#include <vector>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>

ros::Publisher  m_maxvel_pub;
double prev_speed;

double check_collision(double x, double y){ 
  if(x<0.2 && x>-0.6 && y > -0.3 && y <0.3){//own scan
    return 10;//max
  }else if(x<0.3 && x>0 && y > -0.3 && y <0.3){//front very risky
    return 0.01;
  }else if(x<1 && x>0 && y > -0.3 && y <0.3){//front follow
    return sqrt(x/4.0)+0.05;
  }else if(x<=0 &&x > -0.8 && y>-0.4 && y<0.4){//very near
    return 0.1;
  }else if(x<=0 &&x > -0.8 && y>-0.6 && y<0.6){//near
    return 0.3;
  }else{
    return 10;//max
  }
}


void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg){
  double angle=msg->angle_min;
  
  std::vector<double> scan_x;
  std::vector<double> scan_y;
  std_msgs::Float32 maxvel_out;
  double maxvel;

  maxvel=0.5;
  for(int i=0;i<msg->ranges.size();i++){
    if(msg->ranges[i]>0.2){
      double x= msg->ranges[i]*cos(angle);
      double y= msg->ranges[i]*sin(angle);
      
      double vel=check_collision(x,y);
      if(vel<maxvel)maxvel=vel;
    }
    angle+=msg->angle_increment;
  }
  
  if(maxvel>prev_speed+0.01){
    maxvel=prev_speed+0.01;   //40cm/ss
  }
  prev_speed=maxvel;

  maxvel_out.data=maxvel;
  m_maxvel_pub.publish(maxvel_out);
}

int main(int argc,char *argv[]){
  ros::init(argc, argv, "follow");
  ros::NodeHandle n;
  ros::Subscriber m_scan_sub;

  m_maxvel_pub = n.advertise<std_msgs::Float32>("/max_speed",10);
 
  m_scan_sub = n.subscribe("/scan", 10, &scan_callback);

  prev_speed=0.1;

  while(ros::ok()){
    usleep(100000);
    ros::spinOnce();
  }
}
