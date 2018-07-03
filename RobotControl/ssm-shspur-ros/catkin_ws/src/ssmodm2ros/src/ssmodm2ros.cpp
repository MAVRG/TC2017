#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <ssm_common.h>
#include <sensor/Spur_odometry.h>


int main(int argc, char** argv){
  SSM_sid odm_sid;
  Spur_Odometry odm;
  double odmtime;
  int tid;

  initSSM();
  odm_sid = openSSM("spur_odometry", 0, 0);

  ros::init(argc, argv, "ssmodm2ros");
  ros::NodeHandle node;
 
  tid=  readSSM(odm_sid, (char*)&odm, &odmtime, -1); 

  tf::TransformBroadcaster br;


  ros::Rate loop_rate(250);

  while(ros::ok()){
    while(readSSM(odm_sid, (char*)&odm, &odmtime, tid)<=-1)usleep(10000);
    tid++;

    tf::Transform transform;    
    tf::Quaternion q;
    transform.setOrigin( tf::Vector3(odm.x, odm.y, 0.0) );
    q.setRPY(0,0,odm.theta);
    transform.setRotation(q);

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));
    //printf("hoge\n");
    ros::spinOnce();
  }
  return 0;
};

