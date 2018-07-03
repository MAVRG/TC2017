#include <ros/ros.h>
#include <geometry_msgs/Twist.h> 
#include <SHSpur.h>

class sh_spur{ 
    private : 
        ros::Subscriber sub;

    public : 
        sh_spur(); 
        void sh_spur_loop();

    private : 
        void sh_spurCallback(const geometry_msgs::Twist &twist); 

}; 


sh_spur::sh_spur(){ 

    ros::NodeHandle n; 
    sub = n.subscribe("/cmd_vel", 1,&sh_spur::sh_spurCallback,this);
} 

void sh_spur::sh_spur_loop() { 
    ros::Rate loop_rate(10); 

    while(ros::ok()) { 
        ros::spinOnce(); 
        loop_rate.sleep(); 
    } 

return; 
} 


void sh_spur::sh_spurCallback(const geometry_msgs::Twist &twist) { 
    SHSpur_vel(twist.linear.x,twist.angular.z);
    ROS_INFO("linear : %lf angular : %lf\n published.", twist.linear.x, twist.angular.z); 
    return; 
} 


int main(int argc, char **argv) { 
    ros::init(argc, argv, "sh_spur");
    ROS_INFO("start");
    SHSpur_init();
    sh_spur sh_spur; 
    sh_spur.sh_spur_loop(); 
    SHSpur_free();
    return (0); 

}
