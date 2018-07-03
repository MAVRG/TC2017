#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h> 
#include <SHSpur.h>

class auto_sh_spur{ 
    private : 
        ros::Subscriber sub;

    public : 
        auto_sh_spur(); 
        void auto_sh_spur_loop();

    private : 
        void auto_sh_spurCallback(const geometry_msgs::TwistStamped &twist); 

}; 


auto_sh_spur::auto_sh_spur(){ 

    ros::NodeHandle n; 
    sub = n.subscribe("twist_cmd", 1,&auto_sh_spur::auto_sh_spurCallback,this);
} 

void auto_sh_spur::auto_sh_spur_loop() { 
    ros::Rate loop_rate(10); 

    while(ros::ok()) { 
        ros::spinOnce(); 
        loop_rate.sleep(); 
    } 

return; 
} 


void auto_sh_spur::auto_sh_spurCallback(const geometry_msgs::TwistStamped &twist) { 
    SHSpur_vel(twist.twist.linear.x,twist.twist.angular.z);
    ROS_INFO("linear : %lf angular : %lf\n published.", twist.twist.linear.x, twist.twist.angular.z); 
    return; 
} 


int main(int argc, char **argv) { 
    ros::init(argc, argv, "sh_spur");
    ROS_INFO("start");
    SHSpur_init();
    auto_sh_spur auto_sh_spur; 
    auto_sh_spur.auto_sh_spur_loop(); 
    SHSpur_free();
    return (0); 

}
