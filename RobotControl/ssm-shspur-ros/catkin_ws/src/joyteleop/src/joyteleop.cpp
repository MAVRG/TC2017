#include <ros/ros.h>
#include <sensor_msgs/Joy.h> 
#include <geometry_msgs/Twist.h> 
#include <cmath> 

class joyteleop{ 
    private : 
        ros::Subscriber sub; 
        ros::Publisher pub;
        geometry_msgs::Twist twist; 

    public : 
        joyteleop(); 
        void joyop_loop(); 

    private : 
        void joyCallback(const sensor_msgs::Joy &joy_msg); 

}; 


joyteleop::joyteleop(){ 

    ros::NodeHandle n; 
    sub = n.subscribe("joy", 1, &joyteleop::joyCallback, this); 
    pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1); 

} 

void joyteleop::joyop_loop() { 
    ros::Rate loop_rate(10); 

    while(ros::ok()) { 
        ros::spinOnce(); 
        pub.publish(twist); 
        loop_rate.sleep(); 
    } 

return; 
} 


void joyteleop::joyCallback(const sensor_msgs::Joy &joy_msg) { 
    if(joy_msg.buttons[1]==0.0){
       twist.linear.x = joy_msg.axes[4]; 
       twist.angular.z = joy_msg.axes[0]*joy_msg.axes[4];}
    else{
       twist.linear.x = fabs(joy_msg.axes[0]); 
       twist.angular.z = joy_msg.axes[0];}
    return; 

} 


int main(int argc, char **argv) { 

    ros::init(argc, argv, "joyteleop"); 
    joyteleop joyop; 
    joyop.joyop_loop(); 

    return (0); 

}

