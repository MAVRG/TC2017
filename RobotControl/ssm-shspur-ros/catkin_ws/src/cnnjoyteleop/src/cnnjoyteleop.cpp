#include <ros/ros.h>
#include <sensor_msgs/Joy.h> 
#include <geometry_msgs/Twist.h> 
#include <cmath> 

class CNNjoyteleop{ 
    private : 
        ros::Subscriber sub1;
        ros::Subscriber sub2; 
        ros::Publisher pub;
        geometry_msgs::Twist pubtwist;
	float CNNx = 0.0;
	float CNNz = 0.0;
	float button = 0.0;
	float Joyx = 0.0;
	float Joyz = 0.0;

    public : 
        CNNjoyteleop(); 
        void CNNjoyop_loop(); 

    private : 
        void CNNjoyCallback(const sensor_msgs::Joy &joy_msg); 
        void CNNtwistCallback(const geometry_msgs::Twist &twist); 
}; 


CNNjoyteleop::CNNjoyteleop(){ 

    ros::NodeHandle n; 
    sub1 = n.subscribe("joy", 1, &CNNjoyteleop::CNNjoyCallback, this);
    sub2 = n.subscribe("CNN/cmd_vel", 1,&CNNjoyteleop::CNNtwistCallback,this);
    pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1); 
} 

void CNNjoyteleop::CNNjoyop_loop() { 
    ros::Rate loop_rate(10); 

    while(ros::ok()) { 
        ros::spinOnce();
	if(button==0.0) {
	   pubtwist.linear.x = CNNx;
	   pubtwist.angular.z = CNNz;
	}
	else {
	   pubtwist.linear.x = Joyx;
	   pubtwist.angular.z = Joyz;
	}
        
        pub.publish(pubtwist); 
        loop_rate.sleep();
    } 

    return; 
} 


void CNNjoyteleop::CNNjoyCallback(const sensor_msgs::Joy &joy_msg) { 
    Joyx = joy_msg.axes[4]; 
    Joyz = joy_msg.axes[0]*joy_msg.axes[4];
    button = joy_msg.buttons[5];
    return; 
} 

void CNNjoyteleop::CNNtwistCallback(const geometry_msgs::Twist &CNNtwist) { 
    CNNx = CNNtwist.linear.x;
    CNNz = CNNtwist.angular.z;
    return; 
} 


int main(int argc, char **argv) { 

    ros::init(argc, argv, "CNNjoyteleop"); 
    CNNjoyteleop CNNjoyop; 
    CNNjoyop.CNNjoyop_loop();

    return (0); 

}

