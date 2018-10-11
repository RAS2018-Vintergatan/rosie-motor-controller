#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <phidgets/motor_encoder.h>
#include <geometry_msgs/Twist.h>


double t_WLeft = 0;
double t_WRight = 0;

double KpLeft = 1.0;
double KpRight = 1.0;

double KiLeft = 1.0;
double KiRight = 1.0;

double KdLeft = 1.0;
double KdRight = 1.0;

double wlR = 1.0;
double wrR = 1.0;

double wheelSeparation = 1.0;

float motorLeftPWM;
float motorRightPWM;

double errorSumLeft = 0;
double errorSumRight = 0;

void setPWM(float motorLeft, float motorRight){
    motorLeftPWM = motorLeft;
    motorRightPWM = motorRight;
}

/*
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
int32 count
int32 count_change

*/
double lastErrorLeft = 0;
void encoderLeftCallback(const phidgets::motor_encoder& msg){
    ROS_INFO("-----------");
    ROS_INFO("LEFT:: E1: %d, E2: %d", msg.count,msg.count_change);
    
    double est_w = ((double)msg.count_change * 10.0 * 2.0 * 3.1415)/(360.0);
    
    double error = t_WLeft-est_w;

    errorSumLeft += error;
    double dError = (error-lastErrorLeft)/10;
  
    motorLeftPWM = (KpLeft*error + KiLeft*errorSumLeft*0.1 + KdLeft*dError);

    lastErrorLeft = error;

    ROS_INFO("-----------");
}
double lastErrorRight = 0;
void encoderRightCallback(const phidgets::motor_encoder& msg){
    ROS_INFO("-----------");
    ROS_INFO("RIGHT:: E1: %d, E2: %d", msg.count,msg.count_change);
    
    double est_w = ((double)msg.count_change * 10.0 * 2.0 * 3.1415)/(360.0);
    
    double error = t_WRight-est_w;

    errorSumRight += error;
    double dError = (error-lastErrorRight)/10;

    motorRightPWM = (KpRight*error + KiRight*errorSumRight*0.1 + KdRight*dError);

    lastErrorRight = error;

    ROS_INFO("-----------");
}

/*
geometry_msgs/Vector3 linear
  float64 x <-- 2D
  float64 y
  float64 z
geometry_msgs/Vector3 angular
  float64 x
  float64 y
  float64 z <-- 2D
*/
void controllerCallback(const geometry_msgs::Twist& msg){
    ROS_INFO("LinX: %f, AngZ: %f", 
	msg.linear.x, msg.angular.z);

    double wl = msg.linear.x/wlR;
    double wr = msg.linear.x/wrR;
    wl -= (wheelSeparation*msg.angular.z)/(2.0*wlR);
    wr += (wheelSeparation*msg.angular.z)/(2.0*wrR);

    t_WLeft = wl;
    t_WRight = -wr;

    errorSumLeft = 0;
    errorSumRight = 0;

    ROS_INFO("target Left: %f, target Right: %f",t_WLeft,t_WRight);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "rosie_motor_controller");

    ros::NodeHandle n;

    std::map<std::string,double> mLeft_pid, mRight_pid;
    n.getParam("motor_left_pid", mLeft_pid);
    n.getParam("motor_right_pid", mRight_pid);

    KpLeft = mLeft_pid["Kp"];
    KiLeft = mLeft_pid["Ki"];
    KdLeft = mRight_pid["Kd"];
    KpRight = mRight_pid["Kp"];
    KiRight = mRight_pid["Ki"];
    KdRight = mRight_pid["Kd"];

    n.getParam("wheel_left_radius", wlR);
    n.getParam("wheel_right_radius", wrR);

    n.getParam("wheel_separation", wheelSeparation);

    ros::Publisher motorLeft_pub = n.advertise<std_msgs::Float32>("/motorLeft/cmd_vel",1);
    ros::Publisher motorRight_pub = n.advertise<std_msgs::Float32>("/motorRight/cmd_vel",1);
    ros::Subscriber encoderLeft_sub = n.subscribe("/motorLeft/encoder", 1, encoderLeftCallback);
    ros::Subscriber encoderRight_sub = n.subscribe("/motorRight/encoder", 1, encoderRightCallback);
    ros::Subscriber controller_sub = n.subscribe("/motor_controller/twist", 1, controllerCallback);

    ros::Rate loop_rate(10);

    while(ros::ok()){

	std_msgs::Float32 motorLPWM;
	std_msgs::Float32 motorRPWM;

        motorLPWM.data = motorLeftPWM;
        motorRPWM.data = motorRightPWM;

        motorLeft_pub.publish(motorLPWM);
        motorRight_pub.publish(motorRPWM);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
