/*
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <std_msgs/String.h>
//*******GLOBAL VARIABLES******************************
/**
 * Input to inverse kinematics equation (in PWM)
 * x: speed in x direction
 * y: speed in y direction
 * w: angular velocity
 */
float x=0,y=0,w=0;

/*****************************
 * Output to motors(in PWM)
 * ***************************
 * s[0]: front right motor
 * s[1]: front left motor
 * s[2]: back motor
 */
int s[3]={0,0,0};

/********************************
 * Direction of motors(in PWM) **
 ********************************
 * d[0]: front right motor
 * d[1]: front left motor
 * d[2]: back motor
 */
bool d[3]={1,1,1};

/*****************************
 * PWM pins of motors
 *****************************
 * s_p[0]: front right motor
 * s_p[1]: front left motor
 * s_p[2]: back motor
 */
int s_p[3]={3,6,9};

/********************************
 * Direction pins of motors
 ********************************
 * d_p[0]: front right motor
 * d_p[1]: front left motor
 * d_p[2]: back motor
 */
int d_p[3]={2,7,8};


//************************************************

ros::NodeHandle nh;

void messageCb( std_msgs::String msg)//const std_msgs::String::ConstPtr& msg)
{
//  Serial.print("String: ");
//  Serial.println(s);
  String s=msg.data;
  x=atof(s.substring(0,s.indexOf(",")).c_str());
  y=atof(s.substring(s.indexOf(",")+1,s.lastIndexOf(",")).c_str());
  w=atof(s.substring(s.lastIndexOf(",")+1).c_str());
//  Serial.println();
  
  inverseKinematics();
  motion();

}

ros::Subscriber<std_msgs::String> sub("robot_motion", &messageCb );

void setup()
{

  pinMode_setup();

  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  nh.spinOnce();
  delay(1);
}
