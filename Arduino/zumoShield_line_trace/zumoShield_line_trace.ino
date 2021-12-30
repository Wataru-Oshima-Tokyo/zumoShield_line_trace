#define USE_USBCON
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
//#include <sensor_msgs/Imu.h>
//#include <nav_msgs/Odometry.h>
#include <Wire.h>
#include <ZumoShield.h>


long timer=0;              // Elapsed time since program started (milli second)
//int16_t vright = 0;            // Left Morter velocity (speed of motor)
//int16_t vleft = 0;  // Right Morter velocity (speed of motor)
int16_t linearCoefficient = 400;// Base speed of Morter (Effective Range: 1 - 150)
int16_t angularCoefficient = 100;
float ForwardSpeed = 0.0f;
float AngularSpeed = 0.0f;
float cmd_velz=0.0f;
float leftSpeed =0.0f;
float rightSpeed =0.0f;
int16_t leftMotor =0;
int16_t rightMotor = 0;
bool recovery=false;
std_msgs::String str_msg;  // Sensor value to be published
//geometry_msgs::Twist cmd_vel; //cmd_vel value


LSM303 compass;            // Magnetometer
L3G gyro;                  // Gyrometer
ZumoMotors motors;     // Motor
//ZumoShieldEncoders encoders; // Encoder
ros::NodeHandle nh;        // NodeHandler of ROS



void recovery_behavior(){
    leftSpeed = -100;
    rightSpeed = -70; 
    motors.setSpeeds(leftSpeed, rightSpeed);
    delay(200);
    motors.setSpeeds(0, 0);
}

void control_Callback(const geometry_msgs::Twist& cmd_vel_sub)
{
   
   ForwardSpeed = cmd_vel_sub.linear.x*linearCoefficient;
   AngularSpeed = cmd_vel_sub.angular.z*angularCoefficient;
   cmd_velz = cmd_vel_sub.angular.z;
  
   if (cmd_velz<0){
    cmd_velz *=-1; //Absolute value
   }
  if (cmd_velz < 1.0 ) 
    {
       leftSpeed = ForwardSpeed;
       rightSpeed = ForwardSpeed*0.8;
    } else if (AngularSpeed > 0){
      //turn right
       AngularSpeed *=-1;
       if(ForwardSpeed <=0){
          recovery=true;
       }else{
        leftSpeed = ForwardSpeed;
        rightSpeed = ForwardSpeed;
       }

    } else{
       //turn left
       if(ForwardSpeed <=0){
         recovery=true;
       }else{
         leftSpeed = ForwardSpeed*1.3;
         rightSpeed = ForwardSpeed;        
       }

    }
   if(leftSpeed <=0){
    leftMotor = 0;
    rightMotor = (int)rightSpeed;
   }else if(rightSpeed<=0){
    leftMotor = (int)leftSpeed;
    rightMotor = 0;
   }else{
    leftMotor = (int)leftSpeed;
    rightMotor = (int)rightSpeed;
   }
   if(!recovery){
       motors.setSpeeds(leftSpeed, rightSpeed);
       delay(200);
       motors.setSpeeds(0, 0);    
   }else{
    recovery_behavior();
   }
   recovery=false;

}



ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", control_Callback);
ros::Publisher chatter("/sensorval", &str_msg);


void setup()
{

//  cli();
  /*
  * DEBUGにシリアルモニタを使用
  */
  Wire.begin();
//

  nh.initNode();           // Init ROS Node
  nh.advertise(chatter);// Init ROS Publisher
  nh.subscribe(sub);       // Init ROS Subscriber
  compass.init();          // Init magnetometer
  compass.enableDefault();

  gyro.init();             // Init gyrometer
  gyro.enableDefault();
  sei();
}


void loop() {
  // put your main code here, to run repeatedly:
  
  compass.read();   // Read magnetometer
  gyro.read();      // Read gyrometer
  
  timer = millis();
  String s = "";
  s += timer;
  s += ',';
//  s += compass.a.x;    // [1]  Accelerometer.x
//  s += ',';
//  s += compass.a.y;    // [2]  Accelerometer.y
//  s += ',';
//  s += compass.a.z;    // [3]  Accelerometer.z
  s += leftSpeed;
  s += ',';
  s += rightSpeed;
  str_msg.data = s.c_str();
  chatter.publish(&str_msg);
  nh.spinOnce();
//  delay(1);
}
