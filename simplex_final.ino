//Truck Ver.2
#include <stdio.h>
#include <Servo.h>
#include <math.h>
#include <RPLidar.h>

#include <ros.h>

#include <geometry_msgs/Twist.h>

#define _USE_MATH_DEFINES

//#define DEBUG
#define pin_throttle 10 // set pin number 8 of arduino mega as rc_throttle pin 
#define pin_steer 5 // set pin number 7 of arduino mega as rc_steer pin
#define RPLIDAR_MOTOR 3
#define LED 13
#define ZERO_STEER 1500
#define ZERO_SPEED 1500
#define GOAL_DIST 500.0

Servo steer;
Servo throttle;
RPLidar lidar;

int steer_val = ZERO_STEER;
int throttle_val = 1400;
float dist_val = GOAL_DIST;
float Kp = 0.5;

float tx_steer;
float tx_throttle;

bool mode = false;

float dist[360] = {0,};
int dist_count;
int dist_sum;
float dist_avg;
float dist_x;
unsigned long ros_time = 0;
unsigned long cur_time = 0;
unsigned long gap_time = 0;

void rosTwistCallback(const geometry_msgs::Twist& twist_msg){
  ros_time = millis();
  tx_throttle = twist_msg.linear.x;
  tx_steer = twist_msg.angular.z;


  throttle.writeMicroseconds(throttle_val);
  steer.writeMicroseconds(steer_val);

  /*
  Serial.print(twist_msg.angular.z);
  Serial.print(",");
  Serial.println(twist_msg.linear.x);
  */
}

ros::NodeHandle nh;


ros::Subscriber<geometry_msgs::Twist> sub_twist("twist_msg", &rosTwistCallback);





void setup() {
  nh.initNode();
  nh.subscribe(sub_twist);

  Serial.begin(57600);
  
  lidar.begin(Serial1);

  pinMode(RPLIDAR_MOTOR, OUTPUT);
  
  steer.attach(pin_steer);
  throttle.attach(pin_throttle);

  steer.writeMicroseconds(1500);
  throttle.writeMicroseconds(1500);
  
}

void loop() {
  // Performance Mode
  if (mode == true){
    nh.spinOnce();
    cur_time = millis();
    delay(5);

    gap_time = cur_time - ros_time;
    if (gap_time > 3000){
      mode = false;
    }
  }

  // Safety Mode
  else if (mode == false){
    dist_count = 0;
    dist_sum = 0;
    if (IS_OK(lidar.waitPoint())) {
      float distance = lidar.getCurrentPoint().distance; //distance value in mm unit
      float angle    = lidar.getCurrentPoint().angle; //anglue value in degree
      // Exception of LiDAR data (If the distance is under 150mm, this system is dead.)
      if (distance >= 150){
        dist[(int)angle] = distance;
      }
    
      for (int i = 40; i <= 50; i++){
        if (dist[i] != 0){
          dist_sum += dist[i];
          dist_count++;
        }
      }
    
      if (dist_count != 0){
       dist_avg = dist_sum / dist_count;  
      }
    
      dist_x = dist_avg * cos(M_PI / 4);
      //Serial.println(dist_x);
      
      //Control Gear
      throttle_val = 1600;
      // If 1500+ is left, convert - to +
      steer_val = ZERO_STEER + (dist_val - dist_x) * Kp;
    
      if (steer_val >= 1700){
        steer_val = 1700;
      }
      if (steer_val <= 1300){
        steer_val = 1300;
      }
      
      //Serial.println(steer_val);
    
      steer.writeMicroseconds(steer_val);
      throttle.writeMicroseconds(throttle_val);
      delay(5);
    }
  
  
    //perform data processing here...    
    else {
      analogWrite(RPLIDAR_MOTOR, 0); //stop the rplidar motor
      
      // try to detect RPLIDAR... 
      rplidar_response_device_info_t info;
      if (IS_OK(lidar.getDeviceInfo(info, 100))) {
        // detected... 
        lidar.startScan();
         
        // start motor rotating at max allowed speed
        analogWrite(RPLIDAR_MOTOR, 255);
        delay(1000);
      }
    }
  }
}
