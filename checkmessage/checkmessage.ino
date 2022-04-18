// AGV_teleop_key 에서 publish하는 메시지가 잘 받아지는지 확인하는 node
#include <ros.h>
#include <Wire.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <ros/time.h>

#define LOOPTIME  100 

const double radius = 0.032;  //바퀴 반지름[m] 우리껀0.064
const double wheelbase = 0.17;  //축간 거리[m] 우리건 0.17
double linear_speed_cmd = 0; //AGV 선형 속도[m/s]
double angular_speed_cmd = 0; //AGV 각속도[rad/s]
double speed_cmd_left = 0;  //왼쪽 바퀴 속도[m/s]
double speed_cmd_right = 0; //오른쪽 바퀴 속도[m/s]

void publishSpeed(double time);
volatile double velocity_Right,velocity_Left; 

ros::NodeHandle  nh;

void AGVcontrol_cmd (const geometry_msgs::Twist& cmd_vel){

  // cmd_vel에서 속도 추출
  linear_speed_cmd = cmd_vel.linear.x;
  angular_speed_cmd = cmd_vel.angular.z;

  // 각 모터에 속도 명령을 주기 위해 변환
  // 각속도 > 0 : 좌회전, 왼쪽 모터 속도-, 오른쪽 모터 속도+
  // 각속도 < 0 : 우회전, 왼쪽 모터 속도+, 오른쪽 모터 속도-
  speed_cmd_left = linear_speed_cmd - angular_speed_cmd * (wheelbase / 2);
  speed_cmd_right = linear_speed_cmd + angular_speed_cmd * (wheelbase / 2);
}
ros::Subscriber<geometry_msgs::Twist> check_vel("cmd_vel", AGVcontrol_cmd);
geometry_msgs::Vector3Stamped speed_msg;//create a "speed_msg" ROS message
ros::Publisher speed_pub("speed", &speed_msg);

void setup()
{

  nh.initNode();
//  nh.getHardware()->setBaud(57600);
  nh.subscribe(check_vel);
  nh.advertise(speed_pub);
}

void loop()
{
  nh.spinOnce();
  publishSpeed(LOOPTIME);
  delay(10);

}
void publishSpeed(double time) {
  velocity_Left = speed_cmd_left;
  velocity_Right = speed_cmd_right;
  speed_msg.header.stamp = nh.now();      //timestamp for odometry data
  speed_msg.vector.x = velocity_Left;  //left wheel speed (in m/s)
  speed_msg.vector.y = velocity_Right;   //right wheel speed (in m/s)
  speed_msg.vector.z = time / 1000;       //looptime, should be the same as specified in LOOPTIME (in s)
  speed_pub.publish(&speed_msg);
  nh.spinOnce();
}
