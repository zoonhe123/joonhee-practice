// 받은 속도값으로 모터 제어+엔코더 값을 읽어 퍼블리시하는 node
#include <ros.h>
#include <Wire.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
//#include <sensor_msgs/Imu.h>
#include <ros/time.h>


//#define SBUF_SIZE 64 // imu

//char sbuf[SBUF_SIZE]; // imu
//signed int sbuf_cnt=0; // imu

const byte noCommLoopMax = 10;                //number of main loops the robot will execute without communication before stopping
unsigned int noCommLoops = 0;                 //main loop without communication counter

const double radius = 0.085;  //바퀴 반지름[m] 우리껀0.064
const double width = 0.38;  //두 바퀴 사이 거리[m]
double linear_speed_cmd = 0; //AGV 선형 속도[m/s]
double angular_speed_cmd = 0; //AGV 각속도[rad/s]
double speed_cmd_left = 0;  //왼쪽 바퀴 속도[rpm]
double speed_cmd_right = 0; //오른쪽 바퀴 속도[rpm]

void publlish_encoder();
//void publish_imu();

ros::NodeHandle  nh;
/*
//---------------------imu function-------------------------------
int EBimuAsciiParser(float *item, int number_of_item)
{
  int n,i;
  int rbytes;
  char *addr; 
  int result = 0;
  
  rbytes = Serial2.available();
  for(n=0;n<rbytes;n++)
  {
    sbuf[sbuf_cnt] = Serial2.read();
    if(sbuf[sbuf_cnt]==0x0a)
       {
           addr = strtok(sbuf,",");
           for(i=0;i<number_of_item;i++)
           {
              item[i] = atof(addr);
              addr = strtok(NULL,",");
           }

           result = 1;

         // Serial.print("\n\r");
         // for(i=0;i<number_of_item;i++)  {  Serial.print(item[i]);  Serial.print(" "); }
       }
     else if(sbuf[sbuf_cnt]=='*')
       {   sbuf_cnt=-1;
       }

     sbuf_cnt++;
     if(sbuf_cnt>=SBUF_SIZE) sbuf_cnt=0;
  }
  
  return result;
}
//---------------------imu function-------------------------------
*/
// cmd_vel 콜백 함수
void AGVcontrol_cmd (const geometry_msgs::Twist& cmd_vel){
  noCommLoops = 0;   //Reset the counter for number of main loops without communication

  // cmd_vel에서 속도 추출
  linear_speed_cmd = cmd_vel.linear.x;
  angular_speed_cmd = cmd_vel.angular.z;
  
  speed_cmd_left = (linear_speed_cmd - (angular_speed_cmd * width / 2)) * 60/(2*3.14)/radius;
  speed_cmd_right = (linear_speed_cmd + (angular_speed_cmd * width / 2)) * 60/(2*3.14)/radius;

  // 모터제어기 명령 command
  String mvc_cmd = "mvc=";
  String comma = ",";

  mvc_cmd += String(speed_cmd_right) + comma + String(speed_cmd_left);
  Serial3.println(mvc_cmd); // 속도명령을 내리고 엔코더값을 읽어옴.

  publlish_encoder();// 엔코더 값을 읽어 퍼블리시하는 추가 함수
//  publish_imu();
}

// subscriber
ros::Subscriber<geometry_msgs::Twist> cmd_vel("cmd_vel", AGVcontrol_cmd);
// publisher
geometry_msgs::PointStamped wheel;
//sensor_msgs::Imu imu;
ros::Publisher enc_pub("encoder_data", &wheel);
//ros::Publisher imu_data("imu",&imu);

void setup()
{
//Serial2.begin(19200);
  Serial3.begin(38400);
  Serial3.println("co1=1");
  Serial3.println("co2=1");
  nh.initNode();
  nh.getHardware()->setBaud(57600);
  nh.subscribe(cmd_vel);
  nh.advertise(enc_pub);
//  nh.advertise(imu_data);

}

void loop()
{
  
  nh.spinOnce();
//  noCommLoops++;
//  if (noCommLoops == 65535)
//  {
//    noCommLoops = noCommLoopMax;
//  }
  delay(15);
}

void publlish_encoder(){

  String rec_encoder = "";
  String str_l_wheel, str_r_wheel;
  delay(5); ////데이터 수신 delay
  do {
      rec_encoder += (char)Serial3.read();

    } while (Serial3.available());
  // 읽어온 엔코더값은 mvc = _____, _____ 의 형태이므로 숫자값으로 바꿔줘야함
  int find_equal = rec_encoder.indexOf('=', 0); //from 0
  int find_dot = rec_encoder.indexOf(',', 0);
  int find_positive = rec_encoder.indexOf('-', 0);
  // 엔코더 값 추출
  for (int i = find_equal + 1; i < find_dot; i++) {str_l_wheel += rec_encoder[i];}
  for (int i = find_dot + 1; i < rec_encoder.length(); i++) {str_r_wheel += rec_encoder[i];}
  // 엔코더 값을 정수로 메시지에 추가
  wheel.point.x = str_l_wheel.toInt()*1.0; // 왼쪽 엔코더 값
  wheel.point.y = str_r_wheel.toInt()*1.0; // 오른쪽 엔코더 값
  wheel.point.z = 0.0;
  wheel.header.stamp = nh.now();

  enc_pub.publish(&wheel);
}
/*
void publish_imu(){
  
  float euler[9];
  
  if(EBimuAsciiParser(euler, 9)){

    imu.orientation.x = euler[0];
    imu.orientation.y = -euler[1];
    imu.orientation.z = -euler[2];
    imu.orientation.w = 0;

    imu.angular_velocity.x = euler[3];
    imu.angular_velocity.y = euler[4];
    imu.angular_velocity.z = euler[5];

    imu.linear_acceleration.x = euler[6];
    imu.linear_acceleration.y = euler[7];
    imu.linear_acceleration.z = euler[8];
    imu_data.publish(&imu);
  }
  
}
*/
