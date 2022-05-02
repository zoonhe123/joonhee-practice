// 받은 속도값으로 모터 제어+엔코더 값을 읽어 퍼블리시하는 node
#include <ros.h>
#include <Wire.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
//#include <geometry_msgs/Vector3Stamped.h>
#include <ros/time.h>


//initializing all the variables

#define LOOPTIME  100                         //Looptime in millisecond
const byte noCommLoopMax = 10;                //number of main loops the robot will execute without communication before stopping
unsigned int noCommLoops = 0;                 //main loop without communication counter

const double radius = 0.032;  //바퀴 반지름[m] 우리껀0.064
const double wheelbase = 0.17;  //축간 거리[m] 우리건 0.17
double linear_speed_cmd = 0; //AGV 선형 속도[m/s]
double angular_speed_cmd = 0; //AGV 각속도[rad/s]
double speed_cmd_left = 0;  //왼쪽 바퀴 속도[m/s]
double speed_cmd_right = 0; //오른쪽 바퀴 속도[m/s]

void extract_velocity_Right();
void extract_velocity_Left();
//void publishSpeed(double time);
void publlish_encoder();


volatile double v1,v2;  //v1 = Right velocity, v2 = Left velocity
volatile double velocity_Right,velocity_Left; 
ros::NodeHandle  nh;

// cmd_vel 콜백 함수
void AGVcontrol_cmd (const geometry_msgs::Twist& cmd_vel){
  noCommLoops = 0;   //Reset the counter for number of main loops without communication

  // cmd_vel에서 속도 추출
  linear_speed_cmd = cmd_vel.linear.x;
  angular_speed_cmd = cmd_vel.angular.z;

  // 각 모터에 속도 명령을 주기 위해 변환
  // 각속도 > 0 : 좌회전, 왼쪽 모터 속도-, 오른쪽 모터 속도+
  // 각속도 < 0 : 우회전, 왼쪽 모터 속도+, 오른쪽 모터 속도-
  speed_cmd_left = linear_speed_cmd - angular_speed_cmd * (wheelbase / 2);
  speed_cmd_right = linear_speed_cmd + angular_speed_cmd * (wheelbase / 2);

  // 모터제어기 명령을 위해 m/s --> rpm 단위환산
  String rpm_cmd = "mvc=";
  String rpm_R = "";
  String rpm_L = ",";
  
  //right
  speed_cmd_right *= 60/(2*3.14)/radius;
  rpm_R += String(speed_cmd_right);
  
  //left
  speed_cmd_left *= 60/(2*3.14)/radius;
  rpm_L += String(speed_cmd_left);
  
  // mvc = __,__ 형식의 command 를 만들어 Serial3.println
  rpm_cmd += rpm_R + rpm_L;
  Serial3.println(rpm_cmd); // 속도명령을 내리고 엔코더값을 읽어옴.

  publlish_encoder();// 추가 함수

  //loop delay for callback time
  delay(100);
}

//subscriber
ros::Subscriber<geometry_msgs::Twist> cmd_vel("cmd_vel", AGVcontrol_cmd);

//geometry_msgs::Vector3Stamped speed_msg;//create a "speed_msg" ROS message
//ros::Publisher speed_pub("speed", &speed_msg);

//---------------추가된 퍼블리셔--------------------------------------
std_msgs::Int32 l_wheel;
ros::Publisher l_enc_pub("left_encoder", &l_wheel);
std_msgs::Int32 r_wheel;
ros::Publisher r_enc_pub("right_encoder", &r_wheel);
//------------------------------------------------------------------

void setup()
{
  Serial3.begin(57600);
  
  nh.initNode();
  nh.getHardware()->setBaud(57600); // 이거 왜하는지 모르겠음
  nh.subscribe(cmd_vel);
//  nh.advertise(speed_pub);
  nh.advertise(l_enc_pub);
  nh.advertise(r_enc_pub);
}

void loop()
{
  
  nh.spinOnce();
  noCommLoops++;
  if (noCommLoops == 65535)
  {
    noCommLoops = noCommLoopMax;
  }
  extract_velocity_Right();
  extract_velocity_Left();
//  publishSpeed(LOOPTIME);
}

//void publishSpeed(double time) {
//  speed_msg.header.stamp = nh.now();  //timestamp for odometry data
//  speed_msg.vector.x = velocity_Left; //left wheel speed (in m/s)
//  speed_msg.vector.y = velocity_Right;  //right wheel speed (in m/s)
//  speed_msg.vector.z = time / 1000; //looptime, should be the same as specified in LOOPTIME (in s)
//  speed_pub.publish(&speed_msg);
//  nh.spinOnce();
////  nh.loginfo("Publishing odometry");
//}

//--------------추가한 코드-------------------------------------
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
  l_wheel.data = str_l_wheel.toInt();
  r_wheel.data = str_r_wheel.toInt();
  // 엔코더 값 퍼블리시
  l_enc_pub.publish(&l_wheel);
  r_enc_pub.publish(&r_wheel);
}
//--------------------------------------------------------------

void extract_velocity_Right() {
    String recv_str3 = "";
    String str_v1, str_v2;
    bool positive;
    Serial3.println("v");
    delay(5); ////데이터 송신후 수신 delay
    do {
      recv_str3 += (char)Serial3.read();
    } while (Serial3.available());
    //'=', '.' 인덱스 넘버 찾기
    int find_equal = recv_str3.indexOf('=', 0); //from 0
    int find_dot = recv_str3.indexOf('.', 0);
    int find_positive = recv_str3.indexOf('-', 0);
    //속도 값 추출
    for (int i = find_equal + 1; i < recv_str3.length(); i++) {str_v1 += recv_str3[i];}
    if (find_positive < 0) positive = true;
    else if (find_positive > 0) positive = false;
    double v1_1 = str_v1.toInt();
    double v1_2 = str_v1.substring(find_dot - 2, find_dot).toInt() / 100.;
    v1 = positive ? v1_1 + v1_2 : v1_1 - v1_2;
    velocity_Right = v1*radius*2*3.141592/60;
}
void extract_velocity_Left(){
    String recv_str3 = "";
    String str_v1, str_v2;
    bool positive;
    Serial3.println("v2");
    delay(5); ////데이터 송신후 수신 delay
    do {
      recv_str3 += (char)Serial3.read();
    } while (Serial3.available());
    //'=', '.' 인덱스 넘버 찾기
    int find_equal = recv_str3.indexOf('=', 0); //from 0
    int find_dot = recv_str3.indexOf('.', 0);
    int find_positive = recv_str3.indexOf('-', 0);
    //속도 값 추출
    for (int i = find_equal + 1; i < recv_str3.length(); i++) {str_v1 += recv_str3[i];}
    if (find_positive < 0) positive = true;
    else if (find_positive > 0) positive = false;
    double v1_1 = str_v1.toInt();
    double v1_2 = str_v1.substring(find_dot - 2, find_dot).toInt() / 100.;
    v2 = positive ? v1_1 + v1_2 : v1_1 - v1_2;
    velocity_Left = v2*radius*2*3.141592/60;
  }
