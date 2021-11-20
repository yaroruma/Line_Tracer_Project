#include <TimerOne.h> //타이머 라이브러리 호출
#include <Adafruit_TCS34725.h>
Adafruit_TCS34725 tcs= Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
#define servo_p 5
#define EA_CHA_P1 19
#define EA_CHA_P2 3
#define RGB_IN_A 20
#define RGB_IN_B 21  //엔코더 핀 설정
#define trigPin 53
#define echoPin 51
#define DIR_P1 7
#define PWM_P1 6
#define DIR_P2 4
#define PWM_P2 5  //모터 드라이브 설정
#define IR1 A12
#define IR2 A13
#define IR3 A14
#define IR4 A11
#define IR5 A10
#define wheel_r 6.7 //바퀴 지름cm
#define Pi  3.14  //원주율
#define robot_Leng 19//로봇폭
#define TX1 18
#define RX1 19
#define R_OUT A1
#define G_OUT A3
#define B_OUT A2
#define Left_turn 0
#define Right_turn 1
float en_P1_Pos;  
float en_P2_Pos; //좌우측 엔코더 변수 설정
float Len=0;  //현재까지 간 거리 합 변수 설정
int rpm_pre;  //이전 rpm변수
int rpm=0;  //좌측 바퀴rpm
int rpm2=0; //우측 바퀴rpm
//P control
double Kp = 20; //좌측 PID P값 상수
double Kp2 = 40;//우측 PID P값 상수
float P = 0;  //좌측 P값
float P2 = 0; //우측 P값
double Ki = 5;  //우측 PID I값 상수
double Ki2 =5;  //우측 PID I값 상수
float I = 0;  //좌측 I 값
float I2 = 0; //우측 I 값
double Kd = 5;  //우측 PID D값 상수
double Kd2 = 5; //좌측 PID D값 상수
float D = 0;  //좌측 D값
float D2 = 0; //우측 D값
double error_previous=0;  //좌측 이전 에러 값
int TargetRPM_L ; //목표 rpm
int TargetRPM_R;
double error = 0; //좌측 현재 에러 값
double error2 = 0;  //우측 현재 에러 값
double error_previous2 = 0; //우측 이전 에러 값
double PI_A;  //좌측 모터 속도
double PI_B;  //우측 모터 속도
long speen = (robot_Leng/2)*Pi/2; //회전해야 하는 거리
long gocm= 0; //전진해야 하는 거리
long sqcm=30; //현재 이동해야 하는 거리
float T_dt;
int F_distance;
float R,G,B;
int IR_L,IR_R,IR_C1,IR_C2,IR_m;
int Line_cross;
int dist_count;
int R_b,G_b,B_b;
int R_c,G_c,B_c;
int L_sp=100,R_sp=100;
int L_turn_c;
int cross_c=0;
int mode=0;
int turn_count=0;
int RGB_ST;
int RGB_C=0;
float RGB_time;
boolean Is_manual=1;
byte bluetooth = 0;
boolean  Emg=0;
int BL_RPM=0;
int BL_L_Dir=0;
int BL_R_Dir=0;
String state= "";

void setup() 
{
  pinMode(EA_CHA_P1,INPUT);
  pinMode(EA_CHA_P2,INPUT);
  pinMode(DIR_P1, OUTPUT);
  pinMode(PWM_P1, OUTPUT);
  pinMode(DIR_P2, OUTPUT);
  pinMode(PWM_P2, OUTPUT); 
  attachInterrupt(digitalPinToInterrupt(EA_CHA_P1),en_P1_chA_ISR,CHANGE);
  attachInterrupt(digitalPinToInterrupt(EA_CHA_P2),en_P2_chA_ISR,CHANGE); 
  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin, INPUT);
  pinMode(IR1, INPUT);      
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);      
  pinMode(IR4, INPUT);      
  pinMode(IR5, INPUT);    
  pinMode(R_OUT,OUTPUT);
  pinMode(G_OUT,OUTPUT);
  pinMode(B_OUT,OUTPUT);
  Timer1.initialize(10000);
  Timer1.attachInterrupt(TimerISR); 
  Serial3.begin(115200);//통신 주기 설정
  Serial.begin(115200);
  tcs.begin();
}
void loop() {
 while(Emg==0)
  {
   manual();
  }
  if(mode==0)
  {
    mode_change();
    mode++;
  }
  if(mode==1)
  {
    auto_ctrl_L();

  }
  else if(mode==2)
  {
    auto_parking();
    }

}
void mode_change()
{
  IR_L = analogRead(IR2);
  IR_R = analogRead(IR4);
  IR_C1 = analogRead(IR1);
  IR_C2 = analogRead(IR5);
  IR_m = analogRead(IR3);
  while((IR_L>300)&&(IR_R>300))
  {
    PID(10,10,1,1);
    IR_L = analogRead(IR2);
    IR_R = analogRead(IR4);
    }
   line_motor(0,0,1,1);
   delay(3000);
   line_motor(200,200,1,1);
   delay(500);
  }
void auto_parking()
{
  if(dist_count==0)
  {
    PID_go(10,1);
    dist_count++;
    }
  if(dist_count==1)
  {
    if(R_c>0)
    {
     RGB_ST=1;
     }
   if(G_c>0)
   {
      RGB_ST=10;
    }
   if(B_c>0)
    {
     RGB_ST=100;
    } 
    switch(RGB_ST)
    {
      case 1:
     
        PID_turn(speen,Left_turn);
        PID_go(50,1);
        PID_turn(speen,Right_turn);
        RGB_time=millis();
      while(dist_count==1)
     {
       distance_sensor();
      if(F_distance<10)
      {
        PID(20,20,0,0);
       RGB_time=millis();
        }
     else if(F_distance>15)
      {
      PID(20,20,1,1);
      RGB_time=millis();
      }
     else
     {
      line_motor(0,0,1,1);
      RGB_time=RGB_time;
      }
      if((millis()-RGB_time)>1000)
      {
        dist_count++;
        }
  }
        break;
       case 10:
        RGB_time=millis();
      while(dist_count==1)
      {
        distance_sensor();
        if(F_distance<10)
      {
         PID(20,20,0,0);
         RGB_time=millis();
      }
       else if(F_distance>15)
     {
          PID(20,20,1,1);
         RGB_time=millis();
       }
       else
     {
         line_motor(0,0,1,1);
         RGB_time=RGB_time;
        }
      if((millis()-RGB_time)>1000)
      {
        dist_count++;
        }
       }
        break;
        
       case 100:
       
        PID_turn(speen,Right_turn);
        PID_go(50,1);
        PID_turn(speen,Left_turn);
       RGB_time=millis();
      while(dist_count==1)
     {
    distance_sensor();
    if(F_distance<10)
    {
      PID(20,20,0,0);
      RGB_time=millis();
      }
    else if(F_distance>15)
    {
      PID(20,20,1,1);
      RGB_time=millis();
      }
     else
     {
      line_motor(0,0,1,1);
      RGB_time=RGB_time;
      }
      if((millis()-RGB_time)>1000)
      {
        dist_count++;
        }
  }
        break;
        
      }
      line_motor(0,0,1,1);
      mode++;
    }
   }
void PID_go(int Pid_dt,int go_vel)
{
  Len=0;
  while((Pid_dt-Len)>0)
  {
    PID(20,20,go_vel,go_vel);
    }
    line_motor(100,100,go_vel,go_vel);
    delay(150);
    line_motor(0,0,1,1);
    delay(500);
  }
void PID_turn(int Pid_dt,int turn_vel)
{
  Len=0;
  while((Pid_dt-Len)>0)
  {
    PID(20,20,turn_vel,(!turn_vel));
    }
    line_motor(100,100,(!turn_vel),turn_vel);
    delay(150);
    line_motor(0,0,1,1);
    delay(500);
  }
void auto_ctrl_L()
{
  RGB_Check();
    if((R_c>0)&&(RGB_C==0))
    {
     RGB_ST=1;
     }
   if((G_c>0)&&(RGB_C==0))
   {
      RGB_ST=10;
    }
   if((B_c>0)&&(RGB_C==0))
    {
     RGB_ST=100;
    } 
   switch(RGB_ST)
    {
      case 1:
      PID_turn((8*speen+3),Left_turn);
      RGB_time=millis();
      line_motor(200,200,1,1);
      delay(100);
      while((millis()-RGB_time)<5000)
      {
        Line_trace();
        
        line_motor(L_sp,R_sp,1,1);
        if((millis()-RGB_time)>5000)
        {
          break;
          }
        }
        
      R_c=0;
      G_c=0;
      B_c=0;
      RGB_ST=0;
      RGB_C++;
      break;
     case 10:
      PID_turn(4*speen,Left_turn);
      line_motor(200,200,1,1);
      delay(100);
      RGB_time=millis();
      while((millis()-RGB_time)<4000)
      {
        Line_trace();
        line_motor(L_sp,R_sp,1,1);
        if((millis()-RGB_time)>4000)
        {
          break;
          }
        }
      R_c=0;
      G_c=0;
      B_c=0;
      RGB_ST=0;
      RGB_C++;
     break;
     case 100:
      R_c=0;
      G_c=0;
      B_c=0;
      RGB_ST=0;
      RGB_C++;
      break;
  }
  Serial.print("R : ");Serial.print(R);
  Serial.print(" G : ");Serial.print(G);
  Serial.print(" B : ");Serial.println(B);
  Line_trace();
  line_motor(L_sp,R_sp,1,1);
  }
void RGB_Check()
{
  for(int i=0;i<3;i++)
  {
    RGB_sensor();
    }
  }
void Line_trace()
{
  Len=0;
  IR_L = analogRead(IR2);
  IR_R = analogRead(IR4);
  IR_C1 = analogRead(IR1);
  IR_C2 = analogRead(IR5);
  IR_m = analogRead(IR3);
  if((IR_R<300)&&(IR_L<300))
  {
    switch (cross_c)
    {
      case 0:
        digitalWrite(DIR_P1, 1);
        analogWrite(PWM_P1, 200);
        digitalWrite(DIR_P2, 1);
        analogWrite(PWM_P2, 200);
        delay(700);
        R=0;G=0,B=0;
        cross_c++;
        break;
      case 1:
        digitalWrite(DIR_P1, 1);
        analogWrite(PWM_P1,0);
        digitalWrite(DIR_P2, 1);
        analogWrite(PWM_P2, 0);
        delay(3000);
        mode++;
        break;
  }
  }
  //L_sp = ((IR_L<300)||(IR_C1<300))?0:25;
  //R_sp = ((IR_R<300)||(IR_C2<300))?0:25;
  if(((L_sp==0)||(R_sp==0))&&(IR_m>300))
  {
    line_motor(L_sp,R_sp,1,1);
    if(L_sp==0)
      {
        if(IR_C2<300)
        {
          L_turn_c=L_sp;
          L_sp=R_sp;
          R_sp=L_turn_c;
          line_motor(200,100,0,0);
          delay(800);
        }
       }
      else
      {
         if(IR_C1<300)
        {
          L_turn_c=L_sp;
          L_sp=R_sp;
          R_sp=L_turn_c;
          line_motor(100,200,0,0);
          delay(800);
        }
        }

    }
  else
  {
    L_sp = ((IR_L<300)&&(IR_R>300))?0:80;
    R_sp = ((IR_R<300)&&(IR_L>300))?0:80;
    L_sp = (IR_C1<300)?0:L_sp;
    R_sp = (IR_C1<300)?150:R_sp;
    R_sp = (IR_C2<300)?0:R_sp;
    L_sp = (IR_C2<300)?150:L_sp;
    L_sp = (IR_m<300)?80:L_sp;
    R_sp = (IR_m<300)?80:R_sp;
  }
  
}
void line_motor(int L,int R,int L_vel,int R_vel)
{
  digitalWrite(DIR_P1, L_vel);
  analogWrite(PWM_P1,L);
  digitalWrite(DIR_P2, R_vel);
  analogWrite(PWM_P2, R);
  }
void distance_sensor()
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  T_dt= pulseIn(echoPin, HIGH); 
  F_distance= T_dt* 17 / 1000; 
  }
void RGB_sensor()
{
  uint16_t r_raw, g_raw, b_raw, sum;
  delay(60);
  tcs.setInterrupt(false); 
  tcs.getRawData(&r_raw, &g_raw, &b_raw, &sum);
  tcs.setInterrupt(true); 
  R = r_raw; R /= sum;
  G = g_raw; G /= sum;   
  B = b_raw; B /= sum; 
  R *= 256; G *= 256; B *= 256;
  if((R>=120)&&(G>=120)&&(B>=100)||(R<=120)&&(G<=110)&&(B<=100))
  {
    R=0; G=0; B=0;
    }
  if(R>G)
  {
    if(R>B)
      {
        G=0;  B=0;
        }
     else
     {
        R=0;  G=0;
      }
    }
  else
  {
    if(R>B)
      {
        R=0;  B=0;
        }
     else
      {
        if(B>G)
          {
            G=0;  R=0;
            }
         else
          {
            B=0; R=0;
            }   
       }
    }
   if((R==0)&&(G==0)&&(B==0))
   {
      R=R_b; B=B_b; G=G_b;
    }
  if(R!=R_b)
   {
    R_c++;  B_c=0;  G_c=0;
    }
     if(G!=G_b)
   {
    G_c++;  B_c=0;  R_c=0;
    }
     if(B!=B_b)
   {
    B_c++;  R_c=0;  G_c=0;
    }
   R_b=R ; B_b=B; G_b=G;
   analogWrite(R_OUT,R+30);
  analogWrite(G_OUT,G+30);
  analogWrite(B_OUT,B+30);
      }
void PID(int L_motor_rpm,int R_motor_rpm,int L_vel, int R_vel)
{
  TargetRPM_L = L_motor_rpm;
  TargetRPM_R = R_motor_rpm;
  P = error*Kp;
  I += Ki*(error + error_previous)/2*0.001;
  D = Kd*(error - error_previous)/0.001;
 //좌측 바퀴 PID제어
  P2 = error2*Kp2;
  I2 += Ki2*(error2 + error_previous2)/2* 0.001;
  D2 = Kd2*(error2 - error_previous2)/0.001;
  //우측 바퀴 PID제어
  error_previous = error;
  error_previous2 = error2; // 좌,우측 바퀴의 에러값을 이전 에러 값으로 저장
  PI_A = constrain(P+I+D,0,255);
  PI_B = constrain(P2+I2+D2,0,255); // 좌,우측 바퀴의 PWM값 설정
  
  digitalWrite(DIR_P1, L_vel);
  analogWrite(PWM_P1, PI_A);
  digitalWrite(DIR_P2, R_vel);
  analogWrite(PWM_P2, PI_B);  //좌, 우측 바퀴 구동
 Serial.print(rpm);Serial.print("   ");
 Serial.println(rpm2);
  }
void TimerISR()
{
  rpm = abs(en_P1_Pos / 1 * 6000 * 1 / 22 / 90/2); //좌측rpm 측정
  Len += wheel_r*Pi*(rpm2 + rpm_pre)*0.01/60; //현재 이동거리 측정
  rpm_pre = rpm2; //우측 바퀴 rpm값을 이전 rpm으로 저장
  rpm2 = abs(en_P2_Pos / 1 * 6000 * 1 / 22 / 90/2);  //우측rpm측정
  error=TargetRPM_L - rpm;
  error2=TargetRPM_R - rpm2;  //에러값 측정
  en_P1_Pos = 0;
  en_P2_Pos = 0;  //엔코더 값 초기화
  Blue();
  state = "";
}
void en_P1_chA_ISR()  //좌측 바퀴 엔코더 값 측정 확인 함수
{
 if(digitalRead(EA_CHA_P1)==HIGH)
 {
  en_P1_Pos = en_P1_Pos + 1;
  }
 else
 {
  en_P1_Pos = en_P1_Pos + 1;
  }
}

void en_P2_chA_ISR()  //우측 바퀴 엔코더 값 측정 함수
{
 
 if(digitalRead(EA_CHA_P2)==HIGH)
 {
  en_P2_Pos = en_P2_Pos + 1;
  }
 else
 {
  en_P2_Pos = en_P2_Pos + 1;
  }
}
void Blue(){
   if(Serial3.available())
    {
  state +="/";
  state.concat((int)(IR_C1>300));
  state +=".";
  state.concat((int)(IR_L>300));
  state +=".";
  state.concat((int)(IR_m>300));
  state +=".";
  state.concat((int)(IR_R>300));
  state +=".";
  state.concat((int)(IR_C2>300));
  state +=".";
  state.concat((int)R);
  state +=".";
  state.concat((int)G);
  state +=".";
  state.concat((int)B);
  state +=".";
  state.concat((int)(F_distance>30?30:F_distance));
  state +=".";
  Serial3.print(state);
  Serial.print(state);
    }
}
void manual() {
    if(Serial3.available())
    {
      bluetooth = Serial3.read();
  if(bluetooth == 255)
    Emg=1;
  else if(bluetooth == 204){
    Emg=0;
    Serial.write(204);
  }
  else{
    BL_RPM = bluetooth/4;
    BL_L_Dir = bluetooth%4/2;
    BL_R_Dir = bluetooth%2;
  }
  if(BL_RPM==0){
    digitalWrite(DIR_P1, 0);
  analogWrite(PWM_P1, 0);
  digitalWrite(DIR_P2, 0);
  analogWrite(PWM_P2, 0);
  }
  else
  {
    digitalWrite(DIR_P1, BL_L_Dir);
  analogWrite(PWM_P1, (BL_RPM*50));
  digitalWrite(DIR_P2, BL_R_Dir);
  analogWrite(PWM_P2, (BL_RPM*50));
  }
    }
    
    Serial.print(BL_RPM);Serial.print(" ");
  Serial.print(BL_L_Dir);Serial.print(" ");
  Serial.println(BL_R_Dir);

}
