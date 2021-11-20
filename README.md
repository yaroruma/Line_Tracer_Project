# Line Tracer Project
## Brief
로봇공학입문설계 과목 중간 텀프로젝트 미션 코드입니다. 모바일 로봇의 주행을 위한 아두이노 코드와 조종을 위한 안드로이드 애플리케이션으로 구성되어 있습니다. 앱인벤터를 통한 애플리케이션 개발과 모바일 로봇과의 블루투스 패킷 통신, 수동 조종 코드를 담당했습니다.

## Mission
컨트롤러를 이용해 수동으로 조종하거나 초음파 센서를 사용해 미로 구간을 빠져나온 후에 미로 출구에서 라인 트레이싱 시작. 라인 트레이싱 도중에 바닥의 패널의 색을 인식하여 정해진 미션을 수행.   
    ![mission](/mission.JPG)

## Mobile Robot
Board : Arduino Mega   
Motor : 12V DC encoder motor x2 (differential drive)   
Sensor : HC-SR04(ultrasonic sensor) x1, 5ch line_tracer IR sensor x1, TCS-34725(RGB sensor) x1   
Bluetooth : HC-06

## Slave code
requires - TimerOne Library, Adafruit_TCS34725 Library
1.  Making decision   
blutooth serial, TCS interrupt -> mode change
    ```
    void mode_change()
    void RGB_sensor()
    void manual()
    ```
2. Driving   
encoder interrupt, timer interrupt -> PID control   
blutooth serial + PID control -> manual driving
    ```
    void en_P1_chA_ISR()
    void en_P2_chA_ISR()
    void TimerISR()
    void PID(int L_motor_rpm,int R_motor_rpm,int L_vel, int R_vel)
    void Line_trace()
    void line_motor(int L,int R,int L_vel,int R_vel)
    void manual()
    ```
3. Monitering   
blutooth serial + sensor values -> application state moniter
    ```
    void Line_trace()
    void distance_sensor()
    void RGB_sensor()
    void Blue()
    ```
## Master code
MIT AppInventor2를 이용했습니다. Java 클래스 등이 자동생성되기 때문에 소스코드를 별도로 첨부하기 어려워서 블록코드 캡쳐 형태로 올립니다.
