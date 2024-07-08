/**
 * @file drone_controller.ino
 * @brief This code is designed to control a drone using a joystick, an MPU6050 IMU, and servos for a 2-axis gimbal.
 *
 * The code reads joystick inputs to control the drone's movement in various directions (Up, Down, Front, Back, Left, Right)
 * and sends these commands to the drone via UART. The MPU6050 IMU is used to get acceleration and gyroscope data, 
 * which is processed using a Kalman filter to maintain stable flight and gimbal control. The gimbal servos are controlled
 * to stabilize the camera based on the drone's orientation.
 *
 * The code uses the Scheduler library to run multiple tasks concurrently:
 * 1. Send_Data: Sends joystick and mode data to the drone via UART.
 * 2. Gimball: Controls the gimbal servos based on IMU data.
 * 3. Mode_selector: Changes the drone's mode based on a button press.
 *
 * The PID control algorithm is used to adjust the gimbal position for stabilization.
 *
 * @dependencies:
 * - Wire.h
 * - Scheduler.h
 * - Servo.h
 * - Adafruit_MPU6050.h
 * - Adafruit_Sensor.h
 * - PicoSoftwareSerial.h
 *
 * @hardware:
 * - MPU6050 IMU
 * - Servos for 2-axis gimbal
 * - Joystick for control input
 * - Button for mode selection
 * - UART for communication with the drone
 *
 * @author Emrecanbl
 * @version 1.0
 * @date 2024-07-08
 */

#include <Wire.h>
#include <Scheduler.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <PicoSoftwareSerial.h>
// Declare servos for gimbal control
Servo Servo_X;
Servo Servo_Y;

// Initialize MPU6050
Adafruit_MPU6050 mpu;

// Variables for PID control and Kalman filter
int InSum_x, InDiff_x, InOld_x;
int InSum_y, InDiff_y, InOld_y; 
float P_gain_x=1,I_gain_x=0.3,D_gain_x=0.01;
float P_gain_y=1,I_gain_y=0.3,D_gain_y=0.01;
float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=7*7;
float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=7*7;
float Kalman1DOutput[]={0,0};

// MPU6050 I2C address
const int MPU = 0x68;

// Variables to hold accelerometer and gyroscope data
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float AngleRoll, AnglePitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;

// Variables for joystick control
int Up, Down, CW_Turn, CCW_Turn, Front, Back, Left, Right;
int loopTimer;
int mode = 0;
String mode_str = "Waiting";

// Variables for button state
byte lastButtonState = LOW;
byte State = LOW;

// Define button pin
#define BUTTON_PIN 9

// Initialize UART
UART Serial2 (0, 1, 0, 0);

// Setup function
void setup() {
 Serial.begin(115200);
  Serial2.begin(115200);
  pinMode(BUTTON_PIN, INPUT);

  // Initialize MPU6050
  if (!mpu.begin()) {
    delay(10);
    while (1) {
      delay(10);
      Serial.print("MPU_Not_started");
      Serial2.println("MPU_Not_started");
    }
  }

  // Set MPU6050 ranges and bandwidth
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
  
  // Start Scheduler loops
  Scheduler.startLoop(Send_Data);
  Scheduler.startLoop(Gimball);
  Scheduler.startLoop(Mode_selector);
  
  // Attach servos
  Servo_X.attach(18);
  Servo_Y.attach(19);
  
  // Calculate IMU error
  calculate_IMU_error();
}
// Main loop function
void loop() {
  loopTimer = micros();
  JoystickRead();
  MPU_GetData();
  //Serial.println("Micros:");
  //Serial.print(micros()-loopTimer);
}
// Task to send data via UART
void Send_Data() {
  
  Serial2.print("Up");
  Serial2.print(Up);
  Serial2.print("/Down");
  Serial2.print(Down);
  Serial2.print("/CCW_Turn");
  Serial2.print(CCW_Turn);
  Serial2.print("/CW_Turn");
  Serial2.print(CW_Turn);
  Serial2.print("/Front");
  Serial2.print(Front);
  Serial2.print("/Back");
  Serial2.print(Back);
  Serial2.print("/Left");
  Serial2.print(Left);
  Serial2.print("/Right");
  Serial2.print(Right);
  Serial2.print("/Mode");
  Serial2.println(mode_str);
  Serial.print("/Up");
  Serial.print(Up);
  Serial.print("/Down");
  Serial.print(Down);
  Serial.print("/CCW_Turn");
  Serial.print(CCW_Turn);
  Serial.print("/CW_Turn");
  Serial.print(CW_Turn);
  Serial.print("/Front");
  Serial.print(Front);
  Serial.print("/Back");
  Serial.print(Back);
  Serial.print("/Left");
  Serial.print(Left);
  Serial.print("/Right");
  Serial.print(Right);
  Serial.print("/Mode");
  Serial.println(mode_str); 
  //calculate_IMU_error();
  
  delay(100);
}
// Task to control the gimbal
void Gimball() {
  int x_error = 0-KalmanAngleRoll;
  int Pos_y;
   if(KalmanAnglePitch<=0&&KalmanAnglePitch>=-1.5){
    Pos_y=0;
  }else{
     Pos_y = KalmanAnglePitch;
  }
  int y_error = Pos_y;
  //int Pos_x = PID_control_x(x_error);
  //int Pos_y = PID_control_y(y_error);
  Servo_Control(x_error,y_error);
}
// Task to select mode using a button
void Mode_selector(){
  byte buttonState = !digitalRead(BUTTON_PIN);
  if (buttonState != lastButtonState) {
    lastButtonState = buttonState;
    if (buttonState == LOW) {
      State = (State == HIGH) ? LOW : HIGH;
      digitalWrite(LED_BUILTIN, State);
      delay(250);
      if (mode <= 5) { mode++; }
      else { mode = 0; }
      Serial.println(mode);
      switch (mode) {
        case 1: mode_str = "Take_Off"; break;
        case 2: mode_str = "Manual_mode"; break;
        case 3: mode_str = "Face_Track"; break;
        case 4: mode_str = "Object_Track"; break;
        case 5: mode_str = "Flip"; break;
        case 6: mode_str = "Land"; break;
        default: mode_str = "Waiting"; break;
      }
    }
  }
}

// Funtion Definations 
// Function to control the servos
void Servo_Control(float Pos_x,float Pos_y){
  Servo_X.write(-Pos_x+90);            
  Servo_Y.write (int(90-(6*Pos_y)+30));
  delay(20);
}
// PID control functions for x and y axes
int PID_control_x(int In){
  int out = P_gain_x*In+I_gain_x*InSum_x+D_gain_x*InDiff_x;
  InSum_x += In;
  InDiff_x = In - InOld_x;
  InOld_x = In;
  return out;
}
int PID_control_y(int In){
  int out = P_gain_y*In+I_gain_y*InSum_y+D_gain_y*InDiff_y;
  InSum_y += In;
  InDiff_y = In - InOld_y;
  InOld_y = In;
  return out;
}
// Function to read joystick input
void JoystickRead(){
  int X_read = analogRead(A1);
  int Y_read = analogRead(A0);
  if(X_read>780){
    Up = map(X_read,780,1204, 100, 200);
    Down = 100;
    CCW_Turn = 100;
    CW_Turn = 100;
  }
  else if(X_read<750){
    Down = map(X_read,750,0, 100, 200);
    Up = 100;
    CCW_Turn = 100;
    CW_Turn = 100;
  }
  else{
    Up = 100;
    Down = 100;
  }
  if(Y_read>800){
    CW_Turn = map(Y_read,800,1024, 100, 200);
    CCW_Turn = 100;
    Up = 100;
    Down = 100;
  }
  else if(Y_read<750){
    CCW_Turn = map(Y_read,750,0, 100, 200);
    CW_Turn = 100;
    Up = 100;
    Down = 100;
  }
  else{
    CCW_Turn = 100;
    CW_Turn = 100;
  }
}
// Function to get data from MPU6050
void MPU_GetData(){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float x,y,z,RateRoll,RatePitch,RateYaw;
  x = a.acceleration.x; // X-axis value
  y = a.acceleration.y ;// Y-axis value
  z = a.acceleration.z ;// Z-axis value
  RateRoll=(float)g.gyro.x;
  RatePitch=(float)g.gyro.y;
  RateYaw=(float)g.gyro.z;
	float tanx,tany,tanz;
	tanx = x/(sqrt((z)*(z))+(y*y));
	tany = y/(sqrt((x)*(x))+(z*z));
	tanz = z/(sqrt((y)*(y))+(x*x));
	tanx = atan(tanx);
	tany = atan(tany);
	tanz = atan(tanz);
	AngleRoll = tanx*(1/(3.14/180.0));
	AnglePitch = tany*(1/(3.14/180.0));
	yaw = tanz*(1/(3.14/180.0));
  kalman_1d(KalmanAngleRoll,KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll=Kalman1DOutput[0];
  KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
  kalman_1d(KalmanAnglePitch,KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch=Kalman1DOutput[0];
  KalmanUncertaintyAnglePitch=Kalman1DOutput[1];
  /* Print out the values */
  /*Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");
  Serial.print("roll : ");
  Serial.println(roll);
  Serial.print("pitch : ");
  Serial.println(pitch);
  Serial.print("yaw : ");
  Serial.println(yaw);*/
  if(KalmanAnglePitch<-1){
    Front = map(KalmanAnglePitch,-1,-15, 100, 200);
    if(Front>=200){Front=200;}
    Back = 100;
  }
  else if(KalmanAnglePitch>1){
    Back = map(KalmanAnglePitch,1,15, 100, 200);
    if(Back>=200){Back=200;}
    Front = 100;
  }
  else{
    Front = 100;
    Back = 100;
  }
  if(KalmanAngleRoll>5){
    Left = map(KalmanAngleRoll,5,55, 100, 200);
    if(Left>=200){Left=200;}
    Right = 100;
  }
  else if(KalmanAngleRoll<-5){
    Right = map(KalmanAngleRoll,-5,-55, 100, 200);
    if(Right>=200){Right=200;}
    Left = 100;
  }
  else{
    Left = 100;
    Right = 100;
  }
  /*
  Serial.print("Front : ");
  Serial.println(Front);
  Serial.print("Back : ");
  Serial.println(Back);
  Serial.print("Left : ");
  Serial.println(Left);
  Serial.print("Right : ");
  Serial.println(Right);
  */
}
// Function to apply 1D Kalman filter
void kalman_1d(float KalmanState,float KalmanUncertainty, float KalmanInput,float KalmanMeasurement) {
KalmanState=KalmanState+0.0032*KalmanInput;
KalmanUncertainty=KalmanUncertainty + 0.0032 * 0.0032 * 8 * 8;
float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3);
KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
KalmanUncertainty=(1-KalmanGain)*KalmanUncertainty;
Kalman1DOutput[0]=KalmanState;
Kalman1DOutput[1]=KalmanUncertainty;
 }
 // Function to calculate IMU error
void calculate_IMU_error() {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  while (c < 2000) {
  AccX = a.acceleration.x ; // X-axis value
  AccY = a.acceleration.y ;// Y-axis value
  AccZ = a.acceleration.z ;// Z-axis value
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }
  //Divide the sum by 2000 to get the error value
  AccErrorX = AccErrorX / 2000;
  AccErrorY = AccErrorY / 2000;
  c = 0;
  // Read gyro values 2000 times
  while (c < 2000) {
    GyroX = g.gyro.x; 
    GyroY = g.gyro.y;
    GyroZ = g.gyro.z;
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX);
    GyroErrorY = GyroErrorY + (GyroY);
    GyroErrorZ = GyroErrorZ + (GyroZ);
    c++;
  }
  //Divide the sum by 2000 to get the error value
  GyroErrorX = GyroErrorX / 2000;
  GyroErrorY = GyroErrorY / 2000;
  GyroErrorZ = GyroErrorZ / 2000;
  // Print the error values on the Serial Monitor
  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
  Serial.print("GyroErrorZ: ");
  Serial.println(GyroErrorZ);
  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
}