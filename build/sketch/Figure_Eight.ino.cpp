#include <Arduino.h>
#line 1 "c:\\Users\\manas\\Dev\\IMU-Arduino-Robot\\Figure_Eight\\Figure_Eight.ino"
// === Libraries and global variables === //

#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "SoftwareSerial.h"

#define PIN_SERVO           2       

#define PIN_DIRECTION_LEFT  4
#define PIN_DIRECTION_RIGHT 3
#define PIN_MOTOR_PWM_LEFT  6
#define PIN_MOTOR_PWM_RIGHT 5

#define PIN_SONIC_TRIG      7
#define PIN_SONIC_ECHO      8

#define PIN_BATTERY         A0

#define OBSTACLE_DISTANCE   30
#define OBSTACLE_DISTANCE_LOW 15

#define MAX_DISTANCE    300   //cm
#define SONIC_TIMEOUT   (MAX_DISTANCE*45)
#define SOUND_VELOCITY    340   //soundVelocity: 340m/s

Servo servo;
byte servoOffset = 0;
float speedOffset;//batteryVoltageCompensationToSpeed
const   u8 straightAngle  = 80; // Front
const   u8 rightAngle = 0;  // Right
const   u8 leftAngle  = 155;// Left

// point of reference: left side of one of the circles
const int firstExitAngle = 0; // was 32
const int secondExitAngle = 80; // was 155

double previousSonarReading = 0;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
SoftwareSerial bluetooth(10, 11);

#line 45 "c:\\Users\\manas\\Dev\\IMU-Arduino-Robot\\Figure_Eight\\Figure_Eight.ino"
int sgn(double num);
#line 49 "c:\\Users\\manas\\Dev\\IMU-Arduino-Robot\\Figure_Eight\\Figure_Eight.ino"
void displayCalStatus(void);
#line 76 "c:\\Users\\manas\\Dev\\IMU-Arduino-Robot\\Figure_Eight\\Figure_Eight.ino"
void debugPrintIMUData();
#line 92 "c:\\Users\\manas\\Dev\\IMU-Arduino-Robot\\Figure_Eight\\Figure_Eight.ino"
void followArcUltrasonicPID(int leftSpeed, int rightSpeed, float kP, double targetDistance, double exitAngle);
#line 176 "c:\\Users\\manas\\Dev\\IMU-Arduino-Robot\\Figure_Eight\\Figure_Eight.ino"
void driveForwardAnglePID(int speed, float kP, int wait, double angleTarget, bool (*endCondition)());
#line 212 "c:\\Users\\manas\\Dev\\IMU-Arduino-Robot\\Figure_Eight\\Figure_Eight.ino"
void forwardWhileScanning();
#line 225 "c:\\Users\\manas\\Dev\\IMU-Arduino-Robot\\Figure_Eight\\Figure_Eight.ino"
void forwardBrake();
#line 231 "c:\\Users\\manas\\Dev\\IMU-Arduino-Robot\\Figure_Eight\\Figure_Eight.ino"
void reverseBrake();
#line 237 "c:\\Users\\manas\\Dev\\IMU-Arduino-Robot\\Figure_Eight\\Figure_Eight.ino"
float getYaw();
#line 243 "c:\\Users\\manas\\Dev\\IMU-Arduino-Robot\\Figure_Eight\\Figure_Eight.ino"
void moveDrive(double left, double right);
#line 262 "c:\\Users\\manas\\Dev\\IMU-Arduino-Robot\\Figure_Eight\\Figure_Eight.ino"
float getSonar();
#line 284 "c:\\Users\\manas\\Dev\\IMU-Arduino-Robot\\Figure_Eight\\Figure_Eight.ino"
void motorRun(int speedl, int speedr);
#line 319 "c:\\Users\\manas\\Dev\\IMU-Arduino-Robot\\Figure_Eight\\Figure_Eight.ino"
void calculateVoltageCompensation();
#line 328 "c:\\Users\\manas\\Dev\\IMU-Arduino-Robot\\Figure_Eight\\Figure_Eight.ino"
float getBatteryVoltage();
#line 335 "c:\\Users\\manas\\Dev\\IMU-Arduino-Robot\\Figure_Eight\\Figure_Eight.ino"
double moveSonarAndScan(int target);
#line 350 "c:\\Users\\manas\\Dev\\IMU-Arduino-Robot\\Figure_Eight\\Figure_Eight.ino"
bool isWithinDistance();
#line 357 "c:\\Users\\manas\\Dev\\IMU-Arduino-Robot\\Figure_Eight\\Figure_Eight.ino"
void setup();
#line 390 "c:\\Users\\manas\\Dev\\IMU-Arduino-Robot\\Figure_Eight\\Figure_Eight.ino"
void loop();
#line 45 "c:\\Users\\manas\\Dev\\IMU-Arduino-Robot\\Figure_Eight\\Figure_Eight.ino"
int sgn(double num) {
  return (num != 0) ? abs(num) / num : 0;
}

void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
}

void debugPrintIMUData() {
  while (true) {
    sensors_event_t imuEvent;
    bno.getEvent(&imuEvent);
    
    Serial.print("X: ");
    Serial.print(imuEvent.orientation.x, 4);
    Serial.print("\tY: ");
    Serial.print(imuEvent.orientation.y, 4);
    Serial.print("\tZ: ");
    Serial.print(imuEvent.orientation.z, 4);
    Serial.println("");
    delay(100);
  }
}

void followArcUltrasonicPID(int leftSpeed, int rightSpeed, float kP, double targetDistance, double exitAngle) {
  double current = getYaw();
  current *= PI/180.0;
  current = atan2(sin(current), cos(current));
  current *= 180.0/PI;
  // Serial.print("\tangle: ");
  // Serial.print(current);
  double previousAngleError = exitAngle - current;
  previousAngleError *= PI / 180.0;
  previousAngleError = atan2(sin(previousAngleError), cos(previousAngleError));
  previousAngleError *= 180.0 / PI;

  auto currentServoAngle = servo.read();

  while (true) {

    float currentDistance = getSonar();
    //currentDistance = (currentDistance > 200) ? 200.0 : currentDistance;
    if ((currentDistance - previousSonarReading) >= 75) {
      currentDistance = previousSonarReading; // ignore spikes in sonar reading
    }

    float distanceError = targetDistance - currentDistance;
    double correctionPower = distanceError * kP;
    // if (distanceError > 0 && distanceError <= 5) {
    //   correctionPower -= 100; // fake integral term
    // }
    float leftPower = 0;
    float rightPower = 0;
    
    if (currentServoAngle < straightAngle) { // right-facing
      leftPower = leftSpeed - correctionPower;
      rightPower = rightSpeed + correctionPower;
      // Serial.print("right-facing, leftSpeed: ");
      // Serial.print(leftPower, 4);
      // Serial.print("\trightSpeed: ");
      // Serial.print(rightPower, 4);
      // Serial.print("\tcurrentDistance: ");
      // Serial.print(currentDistance, 4);
      //Serial.println("");
      
    } else { // currentServoAngle >= straightAngle, left-facing
      leftPower = leftSpeed + correctionPower;
      rightPower = rightSpeed - correctionPower;
    }

    motorRun(leftPower, rightPower);

    double tempExitAngle = exitAngle;
    tempExitAngle *= PI / 180.0;
    tempExitAngle = atan2(sin(tempExitAngle), cos(tempExitAngle));
    tempExitAngle *= 180.0 / PI;
    current = getYaw();
    current *= PI/180.0;
    current = atan2(sin(current), cos(current));
    current *= 180.0/PI;
    // Serial.print("\tangle: ");
    // Serial.print(current);
    double angleError = tempExitAngle - current;
    angleError *= PI / 180.0;
    angleError = atan2(sin(angleError), cos(angleError));
    angleError *= 180.0 / PI;
    // Serial.print("\tangleError: ");
    // Serial.print(angleError);
    // Serial.println("");

    // bluetooth.print("dist: ");
    // bluetooth.print(currentDistance);
    // bluetooth.print("\tang: ");
    // bluetooth.print(current);

    if (abs(angleError) <= 5) { // remove one of these as a test
      break;
    }
    if (sgn(angleError) != sgn(previousAngleError) && abs(angleError) <= 50) {
      break;
    }

    previousAngleError = angleError;

    delay(5);
  }
}

void driveForwardAnglePID(int speed, float kP, int wait, double angleTarget, bool (*endCondition)()) {
  auto startTime = millis();
  double startAngle = getYaw();
  double target = angleTarget;
  int runTime = 6000;

  while (true) {
    
    //double current = getYaw() - startAngle;
    double current = getYaw();
    double currentCorrected = atan2(sin(current), cos(current));
    //current = current * 180.0 / PI;
    double error = target - current;
    error *= PI / 180.0;
    error = atan2(sin(error), cos(error));
    error = error * 180.0 / PI;
  //  Serial.print("current: ");
  //  Serial.print(current * 180.0 / PI);
  //  Serial.print("\ttarget: ");
  //  Serial.print(target * 180.0 / PI);
  // //  Serial.print("\tcurrentCorrected: ");
  // //  Serial.print(currentCorrected);
  //  Serial.print("\terror: ");
  //  Serial.print(error);
  //   Serial.println("");
    error *= kP;
    
    motorRun(speed + error, speed - error); 
    
    if (endCondition() && (millis() - startTime) >= wait)
      break;

    delay(5);
  }
}

void forwardWhileScanning() {
  motorRun((150 + speedOffset), (150 + speedOffset));
  servo.write(straightAngle);
  while (true) {
    float straightDistance = moveSonarAndScan(straightAngle);
    if (straightDistance < OBSTACLE_DISTANCE) {
      break;
    }
  }
  motorRun(0, 0);
}

// TODO: use IMU acceleration data to brake instead
void forwardBrake() {
  motorRun(-(60 + speedOffset), -(60 + speedOffset));
  delay(150);
  motorRun(0, 0);
}

void reverseBrake() {
  motorRun(60 + speedOffset, 60 + speedOffset);
  delay(150);
  motorRun(0, 0);
}

float getYaw() {
  sensors_event_t event;
  bno.getEvent(&event);
  return event.orientation.x;
}

void moveDrive(double left, double right) {
  calculateVoltageCompensation();
  int leftPower = (left == 0) ? 0 : left + copysignf(speedOffset, left);
  int rightPower = (right == 0) ? 0 : right + copysignf(speedOffset, right);
  // Serial.print("Time: ");
  // Serial.print(millis());
  // Serial.print("\tLeft: ");
  // Serial.print(leftPower);
  // Serial.print("\tRight: ");
  // Serial.print(rightPower);
  // Serial.println("");

  motorRun(leftPower, rightPower);
}

/*!
 * \brief  Get the value of the distance
 * \return Value of the distance
 */
float getSonar() {
  unsigned long pingTime;
  float distance;
  digitalWrite(PIN_SONIC_TRIG, HIGH); // make trigPin output high for 10μs to trigger the ultrasonic,
  delayMicroseconds(10);
  digitalWrite(PIN_SONIC_TRIG, LOW);
  pingTime = pulseIn(PIN_SONIC_ECHO, HIGH, SONIC_TIMEOUT); // Wait for the ultrasonic return to  high and measure out the wait time
  if (pingTime != 0)
    distance = (float)pingTime * SOUND_VELOCITY / 2 / 10000; // calculate the distance based on the time
  else
    distance = MAX_DISTANCE;
  // if ((distance - previousSonarReading) >= 75) {
  //   distance = previousSonarReading; // ignore spikes in sonar reading
  // }
  //distance = (previousSonarReading * 0.5) + (distance * 0.5);
  previousSonarReading = distance;

  return distance;
}
  /*!
 * \brief  Move the selected right and left motorsyy
 */
void motorRun(int speedl, int speedr) {
  int dirL = 0, dirR = 0;
  if (speedl > 0) {
    dirL = 0;
  }
  else {
    dirL = 1;
    speedl = -speedl;
  }

  if (speedr > 0) {
    dirR = 1;
  }
  else {
    dirR = 0;
    speedr = -speedr;
  }

  if (speedl > 255) {
    speedl = 255;
  }
  if (speedr > 255) {
    speedr = 255;
  }

  digitalWrite(PIN_DIRECTION_LEFT, dirL);
  digitalWrite(PIN_DIRECTION_RIGHT, dirR);
  analogWrite(PIN_MOTOR_PWM_LEFT, speedl);
  analogWrite(PIN_MOTOR_PWM_RIGHT, speedr);
}


/*!
 * \brief  Calculate voltage compensation
 */
void calculateVoltageCompensation() {
  float voltageOffset = 8.4 - getBatteryVoltage();
  speedOffset = voltageOffset * 20;
}

/*!
 * \brief  Get the value of the battery voltage
 * \return Value of the battery voltage
 */
float getBatteryVoltage() {
  pinMode(PIN_BATTERY, INPUT);
  int batteryADC = analogRead(PIN_BATTERY);
  float batteryVoltage = batteryADC / 1023.0 * 5.0 * 4.0;
  return batteryVoltage;
}

double moveSonarAndScan(int target) {
  //int movementTime = 130; // 130 ms to move 90 degrees
  int current = servo.read();
  //movementTime *= (abs(angle - currentPos) / 90.0);
  int error = target - current;
  
  servo.write(target);
  while (abs(error) > 5) {
    delay(5);
  }
  int distance = getSonar();
  delayMicroseconds(2 * SONIC_TIMEOUT);
  return distance;
}

bool isWithinDistance() {
  float current = getSonar();
  return (current <= OBSTACLE_DISTANCE);
}

// === SETUP FUNCTION === //

void setup() {
  Serial.begin(115200);
  bluetooth.begin(9600);
  delay(10);
  pinMode(PIN_DIRECTION_LEFT, OUTPUT);
  pinMode(PIN_MOTOR_PWM_LEFT, OUTPUT);
  pinMode(PIN_DIRECTION_RIGHT, OUTPUT);
  pinMode(PIN_MOTOR_PWM_RIGHT, OUTPUT);

  pinMode(PIN_SONIC_TRIG, OUTPUT);// set trigPin to output mode
  pinMode(PIN_SONIC_ECHO, INPUT); // set echoPin to input mode
  servo.attach(PIN_SERVO);
  calculateVoltageCompensation();

  Serial.print("Battery voltage: ");
  Serial.println(getBatteryVoltage(), 4);
  bluetooth.println("Bluetooth Initialized!");
  
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("No BNO055 detected, Check wiring or I2C ADDR!");
    while(1);
  }
  
  delay(1000);
    
  bno.setExtCrystalUse(true);
}

//=== LOOP FUNCTION === //

void loop() {
  // servo.write(straightAngle);
  // driveForwardAnglePID(100, 7.0, &isWithinDistance);
  servo.write(leftAngle);
  delay(500);

  // while (true) {
  //   motorRun(255, 80);
  // }
  // driveForwardAnglePID(200, 7.0, 2000, 45, ([]() {
  //   return (getSonar() <= OBSTACLE_DISTANCE);
  // }));
  // while (true) {
  //   Serial.println(getSonar());
  //   delay(5);
  // }
  for (int i = 0; i <= 5; i++) { // figure 8 5 times then stop

    driveForwardAnglePID(255, 7.0, 250, firstExitAngle, ([]() {
      return (getSonar() <= OBSTACLE_DISTANCE);
    }));
    //followArcUltrasonicPID(100, 255, 3.5, 15, secondExitAngle);
    followArcUltrasonicPID(180, 255, 10.0, 28, secondExitAngle);
    motorRun(0, 0);
    servo.write(rightAngle);
    driveForwardAnglePID(255, 7.0, 250, secondExitAngle, ([]() {
      return (getSonar() <= OBSTACLE_DISTANCE);
    }));
    followArcUltrasonicPID(255, 180, 10.0, 28, firstExitAngle);
    motorRun(0, 0);
    servo.write(leftAngle);
  }
  motorRun(0, 0);
  delay(20000);
}

