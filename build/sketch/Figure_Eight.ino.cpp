#include <Arduino.h>
#line 1 "c:\\Users\\manas\\Dev\\IMU-Arduino-Robot\\Figure_Eight\\Figure_Eight.ino"
// === Libraries and global variables === //

#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

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

#define MAX_DISTANCE    200   //cm
#define SONIC_TIMEOUT   (MAX_DISTANCE*45)
#define SOUND_VELOCITY    340   //soundVelocity: 340m/s

Servo servo;
byte servoOffset = 0;
float speedOffset;//batteryVoltageCompensationToSpeed
const   u8 straightAngle  = 80; // Front
const   u8 rightAngle = 5;  // Right
const   u8 leftAngle  = 180;// Left

// point of reference: left side of one of the circles
const int firstExitAngle = 34.23;
const int secondExitAngle = 180 + firstExitAngle;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

#line 41 "c:\\Users\\manas\\Dev\\IMU-Arduino-Robot\\Figure_Eight\\Figure_Eight.ino"
void displayCalStatus(void);
#line 68 "c:\\Users\\manas\\Dev\\IMU-Arduino-Robot\\Figure_Eight\\Figure_Eight.ino"
void debugPrintIMUData();
#line 84 "c:\\Users\\manas\\Dev\\IMU-Arduino-Robot\\Figure_Eight\\Figure_Eight.ino"
void followArcUltrasonicPID(int leftSpeed, int rightSpeed, float kP, double targetDistance, double exitAngle);
#line 126 "c:\\Users\\manas\\Dev\\IMU-Arduino-Robot\\Figure_Eight\\Figure_Eight.ino"
void driveForwardAnglePID(int speed, float kP, bool (*endCondition)());
#line 160 "c:\\Users\\manas\\Dev\\IMU-Arduino-Robot\\Figure_Eight\\Figure_Eight.ino"
void forwardWhileScanning();
#line 173 "c:\\Users\\manas\\Dev\\IMU-Arduino-Robot\\Figure_Eight\\Figure_Eight.ino"
void forwardBrake();
#line 179 "c:\\Users\\manas\\Dev\\IMU-Arduino-Robot\\Figure_Eight\\Figure_Eight.ino"
void reverseBrake();
#line 185 "c:\\Users\\manas\\Dev\\IMU-Arduino-Robot\\Figure_Eight\\Figure_Eight.ino"
float getYaw();
#line 191 "c:\\Users\\manas\\Dev\\IMU-Arduino-Robot\\Figure_Eight\\Figure_Eight.ino"
void moveDrive(double left, double right);
#line 210 "c:\\Users\\manas\\Dev\\IMU-Arduino-Robot\\Figure_Eight\\Figure_Eight.ino"
float getSonar();
#line 226 "c:\\Users\\manas\\Dev\\IMU-Arduino-Robot\\Figure_Eight\\Figure_Eight.ino"
void motorRun(int speedl, int speedr);
#line 261 "c:\\Users\\manas\\Dev\\IMU-Arduino-Robot\\Figure_Eight\\Figure_Eight.ino"
void calculateVoltageCompensation();
#line 270 "c:\\Users\\manas\\Dev\\IMU-Arduino-Robot\\Figure_Eight\\Figure_Eight.ino"
float getBatteryVoltage();
#line 277 "c:\\Users\\manas\\Dev\\IMU-Arduino-Robot\\Figure_Eight\\Figure_Eight.ino"
double moveSonarAndScan(int target);
#line 292 "c:\\Users\\manas\\Dev\\IMU-Arduino-Robot\\Figure_Eight\\Figure_Eight.ino"
bool isWithinDistance();
#line 299 "c:\\Users\\manas\\Dev\\IMU-Arduino-Robot\\Figure_Eight\\Figure_Eight.ino"
void setup();
#line 330 "c:\\Users\\manas\\Dev\\IMU-Arduino-Robot\\Figure_Eight\\Figure_Eight.ino"
void loop();
#line 41 "c:\\Users\\manas\\Dev\\IMU-Arduino-Robot\\Figure_Eight\\Figure_Eight.ino"
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

  while (true) {

    float currentDistance = getSonar();
    // currentDistance = (currentDistance > 100) ? 70.0 : currentDistance;
    float distanceError = targetDistance - currentDistance;
    double correctionPower = distanceError * kP;
    float leftPower = 0;
    float rightPower = 0;
    
    auto currentServoAngle = servo.read();
    if (currentServoAngle < straightAngle) { // right-facing
      leftPower = leftSpeed - correctionPower;
      rightPower = rightSpeed + correctionPower;
      Serial.print("right-facing, leftSpeed: ");
      Serial.print(leftPower, 4);
      Serial.print("\trightSpeed: ");
      Serial.print(rightPower, 4);
      Serial.print("\tcurrentDistance: ");
      Serial.print(currentDistance, 4);
      Serial.println("");
    } else { // currentServoAngle >= straightAngle, left-facing
      leftPower = leftSpeed + correctionPower;
      rightPower = rightSpeed - correctionPower;
    }

    motorRun(leftPower, rightPower);

    double current = getYaw();
    double angleError = exitAngle - current;
    angleError *= PI / 180.0;
    angleError = atan2(sin(angleError), cos(angleError));
    angleError *= 180.0 / PI;
    if (abs(angleError) < 1) {
      break;
    }

    delay(10);
  }
}

void driveForwardAnglePID(int speed, float kP, bool (*endCondition)()) {
  auto startTime = millis();
  double startAngle = getYaw();
  double target = 0;
  int runTime = 6000;

  while (true) {
    
    double current = getYaw() - startAngle;
    current = current * PI / 180.0;
    double currentCorrected = atan2(sin(current), cos(current));
    //current = current * 180.0 / PI;
    double error = target - currentCorrected;
    error = error * 180.0 / PI;
//    Serial.print("startAngle: ");
//    Serial.print(startAngle);
//    Serial.print("\tcurrent: ");
//    Serial.print(current);
//    Serial.print("\tcurrentCorrected: ");
//    Serial.print(currentCorrected);
//    Serial.print("\terror: ");
//    Serial.print(error);
//    Serial.println("");
    error *= kP;
    
    motorRun(speed + error, speed - error); 
    
    if (endCondition())
      break;

    delay(10);
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
  Serial.begin(9600);
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
  servo.write(rightAngle);
  delay(200);
  for (int i = 0; i <= 5; i++) { // figure 8 5 times then stop
    followArcUltrasonicPID(200, 100, 1.8, 25, firstExitAngle);
    motorRun(0, 0);
    servo.write(leftAngle);
    driveForwardAnglePID(100, 7.0, ([]() {
      return (getSonar() <= OBSTACLE_DISTANCE);
    }));
    followArcUltrasonicPID(200, 100, 1.8, 25, secondExitAngle);
    motorRun(0, 0);
    servo.write(rightAngle);
    driveForwardAnglePID(100, 7.0, ([]() {
      return (getSonar() <= OBSTACLE_DISTANCE);
    }));
  }
  motorRun(0, 0);
  delay(20000);
}

