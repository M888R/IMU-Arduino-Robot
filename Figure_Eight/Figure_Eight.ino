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
const int secondExitAngle = 78; // was 155

double previousSonarReading = 0;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
SoftwareSerial bluetooth(10, 11);

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

  /** REASON FOR atan2:
   * Adafruit's IMU api reports angles with the range [0, 360)
   * When you add/subtract them to find the error which will later be used in PID,
   * You have issues with the error not being the fastest way to the target
   * i.e. if you have a target of 0, and are currently at an angle of 359, you'll do
   * target - current, or 0 - 359, and your error will be -359.
   * The most optimal error there should actually just be +1, because the robot should turn left by 1 degrees
   * instead of turning right by 359 degrees.
   * atan2(sin(error), cos(error)) is a quick and dirty way to constrain the angles to (-180, 180]
   * Doing so on the error means that the error will not only be never > 360, but it will also appropriately
   * change the robot's turning direction according to what's most efficient.
  */ 
  double current = getYaw();
  current *= PI/180.0;
  current = atan2(sin(current), cos(current));
  current *= 180.0/PI;

  double previousAngleError = exitAngle - current;
  previousAngleError *= PI / 180.0;
  previousAngleError = atan2(sin(previousAngleError), cos(previousAngleError));
  previousAngleError *= 180.0 / PI;

  auto currentServoAngle = servo.read();

  while (true) {

    float currentDistance = getSonar();

    if ((currentDistance - previousSonarReading) >= 75) {
      currentDistance = previousSonarReading; // ignore spikes in sonar reading
    }

    float distanceError = targetDistance - currentDistance;

    if (distanceError != 0) {
      distanceError = sqrt(15.0 * abs(distanceError)) * (abs(distanceError) / distanceError);
    }

    double correctionPower = distanceError * kP;
    
    float leftPower = 0;
    float rightPower = 0;
    
    if (currentServoAngle < straightAngle) { // right-facing
      leftPower = leftSpeed - correctionPower;
      rightPower = rightSpeed + correctionPower;
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

    double angleError = tempExitAngle - current;
    angleError *= PI / 180.0;
    angleError = atan2(sin(angleError), cos(angleError));
    angleError *= 180.0 / PI;

    // If the sign of the error changed, the robot has probably gone past the target so let's stop here
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

  servo.write(leftAngle);
  delay(500);

  for (int i = 0; i <= 5; i++) { // figure 8 6 times then stop

    // Drive forward while correcting the robot's angle until it sees an obstacle
    driveForwardAnglePID(255, 7.0, 250, firstExitAngle, ([]() {
      return (getSonar() <= OBSTACLE_DISTANCE);
    }));

    // follow around the circle until it reaches the right angle to transfer to the next circle
    followArcUltrasonicPID(180, 255, 10.0, 22, secondExitAngle);

    // briefly (as in, around 50 milliseconds) stop
    motorRun(0, 0);
    
    // angle the servo to the right in order to get ready to follow the next circle
    servo.write(rightAngle);

    // drive straight while correcting the robot's angle until it sees a circle
    driveForwardAnglePID(255, 7.0, 250, secondExitAngle, ([]() {
      return (getSonar() <= OBSTACLE_DISTANCE);
    }));

    // follow around the circle again until it reaches the right angle to transfer
    followArcUltrasonicPID(255, 180, 10.0, 22, firstExitAngle);

    // briefly stop again
    motorRun(0, 0);

    // turn the servo left in order to make sure you see the circle
    servo.write(leftAngle);

    // repeat
  }
  motorRun(0, 0);
  delay(20000);
}
