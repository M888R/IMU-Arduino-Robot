# 1 "c:\\Users\\manas\\Dev\\IMU-Arduino-Robot\\Figure_Eight\\Figure_Eight.ino"
// === Libraries and global variables === //

# 4 "c:\\Users\\manas\\Dev\\IMU-Arduino-Robot\\Figure_Eight\\Figure_Eight.ino" 2
# 5 "c:\\Users\\manas\\Dev\\IMU-Arduino-Robot\\Figure_Eight\\Figure_Eight.ino" 2
# 6 "c:\\Users\\manas\\Dev\\IMU-Arduino-Robot\\Figure_Eight\\Figure_Eight.ino" 2
# 7 "c:\\Users\\manas\\Dev\\IMU-Arduino-Robot\\Figure_Eight\\Figure_Eight.ino" 2
# 8 "c:\\Users\\manas\\Dev\\IMU-Arduino-Robot\\Figure_Eight\\Figure_Eight.ino" 2
# 28 "c:\\Users\\manas\\Dev\\IMU-Arduino-Robot\\Figure_Eight\\Figure_Eight.ino"
Servo servo;
byte servoOffset = 0;
float speedOffset;//batteryVoltageCompensationToSpeed
const u8 straightAngle = 80; // Front
const u8 rightAngle = 0; // Right
const u8 leftAngle = 180;// Left

// point of reference: left side of one of the circles
const int firstExitAngle = 0; // was 32
const int secondExitAngle = 70; // was 155

double previousSonarReading = 0;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

int sgn(double num) {
  return (num != 0) ? ((num)>0?(num):-(num)) / num : 0;
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
  Serial.print(system, 10);
  Serial.print(" G:");
  Serial.print(gyro, 10);
  Serial.print(" A:");
  Serial.print(accel, 10);
  Serial.print(" M:");
  Serial.print(mag, 10);
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
  current *= 3.1415926535897932384626433832795/180.0;
  current = atan2(sin(current), cos(current));
  current *= 180.0/3.1415926535897932384626433832795;
  // Serial.print("\tangle: ");
  // Serial.print(current);
  double previousAngleError = exitAngle - current;
  previousAngleError *= 3.1415926535897932384626433832795 / 180.0;
  previousAngleError = atan2(sin(previousAngleError), cos(previousAngleError));
  previousAngleError *= 180.0 / 3.1415926535897932384626433832795;

  while (true) {

    float currentDistance = getSonar();
    //currentDistance = (currentDistance > 200) ? 200.0 : currentDistance;
    if ((currentDistance - previousSonarReading) >= 100) {
      currentDistance = previousSonarReading; // ignore spikes in sonar reading
    }
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
      //Serial.println("");

    } else { // currentServoAngle >= straightAngle, left-facing
      leftPower = leftSpeed + correctionPower;
      rightPower = rightSpeed - correctionPower;
    }

    motorRun(leftPower, rightPower);

    double tempExitAngle = exitAngle;
    tempExitAngle *= 3.1415926535897932384626433832795 / 180.0;
    tempExitAngle = atan2(sin(tempExitAngle), cos(tempExitAngle));
    tempExitAngle *= 180.0 / 3.1415926535897932384626433832795;
    double current = getYaw();
    current *= 3.1415926535897932384626433832795/180.0;
    current = atan2(sin(current), cos(current));
    current *= 180.0/3.1415926535897932384626433832795;
    Serial.print("\tangle: ");
    Serial.print(current);
    double angleError = tempExitAngle - current;
    angleError *= 3.1415926535897932384626433832795 / 180.0;
    angleError = atan2(sin(angleError), cos(angleError));
    angleError *= 180.0 / 3.1415926535897932384626433832795;
    Serial.print("\tangleError: ");
    Serial.print(angleError);
    Serial.println("");

    if (((angleError)>0?(angleError):-(angleError)) <= 5) {
      break;
    }
    if (sgn(angleError) != sgn(previousAngleError) && ((angleError)>0?(angleError):-(angleError)) <= 50) {
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
    error *= 3.1415926535897932384626433832795 / 180.0;
    error = atan2(sin(error), cos(error));
    error = error * 180.0 / 3.1415926535897932384626433832795;
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
    if (straightDistance < 30) {
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
  int leftPower = (left == 0) ? 0 : left + 
# 233 "c:\\Users\\manas\\Dev\\IMU-Arduino-Robot\\Figure_Eight\\Figure_Eight.ino" 3
                                          copysign /**< The alias for copysign().	*/
# 233 "c:\\Users\\manas\\Dev\\IMU-Arduino-Robot\\Figure_Eight\\Figure_Eight.ino"
                                                   (speedOffset, left);
  int rightPower = (right == 0) ? 0 : right + 
# 234 "c:\\Users\\manas\\Dev\\IMU-Arduino-Robot\\Figure_Eight\\Figure_Eight.ino" 3
                                             copysign /**< The alias for copysign().	*/
# 234 "c:\\Users\\manas\\Dev\\IMU-Arduino-Robot\\Figure_Eight\\Figure_Eight.ino"
                                                      (speedOffset, right);
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
# 250 "c:\\Users\\manas\\Dev\\IMU-Arduino-Robot\\Figure_Eight\\Figure_Eight.ino"
float getSonar() {
  unsigned long pingTime;
  float distance;
  digitalWrite(7, 0x1); // make trigPin output high for 10μs to trigger the ultrasonic,
  delayMicroseconds(10);
  digitalWrite(7, 0x0);
  pingTime = pulseIn(8, 0x1, (300 /*cm*/*45)); // Wait for the ultrasonic return to  high and measure out the wait time
  if (pingTime != 0)
    distance = (float)pingTime * 340 /*soundVelocity: 340m/s*/ / 2 / 10000; // calculate the distance based on the time
  else
    distance = 300 /*cm*/;
  distance = (previousSonarReading * 0.5) + (distance * 0.5);
  previousSonarReading = distance;
  return distance;
}
  /*!

 * \brief  Move the selected right and left motorsyy

 */
# 268 "c:\\Users\\manas\\Dev\\IMU-Arduino-Robot\\Figure_Eight\\Figure_Eight.ino"
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

  digitalWrite(4, dirL);
  digitalWrite(3, dirR);
  analogWrite(6, speedl);
  analogWrite(5, speedr);
}


/*!

 * \brief  Calculate voltage compensation

 */
# 303 "c:\\Users\\manas\\Dev\\IMU-Arduino-Robot\\Figure_Eight\\Figure_Eight.ino"
void calculateVoltageCompensation() {
  float voltageOffset = 8.4 - getBatteryVoltage();
  speedOffset = voltageOffset * 20;
}

/*!

 * \brief  Get the value of the battery voltage

 * \return Value of the battery voltage

 */
# 312 "c:\\Users\\manas\\Dev\\IMU-Arduino-Robot\\Figure_Eight\\Figure_Eight.ino"
float getBatteryVoltage() {
  pinMode(A0, 0x0);
  int batteryADC = analogRead(A0);
  float batteryVoltage = batteryADC / 1023.0 * 5.0 * 4.0;
  return batteryVoltage;
}

double moveSonarAndScan(int target) {
  //int movementTime = 130; // 130 ms to move 90 degrees
  int current = servo.read();
  //movementTime *= (abs(angle - currentPos) / 90.0);
  int error = target - current;

  servo.write(target);
  while (((error)>0?(error):-(error)) > 5) {
    delay(5);
  }
  int distance = getSonar();
  delayMicroseconds(2 * (300 /*cm*/*45));
  return distance;
}

bool isWithinDistance() {
  float current = getSonar();
  return (current <= 30);
}

// === SETUP FUNCTION === //

void setup() {
  Serial.begin(115200);
  delay(10);
  pinMode(4, 0x1);
  pinMode(6, 0x1);
  pinMode(3, 0x1);
  pinMode(5, 0x1);

  pinMode(7, 0x1);// set trigPin to output mode
  pinMode(8, 0x0); // set echoPin to input mode
  servo.attach(2);
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
  servo.write(leftAngle);
  delay(500);
  // driveForwardAnglePID(200, 7.0, 2000, 45, ([]() {
  //   return (getSonar() <= OBSTACLE_DISTANCE);
  // }));
  // while (true) {
  //   //Serial.println(servo.read());
  //   servo.write(0);
  //   delay(5);
  // }
  for (int i = 0; i <= 5; i++) { // figure 8 5 times then stop

    driveForwardAnglePID(200, 7.0, 250, firstExitAngle, ([]() {
      return (getSonar() <= 30);
    }));
    followArcUltrasonicPID(100, 255, 3.5, 15, secondExitAngle);
    motorRun(0, 0);
    servo.write(rightAngle);
    driveForwardAnglePID(200, 7.0, 250, secondExitAngle, ([]() {
      return (getSonar() <= 30);
    }));
    followArcUltrasonicPID(255, 100, 1.8, 25, firstExitAngle);
    motorRun(0, 0);
    servo.write(leftAngle);
  }
  motorRun(0, 0);
  delay(20000);
}
