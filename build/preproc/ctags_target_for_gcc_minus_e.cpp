# 1 "c:\\Users\\Manas\\Documents\\USC Classes\\AME 101\\Car Code\\Project 1\\Figure_Eight\\Figure_Eight.ino"
// === Libraries and global variables === //

# 4 "c:\\Users\\Manas\\Documents\\USC Classes\\AME 101\\Car Code\\Project 1\\Figure_Eight\\Figure_Eight.ino" 2
# 5 "c:\\Users\\Manas\\Documents\\USC Classes\\AME 101\\Car Code\\Project 1\\Figure_Eight\\Figure_Eight.ino" 2
# 6 "c:\\Users\\Manas\\Documents\\USC Classes\\AME 101\\Car Code\\Project 1\\Figure_Eight\\Figure_Eight.ino" 2
# 7 "c:\\Users\\Manas\\Documents\\USC Classes\\AME 101\\Car Code\\Project 1\\Figure_Eight\\Figure_Eight.ino" 2
# 8 "c:\\Users\\Manas\\Documents\\USC Classes\\AME 101\\Car Code\\Project 1\\Figure_Eight\\Figure_Eight.ino" 2
# 9 "c:\\Users\\Manas\\Documents\\USC Classes\\AME 101\\Car Code\\Project 1\\Figure_Eight\\Figure_Eight.ino" 2
# 29 "c:\\Users\\Manas\\Documents\\USC Classes\\AME 101\\Car Code\\Project 1\\Figure_Eight\\Figure_Eight.ino"
Servo servo;
byte servoOffset = 0;
float speedOffset;//batteryVoltageCompensationToSpeed
const u8 straightAngle = 80; // Front
const u8 rightAngle = 0; // Right
const u8 leftAngle = 155;// Left

// point of reference: left side of one of the circles
const int firstExitAngle = 0; // was 32
const int secondExitAngle = 78; // was 155

double previousSonarReading = 0;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
SoftwareSerial bluetooth(10, 11);

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
# 106 "c:\\Users\\Manas\\Documents\\USC Classes\\AME 101\\Car Code\\Project 1\\Figure_Eight\\Figure_Eight.ino"
  double current = getYaw();
  current *= 3.1415926535897932384626433832795/180.0;
  current = atan2(sin(current), cos(current));
  current *= 180.0/3.1415926535897932384626433832795;

  double previousAngleError = exitAngle - current;
  previousAngleError *= 3.1415926535897932384626433832795 / 180.0;
  previousAngleError = atan2(sin(previousAngleError), cos(previousAngleError));
  previousAngleError *= 180.0 / 3.1415926535897932384626433832795;

  auto currentServoAngle = servo.read();

  while (true) {

    float currentDistance = getSonar();

    if ((currentDistance - previousSonarReading) >= 75) {
      currentDistance = previousSonarReading; // ignore spikes in sonar reading
    }

    float distanceError = targetDistance - currentDistance;

    if (distanceError != 0) {
      distanceError = sqrt(15.0 * ((distanceError)>0?(distanceError):-(distanceError))) * (((distanceError)>0?(distanceError):-(distanceError)) / distanceError);
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
    tempExitAngle *= 3.1415926535897932384626433832795 / 180.0;
    tempExitAngle = atan2(sin(tempExitAngle), cos(tempExitAngle));
    tempExitAngle *= 180.0 / 3.1415926535897932384626433832795;
    current = getYaw();
    current *= 3.1415926535897932384626433832795/180.0;
    current = atan2(sin(current), cos(current));
    current *= 180.0/3.1415926535897932384626433832795;

    double angleError = tempExitAngle - current;
    angleError *= 3.1415926535897932384626433832795 / 180.0;
    angleError = atan2(sin(angleError), cos(angleError));
    angleError *= 180.0 / 3.1415926535897932384626433832795;

    // If the sign of the error changed, the robot has probably gone past the target so let's stop here
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
# 233 "c:\\Users\\Manas\\Documents\\USC Classes\\AME 101\\Car Code\\Project 1\\Figure_Eight\\Figure_Eight.ino" 3
                                          copysign /**< The alias for copysign().	*/
# 233 "c:\\Users\\Manas\\Documents\\USC Classes\\AME 101\\Car Code\\Project 1\\Figure_Eight\\Figure_Eight.ino"
                                                   (speedOffset, left);
  int rightPower = (right == 0) ? 0 : right + 
# 234 "c:\\Users\\Manas\\Documents\\USC Classes\\AME 101\\Car Code\\Project 1\\Figure_Eight\\Figure_Eight.ino" 3
                                             copysign /**< The alias for copysign().	*/
# 234 "c:\\Users\\Manas\\Documents\\USC Classes\\AME 101\\Car Code\\Project 1\\Figure_Eight\\Figure_Eight.ino"
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
# 250 "c:\\Users\\Manas\\Documents\\USC Classes\\AME 101\\Car Code\\Project 1\\Figure_Eight\\Figure_Eight.ino"
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
# 272 "c:\\Users\\Manas\\Documents\\USC Classes\\AME 101\\Car Code\\Project 1\\Figure_Eight\\Figure_Eight.ino"
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
# 307 "c:\\Users\\Manas\\Documents\\USC Classes\\AME 101\\Car Code\\Project 1\\Figure_Eight\\Figure_Eight.ino"
void calculateVoltageCompensation() {
  float voltageOffset = 8.4 - getBatteryVoltage();
  speedOffset = voltageOffset * 20;
}

/*!

 * \brief  Get the value of the battery voltage

 * \return Value of the battery voltage

 */
# 316 "c:\\Users\\Manas\\Documents\\USC Classes\\AME 101\\Car Code\\Project 1\\Figure_Eight\\Figure_Eight.ino"
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
  bluetooth.begin(9600);
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
      return (getSonar() <= 30);
    }));

    // follow around the circle until it reaches the right angle to transfer to the next circle
    followArcUltrasonicPID(180, 255, 10.0, 22, secondExitAngle);

    // briefly (as in, around 50 milliseconds) stop
    motorRun(0, 0);

    // angle the servo to the right in order to get ready to follow the next circle
    servo.write(rightAngle);

    // drive straight while correcting the robot's angle until it sees a circle
    driveForwardAnglePID(255, 7.0, 250, secondExitAngle, ([]() {
      return (getSonar() <= 30);
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
