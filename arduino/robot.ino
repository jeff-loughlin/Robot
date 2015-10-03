#include <PID_v1.h>
#include <Wire.h>
#include <HMC5883L.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
//#include <Adafruit_L3GD20.h>
#include <L3G4200D.h>
#include <Servo.h>

void readCommand(char *command);
void SteerToHeading();
void WallFollower();
void DetectObstacles();
int getDistance();
float getcompasscourse();
void calibrateMagSensor();
void stopMotor(int motor);

void SetMotorSpeed(int motor, int speed);
void getRawMagData(int *x, int *y, int *z); 
void getRawAccelData(float *x, float *y, float *z); 
void getRawGyroData(int *x, int *y, int *z, int *gdt); 

HMC5883L compass;
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
//Adafruit_L3GD20 gyro;
L3G4200D gyro;


// PID controller variables
double SetHeading, Input, Output;
PID headingPID(&Input, &Output, &SetHeading,5,.1,.5, DIRECT);

double SetDistance, wallPIDInput, wallPIDOutput;
PID wallFollowerPID(&wallPIDInput, &wallPIDOutput, &SetDistance, 5, .1, .5, DIRECT);

// Servos
Servo servo1;
Servo servo2;	// Pan Servo
Servo servo3;	// Tilt servo

#define RIGHT_MOTOR 0
#define LEFT_MOTOR 1
#define DIR_FORWARD 0
#define DIR_REVERSE 1

#define FULL_SPEED 128

int ENA=5;    //connected to Arduino's port 5(output pwm)
int IN1=2;    //connected to Arduino's port 2
int IN2=3;    //connected to Arduino's port 3
int ENB=6;    //connected to Arduino's port 6(output pwm)
int IN3=4;    //connected to Arduino's port 4
int IN4=7;    //connected to Arduino's port 7

int servo1_pin = 10;
int servo2_pin = 11;
int servo3_pin = 9;   // Connected to pin 9 (PWM) 

#define DIST_SENSOR1_TRIGGER_PIN A0
#define DIST_SENSOR1_ECHO_PIN A1
#define DIST_SENSOR2_TRIGGER_PIN A2
#define DIST_SENSOR2_ECHO_PIN A3

class Kalman
{
private:
       double q; //process noise covariance
       double r; //measurement noise covariance
       double x; //value
       double p; //estimation error covariance
       double k; //kalman gain

public:
       Kalman(double _q, double _r, double _p, double _intial_value);
       void update(double measurement);
       double GetValue() {return x;}
};

Kalman kalmanFilter(0.125, 4, 1, 0);

Kalman xFilter(0.125, 4, 1, 0);
Kalman yFilter(0.125, 4, 1, 0);
Kalman zFilter(0.125, 4, 1, 0);

// Compass calibration offsets
int xMin = 9999;
int xMax = -9999;
int yMin = 9999;
int yMax = -9999;
int zMin = 9999;
int zMax = -9999;


class ControlMode
{
public:
    bool active;
    int leftMotorPower;
    int rightMotorPower;
    
    ControlMode() {active = false; leftMotorPower = 0; rightMotorPower = 0; };
};


void GetDistanceReadings(int *distance1, int *distance2);


void setup()
{
    pinMode(ENA,OUTPUT);
    pinMode(ENB,OUTPUT);
    pinMode(IN1,OUTPUT);
    pinMode(IN2,OUTPUT);
    pinMode(IN3,OUTPUT);
    pinMode(IN4,OUTPUT);

    Serial.begin(115200);
    accel.begin();
     
    // Both stopped
    stopMotor(LEFT_MOTOR);
    stopMotor(RIGHT_MOTOR);
    digitalWrite(ENA,LOW);
    digitalWrite(ENB,LOW);

    // Set up the compass
    Wire.begin(); // Start the I2C interface.
    compass = HMC5883L(); // Construct a new HMC5883 compass.
    
    compass.SetScale(1.3); // Set the scale of the compass.
    compass.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous

//    gyro.begin(gyro.L3DS20_RANGE_250DPS);
    gyro.enableDefault();

    delay(2000);

    pinMode(13, OUTPUT);
//    calibrate();

    SetMotorSpeed(LEFT_MOTOR, 0);
    SetMotorSpeed(RIGHT_MOTOR, 0);
    
    SetHeading = 65.0;
    float actualHeading = 0; //getcompasscourse();
//     if (SetHeading - actualHeading > 180)
//         actualHeading += 360;
//     else if (SetHeading - actualHeading < -180)
//         actualHeading -= 360;

    Input = actualHeading;
    headingPID.SetOutputLimits(-FULL_SPEED + 2, FULL_SPEED-2);
    headingPID.SetMode(AUTOMATIC);
    
    SetDistance = 40.0;
    wallFollowerPID.SetOutputLimits(-FULL_SPEED + 2, FULL_SPEED-2);
    wallFollowerPID.SetMode(AUTOMATIC);
    
    servo1.attach(servo1_pin);
    servo2.attach(servo2_pin);
    servo3.attach(servo3_pin);

    // Set up the distance sensor pins
    pinMode(DIST_SENSOR1_TRIGGER_PIN, OUTPUT);
    pinMode(DIST_SENSOR1_ECHO_PIN, INPUT);
    pinMode(DIST_SENSOR2_TRIGGER_PIN, OUTPUT);
    pinMode(DIST_SENSOR2_ECHO_PIN, INPUT);
}


ControlMode steerToHeadingControl;
ControlMode wallFollowerControl;
ControlMode detectObstaclesControl;

char msg[32];
int idx = 0;
unsigned long lastTime = 0;
unsigned long calibrationTime = 0;
int switch1 = 0;


#define GPSMAX 5
int gpsCnt = 0;
long latArray[] = {39972139, 39972123, 39972114, 39972098, 39972082};
long longArray[] = {-75736430, -75736312, -75736280, -75736162, -75736108};

long beaconTime = 0;

void loop()
{
    // If we haven't received a LED command in the last 1.5 seconds, something has gone horribly wrong.
    // Shut down the motors
    if (millis() - beaconTime > 1500)
    {
        SetMotorSpeed(LEFT_MOTOR, 0);
        SetMotorSpeed(RIGHT_MOTOR, 0);
    }

    if (Serial.available() > 0)
    {
        char ch = Serial.read();
        if (ch != '\r' && ch != '\n')
        {
            msg[idx++] = ch;
//            delay(10);
            return;
        }
        msg[idx] = 0;

        char msgType[8];
        int c = 0;
        while (msg[c] != ':' && msg[c] != 0)
        {
             msgType[c] = msg[c];
             c++;
        }
        msgType[c] = '\0';
    
        idx = 0;
        char val[32];
        c++;
        while (msg[c] != '\r' && msg[c] != '\n' && msg[c] != 0x00)
        {
            val[idx] = msg[c];
            c++;
            idx++;
        }
        val[idx] = '\0';
        idx = 0;
    
        if (!strcmp(msgType, "LM"))
        {
            // Set Left Motor power
            int speed = strtol(val, 0, 10);
            SetMotorSpeed(LEFT_MOTOR, speed);
        }
        else if (!strcmp(msgType, "RM"))
        {
            // Set Right Motor power
            int speed = strtol(val, 0, 10);
    	    SetMotorSpeed(RIGHT_MOTOR, speed);
        }
        else if (!strcmp(msgType, "LED"))
        {
            int level = strtol(val, 0, 10);
            digitalWrite(13, level);

            // Reset the beacon timer.  If we don't get a LED message within 2 seconds, we'll shut everything down
            beaconTime = millis();
        }
        else if (!strcmp(msgType, "S3"))
        {
            // Set Servo 3 angle
            int angle = strtol(val, 0, 10);
            servo3.write(angle);
        }
        else if (!strcmp(msgType, "S2"))
        {
            // Set Servo 2 angle
            int angle = strtol(val, 0, 10);
            servo2.write(angle);
        }
        else if (!strcmp(msgType, "CAL"))
        {
            calibrateMagSensor();
            // TEMP - TESTING
//            delay(1000);
//            xMin = -200;
//            xMax = 400;
//            yMin = -150;
//            yMax = 200;
//            zMin = -50;
//            zMax = 50;
            //
        }
    }

    if (millis() - lastTime > 100)
    {
        char outMsg[256];


        // Get a reading from the heading sensor
//      float actualHeading = getcompasscourse();
//      kalmanFilter.update(actualHeading);
//      actualHeading = kalmanFilter.GetValue();

        // Send it to the controller
//      sprintf(outMsg, "H:%d\r", (int)actualHeading);
//      Serial.print(outMsg);

        // Get readings from the distance sensors
        int distance1 = 0;
	int distance2 = 0;
        GetDistanceReadings(&distance1, &distance2);

        // Send them to the controller
        sprintf(outMsg, "DIS1:%d\r", distance1);
        Serial.print(outMsg);
        sprintf(outMsg, "DIS2:%d\r", distance2);
        Serial.print(outMsg);

#ifdef JUNK
        // Get a reading from the GPS
        long latitude = latArray[gpsCnt];
        long longitude = longArray[gpsCnt];
        gpsCnt++;
        if (gpsCnt > GPSMAX - 1)
            gpsCnt = 0;
        char latStr[16];
        char longStr[16];
        sprintf(latStr, "%li", latitude);
        sprintf(longStr, "%li", longitude);
        for (char *c = latStr + strlen(latStr) + 1; c > latStr + 1; c--)
            *c = *(c - 1);
        latStr[2] = '.';

        for (char *c = longStr + strlen(longStr) - 1; c > longStr + 2; c--)
          *c = *(c - 1);
        longStr[3] = '.';



        // Send it to the controller
//        sprintf(outMsg, "LA:40.123456\r");
        sprintf(outMsg, "LA:%s\r", latStr);
        Serial.print(outMsg);
//        sprintf(outMsg, "LO:-75.654321\r");
        sprintf(outMsg, "LO:%s\r", longStr);
        Serial.print(outMsg);
#endif


#ifdef PUT_THIS_BACK_WHEN_IMU_CONNECTED    
        // Get raw magnetometer data
        int mX, mY, mZ;
        getRawMagData(&mX, &mY, &mZ);
    
        // Send it to the controller
        sprintf(outMsg, "MX:%d\r", mX);
        Serial.print(outMsg);
        sprintf(outMsg, "MY:%d\r", mY);
        Serial.print(outMsg);
        sprintf(outMsg, "MZ:%d\r", mZ);
        Serial.print(outMsg);
    
        // Get raw accelerometer data
        float aX, aY, aZ;
        getRawAccelData(&aX, &aY, &aZ);
    
        // Send it to the controller
        sprintf(outMsg, "AX:%d\r", (int)(aX * 100));
        Serial.print(outMsg);
        sprintf(outMsg, "AY:%d\r", (int)(aY * 100));
        Serial.print(outMsg);
        sprintf(outMsg, "AZ:%d\r", (int)(aZ * 100));
        Serial.print(outMsg);

        // Get raw gyro data
        int gX, gY, gZ;
        int gyroDeltaT;
        getRawGyroData(&gX, &gY, &gZ, &gyroDeltaT);

        // Send it to the controller
        sprintf(outMsg, "GX:%d\r", gX);
        Serial.print(outMsg);
        sprintf(outMsg, "GY:%d\r", gY);
        Serial.print(outMsg);
        sprintf(outMsg, "GZ:%d\r", gZ);
        Serial.print(outMsg);
        sprintf(outMsg, "GDT:%d\r", gyroDeltaT);
        Serial.print(outMsg);        
#endif


        // Get the state of switch1
        // Send it to the controller
        sprintf(outMsg, "S1:%d\r", switch1);
        Serial.print(outMsg);

#if 0
        if (millis() - calibrationTime > 1000)
        {
        // Send calibration data to controller once every second
            sprintf(outMsg, "CXMIN:%d\r", xMin);
            Serial.print(outMsg);
            sprintf(outMsg, "CYMIN:%d\r", yMin);
            Serial.print(outMsg);
            sprintf(outMsg, "CZMIN:%d\r", zMin);
            Serial.print(outMsg);
            sprintf(outMsg, "CXMAX:%d\r", xMax);
            Serial.print(outMsg);
            sprintf(outMsg, "CYMAX:%d\r", yMax);
            Serial.print(outMsg);
            sprintf(outMsg, "CZMAX:%d\r", zMax);
            Serial.print(outMsg);
            calibrationTime = millis();
        }
#endif
        
        lastTime = millis();
    }    
}


void GetDistanceReadings(int *distance1, int *distance2)
{
    // Sensor 1
    digitalWrite(DIST_SENSOR1_TRIGGER_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(DIST_SENSOR1_TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(DIST_SENSOR1_TRIGGER_PIN, LOW);
    long duration = pulseIn(DIST_SENSOR1_ECHO_PIN, HIGH, 5000);
  
    *distance1 = (duration / 2) / 29.1;


    // Sensor 2
    digitalWrite(DIST_SENSOR2_TRIGGER_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(DIST_SENSOR2_TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(DIST_SENSOR2_TRIGGER_PIN, LOW);
    duration = pulseIn(DIST_SENSOR2_ECHO_PIN, HIGH, 5000);
  
    *distance2 = (duration / 2) / 29.1;
}


#ifdef JUNK
void loop()
{  
   int leftMotorPower;
   int rightMotorPower;
      
   DetectObstacles();
   SteerToHeading();
   WallFollower();
   
   leftMotorPower = FULL_SPEED;
   rightMotorPower = FULL_SPEED;   
   if (steerToHeadingControl.active)
   { 
      leftMotorPower = steerToHeadingControl.leftMotorPower;
      rightMotorPower = steerToHeadingControl.rightMotorPower;
   }
   if (wallFollowerControl.active)
   {
      // See if we're heading back in the desired direction.  If we are, we've followed the wall
      // or obstacle around to a point where we can now continue on our desired course.
      float heading = getcompasscourse();
      if (fabs(SetHeading - heading) < 10)
      {
         wallFollowerControl.active = false;
      }
      else
      {
         // Otherwise, keep following the wall or obstacle
         leftMotorPower = wallFollowerControl.leftMotorPower;
         rightMotorPower = wallFollowerControl.rightMotorPower;
      }
   }
   if (detectObstaclesControl.active)
   {
     forwardMotor(LEFT_MOTOR, FULL_SPEED);
     reverseMotor(RIGHT_MOTOR, FULL_SPEED);
     delay(2000);
     leftMotorPower = FULL_SPEED;
     rightMotorPower = FULL_SPEED;
     wallFollowerControl.active = true;
   }
   if (0)
   {
       // Insert higher priority motor controls here
   }
   
   forwardMotor(LEFT_MOTOR, leftMotorPower);
   forwardMotor(RIGHT_MOTOR, rightMotorPower);
}
#endif


void readCommand(char *command)
{
    char ch = 0;
    int idx = 0;
    while (ch != '\r')
    {
	ch = Serial.read();
        delay(10);
	if (ch != '\r')
	    command[idx++] = ch;
    }
    command[idx] = '\0';
}

void SteerToHeading()
{
   float actualHeading = getcompasscourse();
   kalmanFilter.update(actualHeading);
   actualHeading = kalmanFilter.GetValue();

//   if (SetHeading - actualHeading > 180)
//       actualHeading += 360;
//   else if (SetHeading - actualHeading < -180)
//       actualHeading -= 360;
  Input = actualHeading;

   Serial.print("Heading = ");
   Serial.println(actualHeading);

//  Serial.print("   Input = ");
//  Serial.print(Input);
  headingPID.Compute();

//  Serial.print("   Output = ");
//  Serial.println(Output);

  steerToHeadingControl.leftMotorPower = FULL_SPEED + Output;
  steerToHeadingControl.rightMotorPower = FULL_SPEED - Output;
  
  steerToHeadingControl.active = true;
}


void WallFollower()
{
    // Turn distanceServo to 45 degrees...
    // distanceServo.SetAngle(45); // or something like that
    
    wallPIDInput = getDistance();
    wallFollowerPID.Compute();
    
    wallFollowerControl.leftMotorPower = FULL_SPEED + wallPIDOutput;
    wallFollowerControl.rightMotorPower = FULL_SPEED - wallPIDOutput;
}

void DetectObstacles()
{
   int distance = getDistance();
   if (distance < 40 ) // cm
   {
       detectObstaclesControl.active = true;
   }
   else
       detectObstaclesControl.active = false;
}

int getDistance()
{
  // Do something useful here
  return 50;
}

void getRawMagData(int *x, int *y, int *z)
{
  // Get magnetometer data and scale it to a unit circle with the calibraton limits
  MagnetometerScaled scaled = compass.ReadScaledAxis();
  *x = scaled.XAxis;
  *y = scaled.YAxis;
  *z = scaled.ZAxis;

#ifdef JUNK
    MagnetometerRaw raw = compass.ReadRawAxis();
    *x = raw.XAxis;
    *y = raw.YAxis;
    *z = raw.ZAxis;
#endif
}

void getRawAccelData(float *x, float *y, float *z)
{
  // Get accelerometer data
  sensors_event_t accelEvent;  
  accel.getEvent(&accelEvent);
  *x = accelEvent.acceleration.x;
  *y = accelEvent.acceleration.y;
  *z = accelEvent.acceleration.z;
}

unsigned long lastGyroTime = 0;

void getRawGyroData(int *x, int *y, int *z, int *gdt)
{
  // Get gyro data

    gyro.read();
    *x = gyro.g.x;
    *y = gyro.g.y;
    *z = gyro.g.z;

    unsigned long now = millis();
    *gdt = now - lastGyroTime;
    lastGyroTime = now;
}

float getcompasscourse()
{  
  // Get magnetometer data and scale it to a unit circle with the calibraton limits
  MagnetometerScaled scaled = compass.ReadScaledAxis();
  float cx = (scaled.XAxis-xMin)/(xMax-xMin) - 0.5;
  float cy = (scaled.YAxis-yMin)/(yMax-yMin) - 0.5;
  float cz = (scaled.ZAxis-zMin)/(zMax-zMin) - 0.5;

  // Filter the raw data to remove junk
//  xFilter.update(cx);
//  cx = xFilter.GetValue();
//  yFilter.update(cy);
//  cy = yFilter.GetValue();
//  zFilter.update(cz);
//  cz = zFilter.GetValue();

  delay(50);
  
  // Get accelerometer data; convert from m/s to g
  sensors_event_t accelEvent;  
  accel.getEvent(&accelEvent);
  float Rx = accelEvent.acceleration.x / 9.8;
  float Ry = accelEvent.acceleration.y / 9.8;
  float Rz = accelEvent.acceleration.z / 9.8;

  // Calculate roll, pitch and yaw angles, taking accel and mag into account
  float rollAngle = atan2(Ry, Rz);
  float pitchAngle = atan(-Rx / ((Ry * sin(rollAngle)) + (Rz * cos(rollAngle))));

//  float xCalibration = -50.0;
//  float yCalibration = 200.0;
  float y = (cz * sin(rollAngle)) - (cy * cos(rollAngle));// + xCalibration;
  float x = (cx * cos(pitchAngle))
              + (cy * sin(pitchAngle) * sin(rollAngle))
              + (cz * sin(pitchAngle) * cos(rollAngle));// + yCalibration;

//  Serial.print(x);
//  Serial.print(",");
//  Serial.println(y);

  float yawAngle = atan2(y, x);// - 3.14 / 2;
  
  // Factor in declination angle for this location
//  float declinationAngle = -0.207403293;
//  yawAngle += declinationAngle;


  // Convert to degrees  
  yawAngle = yawAngle * 180 / 3.14;

  // Convert to range (0, 360)
  yawAngle = (yawAngle > 0.0 ? yawAngle : (360.0 + yawAngle));   
//   Serial.print("Heading = ");
//   Serial.println(yawAngle);
  return fabs(yawAngle - 360);
}

void calibrateMagSensor()
{  
    digitalWrite(13, HIGH);

    SetMotorSpeed(LEFT_MOTOR, FULL_SPEED);
    SetMotorSpeed(RIGHT_MOTOR, -FULL_SPEED);
  
    bool quad1 = false;
    bool quad2 = false;
    bool quad3 = false;
    bool quad4 = false;
    int repetitions = 0;
    while (repetitions < 1 && !(quad1 && quad2 && quad3 &&quad4))
    {
        // Get magnetometer data
        MagnetometerScaled scaled = compass.ReadScaledAxis();
        float cx = scaled.XAxis;
        float cy = scaled.YAxis;
        float cz = scaled.ZAxis;
  
        // Filter the raw data to remove junk
        if (cx < -1000 || cx > 1000 || cy < -1000 || cy > 1000 || cz < -1000 || cz > 1000)
            continue;
    
        xFilter.update(cx);
        cx = xFilter.GetValue();
        yFilter.update(cy);
        cy = yFilter.GetValue();
        zFilter.update(cz);
        cz = zFilter.GetValue();
    
        if (cx < xMin)
            xMin = cx;
        if (cx > xMax)
            xMax = cx;
        if (cy < yMin)
            yMin = cy;
        if (cy > yMax)
            yMax = cy;
        if (cz < zMin)
            zMin = cz;
        if (cz > zMax)
            zMax = cz;
  
        if (!quad1 && cx > 0 && cy > 0)
        {
            quad1 = true;
        }
        if (!quad2 && cx < 0 && cy > 0)
        {
            quad2 = true;
        }
        if (!quad3 && cx < 0 && cy < 0)
        {
            quad3 = true;
        }
        if (!quad4 && cx > 0 && cy < 0)
        {
            quad4 = true;
        }

        if (quad1 && quad2 && quad3 && quad4)    
        {
            repetitions++;
            quad1 = false;
            quad2 = false;
            quad3 = false;
            quad4 = false;
        }
    }
  
    digitalWrite(13, LOW);
    SetMotorSpeed(LEFT_MOTOR, FULL_SPEED);
    SetMotorSpeed(RIGHT_MOTOR, FULL_SPEED);
}

#ifdef JUNK
void forwardMotor(int motor, int motorSpeed)
{
    switch (motor)
    {
        case LEFT_MOTOR:
                        digitalWrite(IN1, LOW); 
                        digitalWrite(IN2, HIGH);
                        analogWrite(ENA, motorSpeed);
                        break;
                        
        case RIGHT_MOTOR:
                        digitalWrite(IN3, LOW);
                        digitalWrite(IN4, HIGH);
                        analogWrite(ENB, motorSpeed);
                        break;
    }
}

void reverseMotor(int motor, int motorSpeed)
{
    switch (motor)
    {
        case LEFT_MOTOR:
                        digitalWrite(IN1, HIGH); 
                        digitalWrite(IN2, LOW);
                        analogWrite(ENA, motorSpeed);
                        break;
                        
        case RIGHT_MOTOR:
                        digitalWrite(IN3, HIGH);
                        digitalWrite(IN4, LOW);
                        analogWrite(ENB, motorSpeed);
                        break;
    }
}
#endif

void SetMotorDirection(int motor, int direction)
{
    switch (motor)
    {
        case LEFT_MOTOR:
                        if (direction == DIR_FORWARD)
                        {
                            digitalWrite(IN1, HIGH); 
                            digitalWrite(IN2, LOW);
                        }
                        else
                        {
                            digitalWrite(IN1, LOW);
                            digitalWrite(IN2, HIGH);
                        }
                        break;
                        
        case RIGHT_MOTOR:
                        if (direction == DIR_FORWARD)
                        {
                            digitalWrite(IN3, HIGH);
                            digitalWrite(IN4, LOW);
                        }
                        else
                        {
                            digitalWrite(IN3, LOW);
                            digitalWrite(IN4, HIGH);
                        }
                        break;
    }
}

void SetMotorSpeed(int motor, int speed)
{
    if (speed > 0)
    {
        SetMotorDirection(motor, DIR_FORWARD);
    }
    else if (speed < 0)
    {
        SetMotorDirection(motor, DIR_REVERSE);
    }
    else
    {
        stopMotor(motor);
    }

    switch (motor)
    {
        case LEFT_MOTOR:
                    analogWrite(ENA, abs(speed));
                    break;
                        
        case RIGHT_MOTOR:
                    analogWrite(ENB, abs(speed));
                    break;
    }
}

void stopMotor(int motor)
{
    switch (motor)
    {
        case LEFT_MOTOR:
                        digitalWrite(IN1, LOW); 
                        digitalWrite(IN2, LOW);
                        analogWrite(ENA, 0);
                        break;
                        
        case RIGHT_MOTOR:
                        digitalWrite(IN3, LOW);
                        digitalWrite(IN4, LOW);
                        analogWrite(ENB, 0);
                        break;
    }
}

 

Kalman::Kalman(double _q, double _r, double _p, double _intial_value)
{
       q = _q;
       r = _r;
       p = _p;
       x = _intial_value;
};

void Kalman::update(double measurement)
{
       //prediction update
       //omit x = x
       this->p = this->p + this->q;

       //measurement update
       this->k = this->p / (this->p + this->r);
       this->x = this->x + this->k * (measurement - this->x);
       this->p = (1 - this->k) * this->p;
}


