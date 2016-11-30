#include <Servo.h>
#include <NewPing.h>

void readCommand(char *command);
void SteerToHeading();
void WallFollower();
void DetectObstacles();
int getDistance();
float getcompasscourse();
void calibrateMagSensor();
void stopMotor(int motor);

void SetMotorSpeed(int motor, int speed);


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

#define DIST_SENSOR1_TRIGGER_PIN A1
#define DIST_SENSOR1_ECHO_PIN A0
#define DIST_SENSOR2_TRIGGER_PIN A3
#define DIST_SENSOR2_ECHO_PIN A2
#define DIST_SENSOR3_TRIGGER_PIN A5
#define DIST_SENSOR3_ECHO_PIN A4

NewPing distSensor1(DIST_SENSOR1_TRIGGER_PIN, DIST_SENSOR1_ECHO_PIN, 150);
NewPing distSensor2(DIST_SENSOR2_TRIGGER_PIN, DIST_SENSOR2_ECHO_PIN, 150);
NewPing distSensor3(DIST_SENSOR3_TRIGGER_PIN, DIST_SENSOR3_ECHO_PIN, 150);

#define VOLTAGE_SENSOR1_PIN A6

class ControlMode
{
public:
    bool active;
    int leftMotorPower;
    int rightMotorPower;
    
    ControlMode() {active = false; leftMotorPower = 0; rightMotorPower = 0; };
};


void GetDistanceReadings(int *distance1, int *distance2, int *distance3);


void setup()
{
    pinMode(ENA,OUTPUT);
    pinMode(ENB,OUTPUT);
    pinMode(IN1,OUTPUT);
    pinMode(IN2,OUTPUT);
    pinMode(IN3,OUTPUT);
    pinMode(IN4,OUTPUT);

    Serial.begin(115200);
     
    // Both stopped
    stopMotor(LEFT_MOTOR);
    stopMotor(RIGHT_MOTOR);
    digitalWrite(ENA,LOW);
    digitalWrite(ENB,LOW);

    delay(2000);

    pinMode(13, OUTPUT);
    pinMode(12, OUTPUT);
    digitalWrite(12, HIGH);

    SetMotorSpeed(LEFT_MOTOR, 0);
    SetMotorSpeed(RIGHT_MOTOR, 0);
    
    servo1.attach(servo1_pin);
    servo2.attach(servo2_pin);
    servo3.attach(servo3_pin);

    // Set up the distance sensor pins
//    pinMode(DIST_SENSOR1_TRIGGER_PIN, OUTPUT);
//    pinMode(DIST_SENSOR1_ECHO_PIN, INPUT);
//    pinMode(DIST_SENSOR2_TRIGGER_PIN, OUTPUT);
//    pinMode(DIST_SENSOR2_ECHO_PIN, INPUT);
//    pinMode(DIST_SENSOR3_TRIGGER_PIN, OUTPUT);
//    pinMode(DIST_SENSOR3_ECHO_PIN, INPUT);

    // Set up the voltage sensor pin
    pinMode(VOLTAGE_SENSOR1_PIN, INPUT);
}


char msg[32];
int idx = 0;
unsigned long lastTime = 0;
unsigned long calibrationTime = 0;
int switch1 = 0;

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
        else if (!strcmp(msgType, "S3"))  // Tilt Servo
        {
            // Set Servo 3 angle
            int angle = strtol(val, 0, 10);
	    angle = -angle;
	    angle += 132;
            servo3.write(angle);
        }
        else if (!strcmp(msgType, "S2"))  // Pan Servo
        {
            // Set Servo 2 angle
            int angle = strtol(val, 0, 10);
	    angle += 97;
            servo2.write(angle);
        }
	else if (!strcmp(msgType, "R1"))  // Relay 1
	{
	    int value = strtol(val, 0, 10);
	    digitalWrite(12, !value);
	}
    }

    if (millis() - lastTime > 100)
    {
        char outMsg[256];

        // Get readings from the distance sensors
        int distance1 = 0;
	int distance2 = 0;
	int distance3 = 0;
        GetDistanceReadings(&distance1, &distance2, &distance3);

        // Send them to the controller
        sprintf(outMsg, "DIS1:%d\r", distance1);
        Serial.print(outMsg);
        sprintf(outMsg, "DIS2:%d\r", distance2);
        Serial.print(outMsg);
        sprintf(outMsg, "DIS3:%d\r", distance3);
        Serial.print(outMsg);


        // Get the state of switch1
        // Send it to the controller
        sprintf(outMsg, "S1:%d\r", switch1);
        Serial.print(outMsg);


	// Get the value of the voltage sensor
	// Send it to the controller
	int voltagePinVal = analogRead(VOLTAGE_SENSOR1_PIN);
	double voltage = (double)voltagePinVal * 14.8 / 1024;
	sprintf(outMsg, "VLT1:%d\r", (int)(voltage * 100));
	Serial.print(outMsg);

        lastTime = millis();
    }    
}


void GetDistanceReadings(int *distance1, int *distance2, int *distance3)
{
    *distance1 = 0;
    *distance2 = 0;
    *distance3 = 0;
    int echoTime = distSensor1.ping_median(3);
    if (echoTime != NO_ECHO)
	*distance1 = distSensor1.convert_cm(echoTime);

    return;  /////////////////////////////////////////////////////  Remove This //////////////////////////

    echoTime = distSensor2.ping_median();
    if (echoTime != NO_ECHO)
	*distance2 = distSensor2.convert_cm(echoTime);

    echoTime = distSensor3.ping_median();
    if (echoTime != NO_ECHO)
	*distance3 = distSensor3.convert_cm(echoTime);


#ifdef JUNK
    // Sensor 1
    digitalWrite(DIST_SENSOR1_TRIGGER_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(DIST_SENSOR1_TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(DIST_SENSOR1_TRIGGER_PIN, LOW);
    long duration = pulseIn(DIST_SENSOR1_ECHO_PIN, HIGH, 17500);
  
    *distance1 = (duration / 2) / 29.1;

    delay(10);

    // Sensor 2
    digitalWrite(DIST_SENSOR2_TRIGGER_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(DIST_SENSOR2_TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(DIST_SENSOR2_TRIGGER_PIN, LOW);
    duration = pulseIn(DIST_SENSOR2_ECHO_PIN, HIGH, 17500);
  
    *distance2 = (duration / 2) / 29.1;

    delay(10);

    // Sensor 3
    digitalWrite(DIST_SENSOR3_TRIGGER_PIN, LOW);
    delayMicroseconds(5);
    digitalWrite(DIST_SENSOR3_TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(DIST_SENSOR3_TRIGGER_PIN, LOW);
    duration = pulseIn(DIST_SENSOR3_ECHO_PIN, HIGH, 17500);
  
    *distance3 = (duration / 2) / 29.1;
#endif
}


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

