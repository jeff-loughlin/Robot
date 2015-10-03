#include <time.h>
#include <sys/time.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <ctype.h>
#include "PID_v1.h"
#include "kalman.h"
#include <pthread.h>
#include <sys/types.h>
#include <sys/wait.h>
#include "include.h"
#include "geocoords.h"
#include "gps.h"
#include "imu.h"

//#define _POSIX_SOURCE 1 /* POSIX compliant source */

//#define PI 3.14159265
#define RADTODEG (180 / PI)
#define DEGTORAD (PI / 180)

#define BAUDRATE B115200

#define MAX_SPEED 150
#define NORMAL_SPEED 50
#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1

#define SUBSUMPTION_INTERVAL 100
#define LED_BLINK_INTERVAL 1000
#define CALCULATE_GPS_HEADING_INTERVAL 1000


float G_GAIN = 0.07; //0.00875; //0.07;
float DT = 0.02; // [s/loop] loop period. 20ms
//#define M_PI = 3.14159265358979323846
float RAD_TO_DEG = 57.29578;
float AA = 0.98; // complementary filter constant
const int CAL_SAMPLES = 10;


// Mag calibration constants
short xMin = -321;
short xMax = 251;
short yMin = 3;
short yMax = 589;
short zMin = -527;//0;
short zMax = 486;//768




FILE *outFile;

// Get current waypoint.  TODO: change this from hard-coded values to something useful
//float wpLatitude = 39.971821;
//float wpLongitude = -75.736046;

GeoCoordinate waypoints[256];
int waypointCount = 0;
int currentWaypoint = 0;
double waypointRange = 0.0;
void ReadWaypointsFile();

// Sensor values (read from serial link)
// Each sensor value has a corresponding _t that will contain the timestamp for the last update to that value.
// This allows us to detect stale data and ignore it if necessary.
float heading = 0;
unsigned long heading_t;
float complexHeading = 0;
double latitude = 0;
unsigned long latitude_t;
double longitude = 0;
unsigned long longitude_t;
double latitude_error;
double longitude_error;
int gps_fix = MODE_NOT_SEEN;
int distance1 = 0;
unsigned long distance1_t;
int distance2 = 0;
unsigned long distance2_t;
int switch1 = 0;
unsigned long switch1_t;
int switch2 = 0;
unsigned long switch2_t;
int magX, magY, magZ;
unsigned long magX_t, magY_t, magZ_t;
double accelX, accelY, accelZ;
unsigned long accelX_t, accelY_t, accelZ_t;
int gyroX, gyroY, gyroZ;
unsigned long gyroX_t, gyroY_t, gyroZ_t;
int gyroDeltaT;
unsigned long gyroDeltaT_t;

bool ouchEnabled = true;

// Actuator values (sent to serial link)
int leftPower;
int rightPower;

// File descriptor for serial device
int fdSerial = 0;

// File descriptor for the manual control FIFO
int fdFifo = 0;

// Magnetometer calibration values.
//int xMax, yMax, zMax, xMin, yMin, zMin;

// Gyro calibration values
int gyroXCal = 0, gyroYCal = 0, gyroZCal = 0;

// Accelerometer calibration values
int accelXCal = 0, accelYCal = 0, accelZCal = 980;

// Subsumption task control class
class ControlMode
{
public:
    bool active;
    int leftMotorPower;
    int rightMotorPower;

    ControlMode() {active = false; leftMotorPower = 0; rightMotorPower = 0; };
};


// Kalman filters for smoothing magnetometerm, accelerometer, and calculated heading data
Kalman headingFilter(0.125, 4, 1, 0);
Kalman xFilter(0.125, 4, 1, 0);
Kalman yFilter(0.125, 4, 1, 0);
Kalman zFilter(0.125, 4, 1, 0);

Kalman xAccFilter(0.125, 4, 1, 0);
Kalman yAccFilter(0.125, 4, 1, 0);
Kalman zAccFilter(0.125, 4, 1, 0);



// PID controller variables for heading
double SetHeading, headingPIDInput, headingPIDOutput;
PID headingPID(&headingPIDInput, &headingPIDOutput, &SetHeading,1,0,0, DIRECT);

// PID controller variables for wall follower (distance from wall)
double SetDistance, wallPIDInput, wallPIDOutput;
PID wallFollowerPID(&wallPIDInput, &wallPIDOutput, &SetDistance, 5, .1, .5, DIRECT);

// PID controller variables for manual control
double SetGyro, gyroPIDInput, gyroPIDOutput;
PID gyroPID(&gyroPIDInput, &gyroPIDOutput, &SetGyro,5,0,1, DIRECT);

// Constants for the pan/tilt camera servos
#define PAN_SERVO 1
#define TILT_SERVO 2
#define PAN_MIN -40
#define PAN_MAX 160
#define TILT_MIN 20
#define TILT_MAX 165
#define PAN_MID 100 /*90*/
#define TILT_MID 160

// Servo positions for pan/tilt camera servos
int panServo = PAN_MID;   // defaults to mid point
int tiltServo = TILT_MID;  // defaults to mid point


// Manual control settings - these get set by manual commands coming in through the FIFO.  Start them with sensible defaults.
int manualPowerLeft = 0;
int manualPowerRight = 0;
int manualServoPan = PAN_MID;
int manualServoTilt = TILT_MID;

// Autonomous control or manual control?  This gets set to false when a manual command comes in through the FIFO
bool autonomous = false;


// Quick and dirty function to get elapsed time in milliseconds.  This will wrap at 32 bits (unsigned long), so
// it's not an absolute time-since-boot indication.  It is useful for measuring short time intervals in constructs such
// as 'if (lastMillis - millis() > 1000)'.  Just watch out for the wrapping issue, which will happen every 4,294,967,295
// milliseconds - unless you account for this, I don't recommend using this function for anything that will cause death or
// disembowelment when it suddenly wraps around to zero (e.g. avionics control on an aircraft)...
unsigned long millis()
{
	struct timeval tv;
	gettimeofday(&tv, 0);
	unsigned long count = tv.tv_sec * 1000000 + tv.tv_usec;
	return count / 1000;
}

// Parse a message from the serial link or FIFO.  Message format is generally:  msgType:val1|,val2,val3...|
// This will parse the command string into msgType and value (or a comma separated list of values).  It is
// up to the caller to interpret the values (int, float, list of ints, etc)
void ParseMessage(const char *msg, char *msgType, char *val)
{
    int c = 0;
    while (msg[c] != ':' && msg[c] != 0)
    {
        msgType[c] = msg[c];
        c++;
    }
    msgType[c] = '\0';

    int idx = 0;
    c++;
    while (msg[c] != 0x0d && msg[c] != 0x0a && msg[c] != 0x00)
    {
        val[idx] = msg[c];
        c++;
        idx++;
    }
    val[idx] = '\0';
}

// Process a message received over the serial link
void ProcessSerialMsg(const char *msg)
{
    char msgType[8];
    char val[256];

    ParseMessage(msg, msgType, val);

//    if (!strcmp(msgType, "LA"))
//	{
//		latitude = strtod(val, 0);
//		latitude_t = millis();
//	}
//    else if (!strcmp(msgType, "LO"))
//	{
//		longitude = strtod(val, 0);
//		longitude_t = millis();
//	}
        if (!strcmp(msgType, "DIS1"))
	{
		distance1 = strtol(val, 0, 10);
		distance1_t = millis();
	}
        if (!strcmp(msgType, "DIS2"))
	{
		distance2 = strtol(val, 0, 10);
		distance2_t = millis();
	}
#ifdef JUNK
	else if (!strcmp(msgType, "MX"))
	{
	    magX = strtol(val, 0, 10);
	    magX_t = millis();
	}
	else if (!strcmp(msgType, "MY"))
	{
	    magY = strtol(val, 0, 10);
	    magY_t = millis();
	}
	else if (!strcmp(msgType, "MZ"))
	{
	    magZ = strtol(val, 0, 10);
	    magZ_t = millis();
	}
	else if (!strcmp(msgType, "AX"))
	{
	    accelX = (float)(strtol(val, 0, 10));
	    accelX_t = millis();

        // Invert the X axis because it is mounted upside-down
//        accelX = -accelX;

        // Apply the calibration offset to correct for constant offset
        accelX -= accelXCal;
	}
	else if (!strcmp(msgType, "AY"))
	{
	    accelY = (float)(strtol(val, 0, 10));
	    accelY_t = millis();

        // Invert the Y axis becasue it is mounted upside-down
//        accelY = -accelY;

        // Apply the calibration offset to correct for constant offset
        accelY -= accelYCal;
	}
	else if (!strcmp(msgType, "AZ"))
	{
	    accelZ = (float)(strtol(val, 0, 10));
	    accelZ_t = millis();

        // Invert the Z axis because it is mounted upside-down
//        accelZ = -accelZ;

        // Apply the calibration offset to correct for constant offset.  (NOTE: Level setting for ZAxis is 980)
        accelZ -= (accelZCal - 980);
	}
	else if (!strcmp(msgType, "GX"))
	{
	    gyroX = strtol(val, 0, 10);
	    gyroX_t = millis();

        gyroX -= gyroXCal;
	}
	else if (!strcmp(msgType, "GY"))
	{
	    gyroY = strtol(val, 0, 10);
	    gyroY_t = millis();

        gyroY -= gyroYCal;
	}
	else if (!strcmp(msgType, "GZ"))
	{
	    gyroZ = strtol(val, 0, 10);
	    gyroZ_t = millis();

        gyroZ -= gyroZCal;
	}
#endif
	else if (!strcmp(msgType, "GDT"))
	{
	    gyroDeltaT = strtol(val, 0, 10);
	    gyroDeltaT_t = millis();
	}
    else if (!strcmp(msgType, "S1"))
	{
		switch1 = strtol(val, 0, 10);
		switch1_t = millis();
	}
    else if (!strcmp(msgType, "S2"))
	{
		switch2 = strtol(val, 0, 10);
		switch2_t = millis();
	}
}





float rollAngle = 0;
float pitchAngle = 0;


float getHeading(float ax, float ay, float az, float mx, float my, float mz, float gx, float gy, float gz, int offset, int n)
{
   static float rate_gyr_x(0.0),rate_gyr_y(0.0),rate_gyr_z(0.0);
   static float gyroXangle(0.0),gyroYangle(0.0),gyroZangle(0.0);

//   static float AccYangle(0.0),AccXangle(0.0);

   static float CFangleX = 0.0;
   static float CFangleY = 0.0;



  /* Make sure the input is valid, not null, etc. */
//  my = -my;
//  mx = -mx;

//  float tmp = ax;
//  ax = ay;
//  ay = tmp;



  float const PI_F = 3.14159265F;

  /* roll: Rotation around the X-axis. -180 <= roll <= 180                                          */
  /* a positive roll angle is defined to be a clockwise rotation about the positive X-axis          */
  /*                                                                                                */
  /*                    y                                                                           */
  /*      roll = atan2(---)                                                                         */
  /*                    z                                                                           */
  /*                                                                                                */
  /* where:  y, z are returned value from accelerometer sensor                                      */
  float roll = (float)atan2(ay, az);
  float pitch;


  /* pitch: Rotation around the Y-axis. -180 <= roll <= 180                                         */
  /* a positive pitch angle is defined to be a clockwise rotation about the positive Y-axis         */
  /*                                                                                                */
  /*                                 -x                                                             */
  /*      pitch = atan(-------------------------------)                                             */
  /*                    y * sin(roll) + z * cos(roll)                                               */
  /*                                                                                                */
  /* where:  x, y, z are returned value from accelerometer sensor                                   */
  if (ay * sin(roll) + az * cos(roll) == 0)
    pitch = ax > 0 ? (PI_F / 2) : (-PI_F / 2);
  else
    pitch = (float)atan(-ax / (ay * sin(roll) + az * cos(roll)));


//  pitch = -pitch;
  roll = -roll;  //Move this up above the pitch calculation?

      //Convert Gyro raw to degrees per second
      rate_gyr_x = (float)gx * G_GAIN;
      rate_gyr_y = (float)gy * G_GAIN;
      rate_gyr_z = (float)gz * G_GAIN;

      //Calculate the angles from the gyro
      gyroXangle += rate_gyr_x*DT;
      gyroYangle += rate_gyr_y*DT;
      gyroZangle += rate_gyr_z*DT;
      CFangleX=AA*(CFangleX+rate_gyr_x*DT) +(1 - AA) * (roll * 180 / 3.14159265);
      CFangleY=AA*(CFangleY+rate_gyr_y*DT) +(1 - AA) * (pitch * 180 / 3.14159265);


      float CFangleXRad = CFangleX * 3.14159265 / 180;
      float CFangleYRad = CFangleY * 3.14159265 / 180;
  /* heading: Rotation around the Z-axis. -180 <= roll <= 180                                       */
  /* a positive heading angle is defined to be a clockwise rotation about the positive Z-axis       */
  /*                                                                                                */
  /*                                       z * sin(roll) - y * cos(roll)                            */
  /*   heading = atan2(--------------------------------------------------------------------------)  */
  /*                    x * cos(pitch) + y * sin(pitch) * sin(roll) + z * sin(pitch) * cos(roll))   */
  /*                                                                                                */
  /* where:  x, y, z are returned value from magnetometer sensor                                    */

//  float heading = (float)atan2(mz * sin(roll) - my * cos(roll),
//                                      mx * cos(pitch) +
//                                      my * sin(pitch) * sin(roll) +
//                                      mz * sin(pitch) * cos(roll));
  float heading = (float)atan2(mz * sin(CFangleXRad) - my * cos(CFangleXRad), \
                                      mx * cos(CFangleYRad) + \
                                      my * sin(CFangleYRad) * sin(CFangleXRad) + \
                                      mz * sin(CFangleYRad) * cos(CFangleXRad));


    // Factor in the declination angle for this location  (from http://www.ngdc.noaa.gov/geomag-web/#declination)
    float declinationAngle = -0.20780084;  // should this be negative?
    heading += declinationAngle;


  /* Convert angular heading to degrees */
  roll = roll * 180 / PI_F;
  pitch = pitch * 180 / PI_F;
  heading = heading * 180 / PI_F;

   // Compensate for the sensor being rotated with respect to the device.  Caller specified by how much.
  heading += offset;


    // Convert to range (0, 360)
    heading = (heading > 0.0 ? heading : (360.0 + heading));

  return heading;
}


// Initialization stuff - open and configure the serial device, etc.
void Setup(int *fdSerial, const char *serialDev)
{
    struct termios oldtio, newtio;

    //   Open serial device for reading and writing and not as controlling tty
    //   because we don't want to get killed if linenoise sends CTRL-C.
    printf("Opening serial device\n");
    *fdSerial = open(serialDev, O_RDWR | O_NOCTTY );
    if (*fdSerial < 0)
    {
		printf("Failed to open tty device\n");
        perror(serialDev);
        exit(-1);
    }

	printf("Setting serial device attributes\n");

    tcgetattr(*fdSerial, &oldtio); /* save current serial port settings */
	memset(&newtio, 0, sizeof(newtio));
    /*
          BAUDRATE: Set bps rate. You could also use cfsetispeed and cfsetospeed.
          CRTSCTS : output hardware flow control (only used if the cable has
                    all necessary lines. See sect. 7 of Serial-HOWTO)
          CS8     : 8n1 (8bit,no parity,1 stopbit)
          CLOCAL  : local connection, no modem contol
          CREAD   : enable receiving characters
    */
    newtio.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;

    /*
          IGNPAR  : ignore bytes with parity errors
          ICRNL   : map CR to NL (otherwise a CR input on the other computer
                    will not terminate input)
          otherwise make device raw (no other input processing)
    */
    newtio.c_iflag = IGNPAR | ICRNL;

    /*
         Raw output.
    */
    newtio.c_oflag = 0;

    /*
          ICANON  : enable canonical input
          disable all echo functionality, and don't send signals to calling program
    */
    newtio.c_lflag = ICANON;

    /*
          initialize all control characters
          default values can be found in /usr/include/termios.h, and are given
          in the comments, but we don't need them here
    */
    newtio.c_cc[VINTR]    = 0;     /* Ctrl-c */
    newtio.c_cc[VQUIT]    = 0;     /* Ctrl-\ */
    newtio.c_cc[VERASE]   = 0;     /* del */
    newtio.c_cc[VKILL]    = 0;     /* @ */
    newtio.c_cc[VEOF]     = 4;     /* Ctrl-d */
    newtio.c_cc[VTIME]    = 0;     /* inter-character timer unused */
    newtio.c_cc[VMIN]     = 1;     /* blocking read until 1 character arrives */
    newtio.c_cc[VSWTC]    = 0;     /* '\0' */
    newtio.c_cc[VSTART]   = 0;     /* Ctrl-q */
    newtio.c_cc[VSTOP]    = 0;     /* Ctrl-s */
    newtio.c_cc[VSUSP]    = 0;     /* Ctrl-z */
    newtio.c_cc[VEOL]     = 0;     /* '\0' */
    newtio.c_cc[VREPRINT] = 0;     /* Ctrl-r */
    newtio.c_cc[VDISCARD] = 0;     /* Ctrl-u */
    newtio.c_cc[VWERASE]  = 0;     /* Ctrl-w */
    newtio.c_cc[VLNEXT]   = 0;     /* Ctrl-v */
    newtio.c_cc[VEOL2]    = 0;     /* '\0' */

    /*
          now clean the modem line and activate the settings for the port
    */
    tcflush(*fdSerial, TCIFLUSH);
    tcsetattr(*fdSerial,TCSANOW,&newtio);



    // Set up the PID controllers for heading and wall following
//    SetHeading = 137.0;
    headingPIDInput = 0;
    headingPID.SetOutputLimits(-NORMAL_SPEED, NORMAL_SPEED);
    headingPID.SetMode(AUTOMATIC);

    SetDistance = 40.0;
    wallFollowerPID.SetOutputLimits(-NORMAL_SPEED, NORMAL_SPEED);
    wallFollowerPID.SetMode(AUTOMATIC);

    gyroPIDInput = 0;
    gyroPID.SetOutputLimits(-12000, 12000);
    gyroPID.SetMode(AUTOMATIC);
}


// Send a command to the motor controller to set the speed of a motor
void SetMotorSpeed(int motor, int speed)
{
    // Make sure we stay in the range -255 < speed < 255
    if (speed < -MAX_SPEED)
        speed = -MAX_SPEED;
    if (speed > MAX_SPEED)
        speed = MAX_SPEED;

    char outMsg[256];
    switch (motor)
    {
        case LEFT_MOTOR:    sprintf(outMsg, "LM:%d\r", speed);
                            printf("Setting Left motor speed to \033[1m%d\033[0m\n", speed);
                            break;
        case RIGHT_MOTOR:   sprintf(outMsg, "RM:%d\r", speed);
                            printf("Setting Right motor speed to \033[1m%d\033[0m\n", speed);
                            break;
    }
    write(fdSerial, outMsg, strlen(outMsg));
}

// Send a command to the servo controller to set the angle of a servo
void SetServoAngle(int servo, int angle)
{
    // Make sure we stay in the range 0 < angle < 180
    if (angle < 0)
        angle = 0;
    if (angle > 180)
        angle = 180;

    char outMsg[256];
    switch (servo)
    {
        case PAN_SERVO:     sprintf(outMsg, "S2:%d\r", angle);
                            printf("Setting Pan servo to %d\n", angle);
                            break;
        case TILT_SERVO:    sprintf(outMsg, "S3:%d\r", angle);
                            printf("Setting Tilt servo to %d\n", angle);
                            break;
    }
    write(fdSerial, outMsg, strlen(outMsg));
}


// Steer to heading subsumption task.  If active and not subsumed by a higher priority task, this will set the motor speeds
// to steer to the given heading (SetHeading)
void SteerToHeading(ControlMode *steerToHeadingControl)
{
    // Filter the mag data to eliminate noise
//    xFilter.update(magX);
//    magX = xFilter.GetValue();
//    yFilter.update(magY);
//    magY = yFilter.GetValue();
//    zFilter.update(magZ);
//    magZ = zFilter.GetValue();

    // Do the same with the accelerometer data
//    xAccFilter.update(accelX);
//    accelX = xAccFilter.GetValue();
//    yAccFilter.update(accelY);
//    accelY = yAccFilter.GetValue();
//    zAccFilter.update(accelZ);
//    accelZ = zAccFilter.GetValue();

    float filteredHeading = heading;
    float adjustedHeading = filteredHeading;

    // Deal with the 0 == 360 problem
    float diff = SetHeading - filteredHeading;
    if (diff > 180)
	adjustedHeading += 360;
    else if (diff < -180)
	adjustedHeading -= 360;

//   if (SetHeading - filteredHeading > 180)
//       filteredHeading += 360;
//   else if (SetHeading - filteredHeading < -180)
//       filteredHeading -= 360;

    headingFilter.update(filteredHeading);
    filteredHeading = headingFilter.GetValue();
    headingPIDInput = adjustedHeading;//filteredHeading;  // adjustedHeading ?
    headingPID.Compute();

    printf("Roll Angle: %3.2f\n", rollAngle);
    printf("PitchAngle: %3.2f\n", pitchAngle);
    printf("Raw Heading:         %f\n", heading);
    printf("\033[1mSet Heading:         %d\033[0m \n", (int)SetHeading);
    printf("\033[1mFiltered Heading:    %d\033[0m \n", (int)filteredHeading);
    printf("PID error:           %d\n", (int)headingPIDOutput);
    fprintf(outFile, "Roll Angle: %3.2f\n", rollAngle);
    fprintf(outFile, "PitchAngle: %3.2f\n", pitchAngle);
    fprintf(outFile, "Heading: %d\n", (int)filteredHeading);
    fprintf(outFile, "Target Heading: %d\n", (int)SetHeading);
    fprintf(outFile, "PID error:           %d\n", (int)headingPIDOutput);
    steerToHeadingControl->leftMotorPower = NORMAL_SPEED - headingPIDOutput;
    steerToHeadingControl->rightMotorPower = NORMAL_SPEED + headingPIDOutput;
    steerToHeadingControl->active = true;
}


// Wall follower subsumption task.  If active and not subsumed by a higher priority task, this will set the motor speeds
// to follow a wall at distance WALL_FOLLOWER_DISTANCE
void WallFollower(ControlMode *wallFollowerControl)
{
    wallPIDInput = distance1;
    wallFollowerPID.Compute();
    wallFollowerControl->leftMotorPower = NORMAL_SPEED + wallPIDOutput;
    wallFollowerControl->rightMotorPower = NORMAL_SPEED - wallPIDOutput;
}


// Detect obstacles subsumption task.  If an onstacle is detected, set active flag to subsume all other tasks.  This
// will generally be the highest priority task (except for manual control), since we always want to avoid obstacles
// regardless of what other tasks are active.
void DetectObstacles(ControlMode *detectObstaclesControl)
{
    // Need to set the servo to LEFT, CENTER, or RIGHT, then wait a few hundres ms for it to get there, then grab the
    // distance reading.  Reading the distance while the servo is moving will generate too much noise.
    //
    // ...
    //
    // Do something useful here (move servo, wait)

    int distanceAhead = distance1;
    if (distanceAhead > 4 && distanceAhead < 40 ) // cm
    {
        detectObstaclesControl->active = true;
    }
    else
        detectObstaclesControl->active = false;
}


void CalculateHeadingToWaypoint()
{
    GeoCoordinate current(latitude, longitude);
    GeoCoordinate waypoint = waypoints[currentWaypoint];

    // getBearing() expects its waypoint coordinates in radians
    waypoint.latitude = waypoint.latitude * PI / 180.0;
    waypoint.longitude = waypoint.longitude * PI / 180.0;

    // SetHeading is the value used by the heading PID controller.  By changing this, we change the heading
    // to which the SteerToHeading subsumption task will try to steer us.
    SetHeading = getBearing(current, waypoint);

    return;
}

void CalculateDistanceToWaypoint()
{
    GeoCoordinate current(latitude, longitude);
    GeoCoordinate waypoint = waypoints[currentWaypoint];

    // getDistance() expects its waypoint coordinates in radians
    waypoint.latitude = waypoint.latitude * PI / 180.0;
    waypoint.longitude = waypoint.longitude * PI / 180.0;

    // SetHeading is the value used by the heading PID controller.  By changing this, we change the heading
    // to which the SteerToHeading subsumption task will try to steer us.
    waypointRange = getDistance(current, waypoint);

    if (waypointRange < 0.0030) // 3.0 meters
	currentWaypoint++;
    if (currentWaypoint >= waypointCount)
	currentWaypoint = 0;

    return;
}

void CalibrateSensors()
{
        printf("\nEntering calibration mode\n");
        printf("Place on a flat level surface\n\n");

        // Now calibrate the gyros and accelerometers
        int xg = 0;
        int yg = 0;
        int zg = 0;
        int ax = 0;
        int ay = 0;
        int az = 0;
        gyroXCal = 0;
        gyroYCal = 0;
        gyroZCal = 0;
        accelXCal = 0;
        accelYCal = 0;
        accelZCal = 980;
        for (int i = 0; i < 100; i++)
        {
            xg += gyroX;
            yg += gyroY;
            zg += gyroZ;
            ax += accelX;
            ay += accelY;
            az += accelZ;
            usleep(100000);
        }

        gyroXCal = xg / 100;
        gyroYCal = yg / 100;
        gyroZCal = zg / 100;

#ifdef JUNK_FOR_NOW  // Commenting out for now so can calibrate other sensors without messing up the mag calibration
        accelXCal = ax / 100;
        accelYCal = ay / 100;
        accelZCal = az / 100;

        printf("Rotate through all axes X-Y-Z\n");
        xMin = 9999;
        xMax = -9999;
        yMin = 9999;
        yMax = -9999;
        zMin = 9999;
        zMax = -9999;
        for (int i = 0; i < 100; i++)
        {
            if (magX > xMax)
	            xMax = magX;
	        if (magX < xMin)
	            xMin = magX;
            if (magY > yMax)
                yMax = magY;
            if (magY < yMin)
                yMin = magY;
            if (magZ > zMax)
                zMax = magZ;
            if (magZ < zMin)
                zMin = magZ;
            usleep(100000);
        }
#endif

        printf("Done calibration sequence.  Writing to disk.\n");
        FILE *f = fopen("./.calibration", "w");
        fprintf(f, "AX:%d\n",accelXCal);
        fprintf(f, "AY:%d\n",accelYCal);
        fprintf(f, "AZ:%d\n",accelZCal);
        fprintf(f, "GX:%d\n",gyroXCal);
        fprintf(f, "GY:%d\n",gyroYCal);
        fprintf(f, "GZ:%d\n",gyroZCal);
        fprintf(f, "MXMin:%d\n",xMin);
        fprintf(f, "MXMax:%d\n",xMax);
        fprintf(f, "MYMin:%d\n",yMin);
        fprintf(f, "MYMax:%d\n",yMax);
        fprintf(f, "MZMin:%d\n",zMin);
        fprintf(f, "MZMax:%d\n",zMax);
        fclose(f);
}

void CalculateGyroPID(ControlMode *manualControl)
{
    if (manualControl->leftMotorPower == manualControl->rightMotorPower && manualControl->leftMotorPower != 0)
    {
        SetGyro = 0;
        gyroPIDInput = gyroZ;
        gyroPID.Compute();
        printf("\n\n\ngyroPID:  %f\n\n\n",gyroPIDOutput);

        manualControl->leftMotorPower -= (gyroPIDOutput / 500);
        manualControl->rightMotorPower += (gyroPIDOutput / 500);
    }
}

// Manual control subsumption task.  If a command comes in over the FIFO, it will override all other tasks.
void ManualControl(ControlMode *manualControl)
{
    // Read from FIFO (non-blocking)
    // If command present, set active flag for manualControl and use it.  Active flag remains in effect
    // until "A" command comes in to deactivate it.
    //
    // Valid commands are:
    // M:left,right          - Motor power (left = 0-255, right = 0-255)
    // PT:panAngle,tiltAngle - Set angle for pan and tilt servos
    // A:0                   - return to autonomous mode

    char buf[256];
    int res = read(fdFifo, buf, 255);
    if (res <= 0)
    {
        manualControl->active = !autonomous;
        manualControl->leftMotorPower = manualPowerLeft;
        manualControl->rightMotorPower = manualPowerRight;
//        CalculateGyroPID(manualControl);
        return;
    }

    char msgType[8];
    char val[256];
    ParseMessage(buf, msgType, val);

    if (!strcmp(msgType, "M"))
    {
        char *leftVal = strtok(val, ",");
        char *rightVal = strtok(0, ",");
        int left = strtol(leftVal, 0, 10);
        int right = strtol(rightVal, 0, 10);
        manualPowerLeft = left;
        manualPowerRight = right;
        panServo = manualServoPan + ((manualPowerLeft - manualPowerRight) * 0.10);
        if (panServo < PAN_MIN)
            panServo = PAN_MIN;
        if (panServo > PAN_MAX)
            panServo = PAN_MAX;
        autonomous = false;
    }
    else if (!strcmp(msgType, "A"))
    {
        manualPowerLeft = 0;
        manualPowerRight = 0;
        autonomous = true;
    }
    else if (!strcmp(msgType, "PT"))
    {
        char *panVal = strtok(val, ",");
        char *tiltVal = strtok(0, ",");
        panServo = strtol(panVal, 0, 10);
        if (panServo < PAN_MIN)
            panServo = PAN_MIN;
        if (panServo > PAN_MAX)
            panServo = PAN_MAX;

        tiltServo = 180 - strtol(tiltVal, 0, 10);
        if (tiltServo < TILT_MIN)
            tiltServo = TILT_MIN;
        if (tiltServo > TILT_MAX)
            tiltServo = TILT_MAX;

        manualServoPan = panServo;
        manualServoTilt = tiltServo;
        SetServoAngle(PAN_SERVO, panServo);
        SetServoAngle(TILT_SERVO, tiltServo);
    }
    else if (!strcmp(msgType, "H"))
    {
        SetHeading = strtof(val, 0);
        autonomous = true;
    }
    else if (!strcmp(msgType, "CAL"))
    {
        CalibrateSensors();
    }
    else if (!strcmp(msgType, "RW"))
    {
	// Re-read waypoints file
	ReadWaypointsFile();
	currentWaypoint = 0;
    }
    else if (!strcmp(msgType, "WPT"))
    {
        char *valStr = strtok(val, ",");
        int val = strtol(valStr, 0, 10);
        if ((currentWaypoint + val) < waypointCount && (currentWaypoint + val) >= 0)
	    currentWaypoint += val;
    }
    manualControl->active = !autonomous;
    manualControl->leftMotorPower = manualPowerLeft;
    manualControl->rightMotorPower = manualPowerRight;
}

int leftMotorPower;
int rightMotorPower;

// Called every SUBSUMPTION_INTERVAL milliseconds, this will run each of the subsumption tasks in succession and then
// use the one with the highest priority to control the motors.
void ProcessSubsumptionTasks()
{
    ControlMode steerToHeadingControl;
    ControlMode wallFollowerControl;
    ControlMode detectObstaclesControl;
    ControlMode manualControl;


    // Run each subsumption task.
    DetectObstacles(&detectObstaclesControl);
    SteerToHeading(&steerToHeadingControl);
    WallFollower(&wallFollowerControl);
    ManualControl(&manualControl);


    // Go through the tasks from lowest to highest priority.  If a task's active flag is set, set the motor speeds
    // to that task's calculated values.  At the end, we'll have the motor speeds for the highest priority task in
    // leftMotorPower and rightMotorPower - these are the values that will get sent to the motor controller.
    leftMotorPower = NORMAL_SPEED;
    rightMotorPower = NORMAL_SPEED;
    if (steerToHeadingControl.active)
    {
        leftMotorPower = steerToHeadingControl.leftMotorPower;
        rightMotorPower = steerToHeadingControl.rightMotorPower;
        printf("SteerToHeadingControl Active\n");
        fprintf(outFile, "SteerToHeadingControl: Active\n");
    }
    else
    {
        fprintf(outFile, "SteerToHeadingControl: Inactive\n");
    }
    if (wallFollowerControl.active)
    {
        // See if we're heading back in the desired direction.  If we are, we've followed the wall
        // or obstacle around to a point where we can now continue on our desired course.
        if (fabs(SetHeading - heading) < 10)
        {
            wallFollowerControl.active = false;
            fprintf(outFile, "WallFollowerControl: Inactive\n");
        }
        else
        {
            // Otherwise, keep following the wall or obstacle
            leftMotorPower = wallFollowerControl.leftMotorPower;
            rightMotorPower = wallFollowerControl.rightMotorPower;
            printf("WallFollowerControl Active\n");
            fprintf(outFile, "WallFollowerControl: Active\n");
        }
    }
    if (0)//detectObstaclesControl.active)
    {
        printf("Obstacle detected.\n");
        fprintf(outFile, "Obstacle detected: true\n");

        if (manualControl.active && !((manualControl.leftMotorPower > 0 && manualControl.rightMotorPower < 0) ||
                                      (manualControl.leftMotorPower < 0 && manualControl.rightMotorPower > 0) ||
                                      (manualControl.leftMotorPower < 0 && manualControl.rightMotorPower < 0)))
        {
            manualControl.leftMotorPower = 0;
            manualControl.rightMotorPower = 0;
        }
        else
        {
            // TODO: Turn left or right, depending on where the obstacle was detected
            SetMotorSpeed(LEFT_MOTOR, NORMAL_SPEED);
            SetMotorSpeed(RIGHT_MOTOR, -NORMAL_SPEED);
            sleep(2);
            leftMotorPower = NORMAL_SPEED;
            rightMotorPower = NORMAL_SPEED;
            wallFollowerControl.active = true;
        }
        // TODO: Turn distance servo to 45 degrees (left or right, depending on where the obstacle was detected)
    }
    if (manualControl.active)
    {
        leftMotorPower = manualControl.leftMotorPower;
        rightMotorPower = manualControl.rightMotorPower;
        printf("ManualControl Active\n");
        fprintf(outFile, "ManualControl: Active\n");
    }
    else
    {
        fprintf(outFile, "ManualControl: Inactive\n");
    }
    if (0)
    {
        // Insert higher priority motor controls here
    }
    SetMotorSpeed(LEFT_MOTOR, leftMotorPower);
    SetMotorSpeed(RIGHT_MOTOR, rightMotorPower);
    printf("LEFT MOTOR: %d\n",leftMotorPower);
    printf("RIGHT MOTOR: %d\n",rightMotorPower);
    fprintf(outFile, "LEFT MOTOR: %d\n",leftMotorPower);
    fprintf(outFile, "RIGHT MOTOR: %d\n",rightMotorPower);
}


// Read from the serial link.  This runs in its own thread so it will continue regardless of what else is going
// on.  As we read telemetry and other sensor values from the link, update their values for other tasks to see
static void *ReadSerialThread(void *)
{
    int res;
    char buf[255];

    printf("ReadSerial thread started.\n");
    while (1)
    {
        // Read will block until characters are available on the serial line
        res = read(fdSerial, buf, 255);
        if (res > 1 && res < 20)
        {
            buf[res - 1] = '\0';
            ProcessSerialMsg(buf);
        }
    }
    return NULL;
}


void ReadCalibrationFile()
{
    FILE *f = fopen("./.calibration", "r");
    if (f == NULL)
        return;

    while (!feof(f))
    {
        char line[256];
        fgets(line, 256, f);
        char *t = strtok(line, ":");
        char *v = strtok(NULL, ":");
        if (t == NULL || v == NULL || strlen(t) == 0 || strlen(v) == 0)
            break;
        if (!strcmp(t, "AX"))
        {
            accelXCal = strtol(v, 0, 10);
        }
        if (!strcmp(t, "AY"))
        {
            accelYCal = strtol(v, 0, 10);
        }
        if (!strcmp(t, "AZ"))
        {
            accelZCal = strtol(v, 0, 10);
        }
        if (!strcmp(t, "GX"))
        {
            gyroXCal = strtol(v, 0, 10);
        }
        if (!strcmp(t, "GY"))
        {
            gyroYCal = strtol(v, 0, 10);
        }
        if (!strcmp(t, "GZ"))
        {
            gyroZCal = strtol(v, 0, 10);
        }
        if (!strcmp(t, "MXMin"))
        {
            xMin = strtol(v, 0, 10);
        }
        if (!strcmp(t, "MXMax"))
        {
            xMax = strtol(v, 0, 10);
        }
        if (!strcmp(t, "MYMin"))
        {
            yMin = strtol(v, 0, 10);
        }
        if (!strcmp(t, "MYMax"))
        {
            yMax = strtol(v, 0, 10);
        }
        if (!strcmp(t, "MZMin"))
        {
            zMin = strtol(v, 0, 10);
        }
        if (!strcmp(t, "MZMax"))
        {
            zMax = strtol(v, 0, 10);
        }
    }
    fclose(f);
}

gps_data_t gpsData;

static void *GpsThread(void *)
{
    // Init libgps so we can get gps data
    // connect to GPSd
    while (gps_open("localhost", "2947", &gpsData) < 0)
    {
        fprintf(stderr,"Could not connect to GPSd\n");
	sleep(10);
    }

    //register for updates
    gps_stream(&gpsData, WATCH_ENABLE | WATCH_JSON, NULL);

    int nofixCount = 0;
    while (1) //gpsData.status==0)
    {
        //block for up to 0.5 seconds
        if (gps_waiting(&gpsData, 500))
        {
            //dunno when this would happen but its an error
            if(gps_read(&gpsData) == -1)
            {
//                fprintf(stderr,"GPSd Error\n");
		latitude = 0.0;
		longitude = 0.0;
		latitude_error = 0;
		longitude_error = 0;
		gps_fix = MODE_NO_FIX;
            }
            else
            {
                //status>0 means you have data
                if(gpsData.status > 0)
                {
                    //sometimes if your GPS doesnt have a fix, it sends you data anyway
                    //the values for the fix are NaN. this is a clever way to check for NaN.
                    if(gpsData.fix.longitude != gpsData.fix.longitude || gpsData.fix.altitude != gpsData.fix.altitude)
                    {
//                        fprintf(stderr,"Could not get a GPS fix.\n");
			latitude = 0.0;
			longitude = 0.0;
			latitude_error = 0;
			longitude_error = 0;
			gps_fix = MODE_NO_FIX;
                    }
                    //otherwise you have a legitimate fix!
                    else
                    {
			latitude = gpsData.fix.latitude;
			longitude = gpsData.fix.longitude;

			latitude_error = gpsData.fix.epy;
			longitude_error = gpsData.fix.epx;
			gps_fix = gpsData.fix.mode;
			nofixCount = 0;

			FILE *telemetry = fopen("./telemetry.dat","w");
			if (telemetry)
			{
			    // Lat | Long | Lat_accuracy | Long_accuracy | waypoint_lat | waypoint_long
			    fprintf(telemetry, "%f|%f|%d|%d|%f|%f\n", latitude, longitude, (int)latitude_error, (int)longitude_error, waypoints[currentWaypoint].latitude, waypoints[currentWaypoint].longitude);
			    fclose(telemetry);
			}
                    }
                }
            }
        }
        //apparently gps_stream disables itself after a few seconds.. in this case, gps_waiting returns false.
        //we want to re-register for updates and keep checking we dont have a fix yet.
        else
	{
            gps_stream(&gpsData, WATCH_ENABLE | WATCH_JSON, NULL);
	    nofixCount++;
	}

	// If we've been without a fix for 10 iterations (10 seconds), set the fix mode to NO_FIX
	if (nofixCount > 10)
	    gps_fix = MODE_NO_FIX;


	// Sleep so we don't kill the CPU.  Updates every second shuold be enough
	usleep(100000);
    }

    // Should never get here
    return NULL;
}

void *IMUThread(void *)
{
   int fd;
   short ax,ay,az;
   short gx,gy,gz;
   short mx,my,mz;

   float rate_gyr_x(0.0),rate_gyr_y(0.0),rate_gyr_z(0.0);
   float gyroXangle(0.0),gyroYangle(0.0),gyroZangle(0.0);

   float AccYangle(0.0),AccXangle(0.0);

   float CFangleX = 0.0;
   float CFangleY = 0.0;
   if(!init(fd))
         exit(1);
   usleep(100000);

   int samples = 0;
   int accX[CAL_SAMPLES];
   int accY[CAL_SAMPLES];
   int accZ[CAL_SAMPLES];
   int accOffsetX = 0;
   int accOffsetY = 0;
   int accOffsetZ = 0;


   long impactTimer = millis();
   while(readADXL345(fd,ax,ay,az) && readL3G4200D(fd,gx,gy,gz) && readHMC5883L(fd,mx,my,mz))
   {
        // Swap the x and y axes around because of sensor orientation
        short tmp = my;
        my = mx;
        mx = tmp;
        my = -my;
///        mz = -mz; // ???
//      mx = -mx;

        // ??? Negate the X and Y axes if the accelerometer is upside-down
//      ax = -ax;
//      ay = -ay;
//      az = -az;

        // ??? Negate the Y axis if magnetometer is upside-down
//      mx = -mx;
///     my = -my;
//      mz = -mz;


// Offsets for az inverted
//      ax -= 32;
//      ay += 9;
//      az -= 26;

// Offsets for ax,ay,az unchanged
//      ax -= 35;
//      ay += 10;
//      az += 26;

// Offsets for ax,az unchanged, ay inverted
//      ax -= 35;
//      ay -= 10;
//      az += 26;

	// Invert the gyro X axis
	gx = -gx;

	// Apply the calibration values for the accelerometers
      if (samples > CAL_SAMPLES)
      {
          ax -= accOffsetX;
          ay -= accOffsetY;
          az -= (accOffsetZ - 256);
      }


        // We get bogus readings from the magnetometer sometimes.  I don't know why.  Skip this iteration if the readings are way out of whack
	if (mx > -1000 && mx < 1000 && my > -1000 && my < 1000 && mz > -1000 && mz < 1000)
	{
	magX = mx;
	magY = my;
	magZ = mz;
	accelX = ax;
	accelY = ay;
	accelZ = az;
	gyroX = gx;
	gyroY = gy;
	gyroZ = gz;
	}
	else
	{
	    // Got a bad reading from the magnetometer - skip this time through the loop and get it right next time
	    usleep(20000);
	    continue;
	}

//	complexHeading = getHeadingNewNew();



      //Convert Gyro raw to degrees per second
      rate_gyr_x = (float)gx * G_GAIN;
      rate_gyr_y = (float)gy * G_GAIN;
      rate_gyr_z = (float)gz * G_GAIN;

      //Calculate the angles from the gyros
      gyroXangle += rate_gyr_x*DT;
      gyroYangle += rate_gyr_y*DT;
      gyroZangle += rate_gyr_z*DT;

      //Convert Accelerometer values to degrees
      AccXangle = (float) (atan2(ay,az)/*+M_PI*/)*RAD_TO_DEG;
      AccYangle = (float) (atan2(ax,az)/*+M_PI*/)*RAD_TO_DEG;


      //Change the rotation value of the accelerometer to -/+ 180
      if (AccXangle >180)
      {
          AccXangle -= (float)360.0;
      }
      if (AccYangle >180)
          AccYangle -= (float)360.0;

//      AccYangle = -AccYangle;


      // Complementary filter used to combine the accelerometer and gyro values.
      CFangleX=AA*(CFangleX+rate_gyr_x*DT) +(1 - AA) * AccXangle;
      CFangleY=AA*(CFangleY+rate_gyr_y*DT) +(1 - AA) * AccYangle;



      // Get the compass heading based on the roll and pitch angles and the magnetometer readings
///      heading = getIMUHeading(CFangleX, CFangleY, mx, my, mz);

       // Hard iron compensation
       float cx = mx - (xMin + xMax) / 2;
       float cy = my - (yMin + yMax) / 2;
       float cz = mz - (zMin + zMax) / 2;

       heading = getHeading(ax, ay, az, cy, cx, -cz, gx, gy, gz, 180, 0);


    rollAngle = CFangleX;
    pitchAngle = CFangleY;

    // Detect impacts
    if (ouchEnabled && (abs(gx) > 300 || abs(gy) > 300 || abs(gz) > 300))
    {
	// Only if it's been more than 10 seconds since the last impact, and we're not currently moving
	if (millis() - impactTimer > 5000 && leftMotorPower == 0 && rightMotorPower == 0)
	{
	    pid_t child_PID;
	    child_PID = fork();
	    if(child_PID >= 0)
	    {
		if(child_PID == 0)
		{
		    // Child process
		    printf("Child process! PID=%d, Parent PID=%d\n", getpid(), getppid());
	    	    execl("/usr/bin/espeak", "/usr/bin/espeak", "Ouch.", (char *)0);
		    _exit(0);
		}
		else
	        {
	            int status;
	            waitpid(child_PID, &status, 0);
	        }
	   }
	   impactTimer = millis();
	}
    }

//      printf ("   GyroX  %7.3f \t AccXangle \e[m %7.3f \t \033[22;31mCFangleX %7.3f\033[0m\t GyroY  %7.3f \t AccYangle %7.3f \t \033[22;36mCFangleY %7.3f\t\033[0m Heading %7.3f gx=%d gy=%d gz=%d\n",gyroXangle,AccXangle,CFangleX,gyroYangle,AccYangle,CFangleY, heading, gx, gy, gz);
//      printf ("   GyroX  %7.3f \t AccXangle \e[m %7.3f \t \033[22;31mCFangleX %7.3f\033[0m\t GyroY  %7.3f \t AccYangle %7.3f \t \033[22;36mCFangleY %7.3f\t\033[0m Heading %7.3f mx=%d my=%d mz=%d\n",gyroXangle,AccXangle,CFangleX,gyroYangle,AccYangle,CFangleY, heading, mx, my, mz);
//      printf("AccX Raw: %d \t AccY Raw: %d \t AccZ raw: %d\n", ax, ay, az);
//      printf("MagX Raw: %d \t MagY Raw: %d \t MagZ Raw: %d \t xmin: %d \t xmax: %d \t ymin: %d \t ymax: %d \t zmin: %d \t zmax: %d\n", mx, my, mz, xmin, xmax, ymin, ymax, zmin, zmax);
      usleep(20000);

      // Do this every 200 milliseconds or something
//      gyroXangle = 0.0;
//      gyroYangle = 0.0;
//      gyroZangle = 0.0;

      // Gather 10 samples from each accelerometer axis and average them - X and Y should average to 0, Z should average to 256
      // Adjust the offsets accordingly
      if (samples < CAL_SAMPLES)
      {
        accX[samples] = ax;
        accY[samples] = ay;
        accZ[samples] = az;
        samples++;
      }
      if (samples == CAL_SAMPLES)
      {
        int total = 0;
        for (int n = 0; n < CAL_SAMPLES; n++)
           total += accX[n];
        accOffsetX = total / CAL_SAMPLES;

        total = 0;
        for (int n = 0; n < CAL_SAMPLES; n++)
           total += accY[n];
        accOffsetY = total / CAL_SAMPLES;

        total = 0;
        for (int n = 0; n < CAL_SAMPLES; n++)
           total += accZ[n];
        accOffsetZ = total / CAL_SAMPLES;

        samples++;
      }
   }
   return 0;
}


void ReadWaypointsFile()
{
    FILE *waypointFile = fopen("waypoints.dat", "r");
    if (waypointFile == 0)
	return;

    waypointCount = 0;
    while (!feof(waypointFile))
    {
	char line[256];
	fgets(line, 256, waypointFile);
	const char *wpLat = strtok(line, "|");
	const char *wpLong = strtok(0, "|");
	if (wpLat && wpLong)
	{
	    GeoCoordinate waypoint(wpLat, wpLong);
	    waypoints[waypointCount] = waypoint;
	    waypointCount++;
	}
    }
    fclose(waypointFile);
}


int calFlag = 0;
int waypointFlag = 1;  // default
int headingFlag = 0;
int led = 0;
int main(int argc, char **argv)
{
    char serialDev[256];
    strcpy(serialDev, DEFAULTDEVICE);

    opterr = 0;
    int c;
    while ((c = getopt (argc, argv, "cwh:d:")) != -1)
    {
        switch (c)
        {
            case 'c':
                calFlag = 1;
                break;
            case 'h':
                headingFlag = 1;
                SetHeading = strtol(optarg, 0, 10);
                waypointFlag = 0;
                break;
            case 'w':
                waypointFlag = 1;
                break;
            case 'd':
                strcpy(serialDev, optarg);
                break;
            case '?':
                if (optopt == 'h')
                    fprintf (stderr, "Option -%c requires an argument.\n", optopt);
                else if (isprint (optopt))
                    fprintf (stderr, "Unknown option `-%c'.\n", optopt);
                else
                    fprintf (stderr, "Unknown option character `\\x%x'.\n", optopt);
                return 1;
            default:
                abort ();
        }
    }




//    if (argc == 2)
//        strcpy(serialDev, argv[1]);
//    else
//        strcpy(serialDev, DEFAULTDEVICE);


    Setup(&fdSerial, serialDev);
    fdFifo = open("./RobotFifo", O_RDONLY | O_NONBLOCK);

    // Default calibration.  We'll use these until the complex heading calculation routine has been called with enough
    // variation to do a real calibration
    // Commented out.  Read from .calibration file on disk now (ReadCalibrationFile()).
//    xMin = -735;
//    xMax = 420;
//    yMin = -700;
//    yMax = 380;
//    zMin = -450;
//    zMax = 100;

/// Will need to put these back
///    InitMagCalibrationData();
///    ReadCalibrationFile();

    printf("Starting IMU thread\n");
    pthread_t imuThreadId;
    pthread_create(&imuThreadId, NULL, IMUThread, NULL);

    printf("Starting GPS thread\n");
    pthread_t gpsThreadId;
    pthread_create(&gpsThreadId, NULL, GpsThread, NULL);

    printf("Starting ReadSerial thread\n");
    pthread_t threadId;
    pthread_create(&threadId, NULL, ReadSerialThread, NULL);
    sleep(2);

    if (calFlag)
    {
        CalibrateSensors();
    }

    printf("Starting main loop\n");
    unsigned long lastLEDMillis = 0;
    unsigned long lastSubMillis = 0;
    unsigned long lastGPSMillis = 0;
    unsigned long motorsOffMillis = 0;
    bool motorsRunning = false;

    ReadWaypointsFile();

    // The main loop.  Run forever.
    while (1)
    {
        unsigned long loopTime = millis();


        if ((millis() - lastSubMillis > SUBSUMPTION_INTERVAL))
        {
            outFile = fopen("./data.log", "w");
            ProcessSubsumptionTasks();

	    printf("Complex Heading: %3.2f\n", complexHeading);
	    printf("Waypoint Range: %1.1fm\n", waypointRange * 1000);
            printf("Pan Servo: %d\n", panServo);
            printf("Tilt Servo: %d\n", tiltServo);
            printf("MagX: %d\n", magX);
            printf("MagY: %d\n", magY);
            printf("MagZ: %d\n", magZ);
            printf("AccelX: %f\n", accelX);
            printf("AccelY: %f\n", accelY);
            printf("AccelZ: %f\n", accelZ);
            printf("GyroX: %d\n", gyroX);
            printf("GyroY: %d\n", gyroY);
            printf("GyroZ: %d\n", gyroZ);
            printf("gyroDeltaT: %d\n", gyroDeltaT);
	    char gps_fix_mode[32];
	    switch (gps_fix)
	    {
		case MODE_NOT_SEEN: strcpy(gps_fix_mode, "NOT SEEN"); break;
		case MODE_NO_FIX: strcpy(gps_fix_mode, "NO FIX"); break;
		case MODE_2D: strcpy(gps_fix_mode, "2D"); break;
		case MODE_3D: strcpy(gps_fix_mode, "3D"); break;
		default: strcpy(gps_fix_mode, "UNKNOWN"); break;
	    }
	    char gps_status[32];
	    switch (gpsData.status)
	    {
		case STATUS_NO_FIX: strcpy(gps_status, "NO FIX"); break;
		case STATUS_FIX: strcpy(gps_status, "FIX - NO DGPS"); break;
		case STATUS_DGPS_FIX: strcpy(gps_status, "FIX - DGPS"); break;
		default: strcpy(gps_status, "Unknown"); break;
 	    }
	    printf("GPS STATUS: %s\n", gps_status);
	    printf("    Fix Mode: %s\n", gps_fix_mode);
            printf("    Latitude: %f\n", latitude);
            printf("    Longitude: %f\n", longitude);
	    printf("    Latitude Accuracy: %dm\n", (int)latitude_error);
	    printf("    Longitude accuracy: %dm\n", (int)longitude_error);
            printf("Distance1: %d\n", distance1);
            printf("Distance2: %d\n", distance2);
            printf("LED: %s\n\n", led ? "ON" : "OFF");

	    fprintf(outFile, "Waypoint Range: %1.1fm\n", waypointRange * 1000);
            fprintf(outFile, "Pan Servo: %d\n", panServo);
            fprintf(outFile, "Tilt Servo: %d\n", tiltServo);
//            fprintf(outFile, "MagX: %d<br/>\n", magX);
//            fprintf(outFile, "MagY: %d<br/>\n", magY);
//            fprintf(outFile, "MagZ: %d<br/>\n", magZ);
//            fprintf(outFile, "AccelX: %f<br/>\n", accelX);
//            fprintf(outFile, "AccelY: %f<br/>\n", accelY);
//            fprintf(outFile, "AccelZ: %f<br/>\n", accelZ);
//            fprintf(outFile, "GyroX: %d<br/>\n", gyroX);
//            fprintf(outFile, "GyroY: %d<br/>\n", gyroY);
//            fprintf(outFile, "GyroZ: %d<br/>\n", gyroZ);
//            fprintf(outFile, "gyroDeltaT: %d<br/>\n", gyroDeltaT);
	    fprintf(outFile, "GPS STATUS: %s\n", gps_status);
	    fprintf(outFile, "Fix Mode: %s\n", gps_fix_mode);
            fprintf(outFile, "Latitude: %f\n", latitude);
            fprintf(outFile, "Longitude: %f\n", longitude);
	    fprintf(outFile, "Latitude Accuracy: %dm\n", (int)latitude_error);
	    fprintf(outFile, "Longitude accuracy: %dm\n", (int)longitude_error);
            fprintf(outFile, "Distance1: %d\n", distance1);
            fprintf(outFile, "Distance2: %d\n", distance2);
            fprintf(outFile, "LED: %s\n", led ? "ON" : "OFF");

            lastSubMillis = millis();
            fclose(outFile);
        }

	if (leftMotorPower == 0 && rightMotorPower == 0)
	{
	    if (motorsRunning)
	    {
		// Motors just stopped - set the timer
		motorsOffMillis = millis();
	    }
	    if (millis() - motorsOffMillis > 1000)
		ouchEnabled = true;
	}
	else
	    ouchEnabled = false;

	motorsRunning = leftMotorPower && rightMotorPower;

        if (waypointFlag && millis() - lastGPSMillis > CALCULATE_GPS_HEADING_INTERVAL)
        {
            CalculateHeadingToWaypoint();
	    CalculateDistanceToWaypoint();
            lastGPSMillis = millis();
        }

        char msg[256];
        if (led == 0 && millis() - lastLEDMillis > LED_BLINK_INTERVAL)
        {
            sprintf(msg, "LED:1\r");
            write(fdSerial, msg, strlen(msg));
            lastLEDMillis = millis();
	        led = 1;
        }
        if (led == 1 && millis() - lastLEDMillis > LED_BLINK_INTERVAL)
        {
            sprintf(msg, "LED:0\r");
            write(fdSerial, msg, strlen(msg));
            lastLEDMillis = millis();
	        led = 0;
        }
        unsigned long now = millis();
    	if (now - loopTime < 1)
            usleep((1 - (now - loopTime)) * 1000);
    }
}



