#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <iostream>
#include <cmath>
#include "imu.h"
using namespace std;

float G_GAIN = 0.07; //0.00875; //0.07;
float DT = 0.02; // [s/loop] loop period. 20ms
//#define M_PI = 3.14159265358979323846;
float RAD_TO_DEG = 57.29578;
float AA = 0.98; // complementary filter constant
const int CAL_SAMPLES = 10;

float fusionGetOrientation(float ax, float ay, float az, float mx, float my, float mz, float gx, float gy, float gz, int offset, int n);

// xmin: -772      xmax: -1        ymin: 208       ymax: 543       zmin: -445      zmax: 432
#ifdef JUNK
const short xMin = -772;
const short xMax = -1;
const short yMin = 208;
const short yMax = 543;
const short zMin = -445;
const short zMax = 432;
// X and Y swapped
const short yMin = 1;
const short yMax = 772;
const short xMin = -543;
const short xMax = -208;
const short zMin = -432;
const short zMax = 445;
#endif
// xmin: -422      xmax: 482       ymin: -225      ymax: 819       zmin: 215       zmax: 555
#ifdef USE_THESE
const short xMin = -422;
const short xMax = 482;
const short yMin = -225;
const short yMax = 819;
const short zMin = 215;
const short zMax = 555;
#endif
// xmin: -321      xmax: 251       ymin: 3         ymax: 589       zmin: 447       zmax: 542
// xmin: -318      xmax: 252       ymin: 3         ymax: 593       zmin: -512      zmax: 770
const short xMin = 0;// -321;
const short xMax = 0;// 251;
const short yMin = 0;// 3;
const short yMax = 0;// 589;
const short zMin = 0;// -527;
const short zMax = 0;// 486; //532;

//xmin: -635      xmax: 467       ymin: -385      ymax: 820       zmin: -527      zmax: 486



float getHeading(float rollAngle, float pitchAngle, short mx, short my, short mz, short ax, short ay, short az)
{
    // Normalize raw magnetometer data to a unit circle with the calibraton limits
///    float cx = (float)(mx-xMin)/(float)(xMax-xMin) - 0.5;
///    float cy = (float)(my-yMin)/(float)(yMax-yMin) - 0.5;
///    float cz = (float)(mz-zMin)/(float)(zMax-zMin) - 0.5;
//	float cx = mx;
//	float cy = my;
//	float cz = mz;
    // Hard iron compensation
    float cx = (float)mx - (float)(xMin + xMax) / 2;
    float cy = (float)my - (float)(yMin + yMax) / 2;
    float cz = (float)mz - (float)(zMin + zMax) / 2;

//  FILE *f = fopen("cal.dat","a");
//  fprintf(f,"%3.2f,%3.2f,%3.2f\n",cx,cy,cz);
//  fclose(f);

    // Soft iron compensation
//    cx = (float)(mx-xMin)/(float)(xMax-xMin) * 2 - 1.0;
//   cy = (float)(my-yMin)/(float)(yMax-yMin) * 2 - 1.0;
//    cz = (float)(mz-zMin)/(float)(zMax-zMin) * 2 - 1.0;


	// Derive unit vector for the mag vector
	cx = cx / fabs(xMax - xMin);
	cy = cy / fabs(yMax - yMin);
	cz = cz / fabs(zMax - zMin);




	// Swap roll and pitch angles.  Invert pitch angle.
	float tmp = rollAngle;
	rollAngle = pitchAngle; //+pitchAngle?
	pitchAngle = tmp;

	printf("PitchAngle = %f\trollAngle = %f", pitchAngle, rollAngle);

   // Convert rollAngle and pitchAngle to radians
    rollAngle *= (3.14159265 / 180);
    pitchAngle *= (3.14159265 / 180);




#ifdef MAYBE
  float cosRoll = (float)cos(rollAngle);
  float sinRoll = (float)sin(rollAngle);
  float cosPitch = (float)cos(-1*pitchAngle);
  float sinPitch = (float)sin(-1*pitchAngle);

  /* The tilt compensation algorithm                            */
  /* Xh = X.cosPitch + Z.sinPitch                               */
  /* Yh = X.sinRoll.sinPitch + Y.cosRoll - Z.sinRoll.cosPitch   */
  float mag_X = mx * cosPitch + mz * sinPitch;
  float mag_Y = mx * sinRoll * sinPitch + my * cosRoll - mz * sinRoll * cosPitch;
  float yawAngle = atan2(mag_Y, mag_X);
    // Convert to degrees
    yawAngle = yawAngle * (180 / 3.14159265); //RADTODEG; //180 / PI;

    // Factor in the declination angle for this location  (from http://www.ngdc.noaa.gov/geomag-web/#declination)
    float declinationAngle = -0.20780084;  // should this be negative?
    yawAngle += declinationAngle * 180 / 3.14159265;


    // Convert to range (0, 360)
    yawAngle = (yawAngle > 0.0 ? yawAngle : (360.0 + yawAngle));
    return yawAngle;
#endif


    // Now use trigonometry to project the magnetic unit circle onto a 2D X-Y plane oriented at the calculated angles.
    // X and Y will be the point on the unit circle that represents our compass heading.
//    float y = (cz * sin(rollAngle)) - (cy * cos(rollAngle));
//    float x = (cx * cos(pitchAngle))
//                     + (cy * sin(pitchAngle) * sin(rollAngle))
//                     + (cz * sin(pitchAngle) * cos(rollAngle));
    float x = (float)cx * cos(pitchAngle) + (float)cz * sin(pitchAngle);
    float y = (float)cx * sin(rollAngle) * sin(pitchAngle) + (float)cy * cos(rollAngle) - (float)cz * sin(rollAngle) * cos(pitchAngle);
//    float y = cy * cos(pitchAngle) + cz * sin(pitchAngle);
//    float x = cy * sin(rollAngle) * sin(pitchAngle) + cx * cos(rollAngle) - cz * sin(rollAngle) * cos(pitchAngle);


    // Find the compass heading for the calculated x and y
    float yawAngle = atan2(y, x);
//    float yawAngle = atan2(cy, cx);

    // Off by 90 degrees because of sensor orientation?
      yawAngle += 3.14159265 / 2;


    // Convert to degrees
    yawAngle = yawAngle * (180 / 3.14159265); //RADTODEG; //180 / PI;

    // Factor in the declination angle for this location  (from http://www.ngdc.noaa.gov/geomag-web/#declination)
    float declinationAngle = -0.20780084;  // should this be negative?
    yawAngle += declinationAngle * 180 / 3.14159265;


    // Convert to range (0, 360)
    yawAngle = (yawAngle > 0.0 ? yawAngle : (360.0 + yawAngle));
    return yawAngle;
}

short xmin = 32000;
short xmax = -32000;
short ymin = 32000;
short ymax = -32000;
short zmin = 32000;
short zmax = -32000;

int main(int argc, char **argv)
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
   usleep(200000);

   int samples = 0;
   int accX[CAL_SAMPLES];
   int accY[CAL_SAMPLES];
   int accZ[CAL_SAMPLES];
   int accOffsetX = 0;
   int accOffsetY = 0;
   int accOffsetZ = 0;
   while(readADXL345(fd,ax,ay,az) && readL3G4200D(fd,gx,gy,gz) && readHMC5883L(fd,mx,my,mz))
   {
	// Swap the x and y axes around because of sensor orientation
	short tmp = my;
	my = mx;
	mx = tmp;
	my = -my;
///	mz = -mz; // ???
//	mx = -mx;

	// ??? Negate the X and Y axes if the accelerometer is upside-down
//	ax = -ax;
//	ay = -ay;
//	az = -az;

	// ??? Negate the Y axis if magnetometer is upside-down
//	mx = -mx;
///	my = -my;
//	mz = -mz;

// Offsets for az inverted
//      ax -= 32;
//      ay += 9;
//      az -= 26;

// Offsets for ax,ay,az unchanged
//	ax -= 35;
//	ay += 10;
//	az += 26;

// Offsets for ax,az unchanged, ay inverted
//	ax -= 35;
//	ay -= 10;
//	az += 26;


	// Reverse the gyro's X axis
	gx = -gx;

      if (samples > CAL_SAMPLES)
      {
          ax -= accOffsetX;
          ay -= accOffsetY;
          az -= (accOffsetZ - 256);
      }


	// We get bogus readings from the magnetometer sometimes.  I don't know why.  Skip this iteration if the readings are way out of whack
//	if (mx > xMax || mx < xMin || my > yMax || my < yMin || mz > zMax || mz < zMin)
//		continue;

      //Convert Gyro raw to degrees per second
      rate_gyr_x = (float)gx * G_GAIN;
      rate_gyr_y = (float)gy * G_GAIN;
      rate_gyr_z = (float)gz * G_GAIN;

      //Calculate the angles from the gyro
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

//	AccYangle = -AccYangle;


      // Complementary filter used to combine the accelerometer and gyro values.
      CFangleX=AA*(CFangleX+rate_gyr_x*DT) +(1 - AA) * AccXangle;
      CFangleY=AA*(CFangleY+rate_gyr_y*DT) +(1 - AA) * AccYangle;



#ifdef ZZZ
 mx = (float)mx - (float)(xMin + xMax) / 2;
 my = (float)my - (float)(yMin + yMax) / 2;
 mz = (float)mz - (float)(zMin + zMax) / 2;
	float heading = fusionGetOrientation(ax, ay, az, my, mx, mz, gx, gy, gz, 180, 27);  // <<<--------
#endif


//      float heading = getHeading(CFangleX, CFangleY, mx, my, mz, ax, ay, az);

#ifdef JUNK
//1:  Heading: -78.60      Pitch: -0.67    Roll: -0.22 (add 180 degrees)
//2:  Heading: -80.43      Pitch: -0.67    Roll: -0.22 (add 180 degrees)
//7:  Heading: -77.40      Pitch: 0.22     Roll: 0.67 (add 180 degrees)
//8:  Heading: -79.44      Pitch: 0.22     Roll: 0.67 (add 180 degrees)
//15:  Heading: 11.73      Pitch: -0.00    Roll: -0.22
//17:  Heading: 9.87       Pitch: -0.00    Roll: -0.22
//20:  Heading: -80.22     Pitch: 0.22     Roll: 0.67 (add 180 degrees)
//25:  Heading: -77.18     Pitch: -0.67    Roll: -0.22 (add 180 degrees)
//31:  Heading: 101.73     Pitch: 0.22     Roll: 0.00
//35:  Heading: 10.12      Pitch: 0.22     Roll: 0.00
//38:  Heading: 100.12     Pitch: -0.00    Roll: -0.22
//39:  Heading: 12.14      Pitch: -0.00    Roll: -0.22
//49:  Heading: 101.85     Pitch: -0.00    Roll: -0.22
//50:  Heading: 99.98      Pitch: -0.00    Roll: -0.22

 mx = (float)mx - (float)(xMin + xMax) / 2;
 my = (float)my - (float)(yMin + yMax) / 2;
 mz = (float)mz - (float)(zMin + zMax) / 2;

	fusionGetOrientation(ax, ay, az, mx, my, mz, 0, 1);
	fusionGetOrientation(ax, ay, az, mx, mz, my, 180, 2);
	fusionGetOrientation(ax, ay, az, my, mx, mz, 180, 3);
	fusionGetOrientation(ax, ay, az, my, mz, mx, 0, 4);
	fusionGetOrientation(ax, ay, az, mz, mx, my, 0, 5);
	fusionGetOrientation(ax, ay, az, mz, my, mx, 0, 6);

	fusionGetOrientation(ay, ax, az, mx, my, mz, 0, 7);
	fusionGetOrientation(ay, ax, az, mx, mz, my, 180, 8);
	fusionGetOrientation(ay, ax, az, my, mx, mz, 180, 9);
	fusionGetOrientation(ay, ax, az, my, mz, mx, 0, 10);
	fusionGetOrientation(ay, ax, az, mz, mx, my, 0, 11);
	fusionGetOrientation(ay, ax, az, mz, my, mx, 0, 12);

	fusionGetOrientation(ax, ay, az, -mx, my, mz, 0, 13);
	fusionGetOrientation(ax, ay, az, -mx, mz, my, 0, 14);
	fusionGetOrientation(ax, ay, az, my, -mx, mz, 0, 15);
	fusionGetOrientation(ax, ay, az, my, mz, -mx, 0, 16);
	fusionGetOrientation(ax, ay, az, mz, -mx, my, 90, 17);
	fusionGetOrientation(ax, ay, az, mz, my, -mx, 0, 18);

	fusionGetOrientation(ay, ax, az, mx, -my, mz, 0, 19);
	fusionGetOrientation(ay, ax, az, mx, mz, -my, 180, 20);
	fusionGetOrientation(ay, ax, az, -my, mx, mz, 0, 21);
	fusionGetOrientation(ay, ax, az, -my, mz, mx, 0, 22);
	fusionGetOrientation(ay, ax, az, mz, mx, -my, 0, 23);
	fusionGetOrientation(ay, ax, az, mz, -my, mx, 90, 24);

	fusionGetOrientation(ax, ay, az, mx, my, -mz, 0, 25);
	fusionGetOrientation(ax, ay, az, mx, -mz, my, 0, 26);
	fusionGetOrientation(ax, ay, az, my, mx, -mz, 180, 27);  // <<<--------
	fusionGetOrientation(ax, ay, az, my, -mz, mx, 0, 28);
	fusionGetOrientation(ax, ay, az, -mz, mx, my, 0, 29);
	fusionGetOrientation(ax, ay, az, -mz, my, mx, 0, 30);

	fusionGetOrientation(ay, ax, az, -mx, -my, mz, 0, 31);
	fusionGetOrientation(ay, ax, az, -mx, mz, -my, 0, 32);
	fusionGetOrientation(ay, ax, az, -my, -mx, mz, 0, 33);
	fusionGetOrientation(ay, ax, az, -my, mz, -mx, 0, 34);
	fusionGetOrientation(ay, ax, az, mz, -mx, -my, 90, 35);
	fusionGetOrientation(ay, ax, az, mz, -my, -mx, 90, 36);

	fusionGetOrientation(ax, ay, az, -mx, my, -mz, 0, 37);
	fusionGetOrientation(ax, ay, az, -mx, -mz, my, 0, 38);
	fusionGetOrientation(ax, ay, az, my, -mx, -mz, 0, 39);
	fusionGetOrientation(ax, ay, az, my, -mz, -mx, 0, 40);
	fusionGetOrientation(ax, ay, az, -mz, -mx, my, 0, 41);
	fusionGetOrientation(ax, ay, az, -mz, my, -mx, 0, 42);

	fusionGetOrientation(ay, ax, az, mx, -my, -mz, 0, 43);
	fusionGetOrientation(ay, ax, az, mx, -mz, -my, 0, 44);
	fusionGetOrientation(ay, ax, az, -my, mx, -mz, 0, 45);
	fusionGetOrientation(ay, ax, az, -my, -mz, mx, 0, 46);
	fusionGetOrientation(ay, ax, az, -mz, mx, -my, 0, 47);
	fusionGetOrientation(ay, ax, az, -mz, -my, mx, 0, 48);

	fusionGetOrientation(ax, ay, az, -mx, -my, -mz, 0, 49);
	fusionGetOrientation(ax, ay, az, -mx, -mz, -my, 0, 50);
	fusionGetOrientation(ax, ay, az, -my, -mx, -mz, 0, 51);
	fusionGetOrientation(ax, ay, az, -my, -mz, -mx, 0, 52);
	fusionGetOrientation(ax, ay, az, -mz, -mx, -my, 0, 53);
	fusionGetOrientation(ax, ay, az, -mz, -my, -mx, 0, 54);


	printf("\n");
#endif

	if (mx < xmin && mx > -1000)
		xmin = mx;
	if (mx > xmax && mx < 1000)
		xmax = mx;
	if (my < ymin && my > -1000)
		ymin = my;
	if (my > ymax && my < 1000)
		ymax = my;
	if (mz < zmin && mz > -1000)
		zmin = mz;
	if (mz > zmax && mz < 1000)
		zmax = mz;

//      printf ("   GyroX  %7.3f \t AccXangle \e[m %7.3f \t \033[22;31mCFangleX %7.3f\033[0m\t GyroY  %7.3f \t AccYangle %7.3f \t \033[22;36mCFangleY %7.3f\t\033[0m Heading %7.3f gx=%d gy=%d gz=%d\n",gyroXangle,AccXangle,CFangleX,gyroYangle,AccYangle,CFangleY, heading, gx, gy, gz);
///      printf ("   GyroX  %7.3f \t AccXangle \e[m %7.3f \t \033[22;31mCFangleX %7.3f\033[0m\t GyroY  %7.3f \t AccYangle %7.3f \t \033[22;36mCFangleY %7.3f\t\033[0m Heading %7.3f mx=%d my=%d mz=%d\n",gyroXangle,AccXangle,CFangleX,gyroYangle,AccYangle,CFangleY, heading, mx, my, mz);
//      printf("AccX Raw: %d \t AccY Raw: %d \t AccZ raw: %d\n", ax, ay, az);
      printf("MagX Raw: %d \t MagY Raw: %d \t MagZ Raw: %d \t xmin: %d \t xmax: %d \t ymin: %d \t ymax: %d \t zmin: %d \t zmax: %d\n", mx, my, mz, xmin, xmax, ymin, ymax, zmin, zmax);
      usleep(10000);

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



float fusionGetOrientation(float ax, float ay, float az, float mx, float my, float mz, float gx, float gy, float gz, int offset, int n)
{
   static float rate_gyr_x(0.0),rate_gyr_y(0.0),rate_gyr_z(0.0);
   static float gyroXangle(0.0),gyroYangle(0.0),gyroZangle(0.0);

   static float AccYangle(0.0),AccXangle(0.0);

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


  pitch = -pitch;

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

//  float heading = (float)atan2(mz * sin(roll) - my * cos(roll), \
//                                      mx * cos(pitch) + \
//                                      my * sin(pitch) * sin(roll) + \
//                                      mz * sin(pitch) * cos(roll));
  float heading = (float)atan2(mz * sin(CFangleXRad) - my * cos(CFangleXRad), \
                                      mx * cos(CFangleYRad) + \
                                      my * sin(CFangleYRad) * sin(CFangleXRad) + \
                                      mz * sin(CFangleYRad) * cos(CFangleXRad));


    // Factor in the declination angle for this location  (from http://www.ngdc.noaa.gov/geomag-web/#declination)
    float declinationAngle = -0.20780084;  // should this be negative?
    heading += declinationAngle;


  /* Convert angular data to degree */
  roll = roll * 180 / PI_F;
  pitch = pitch * 180 / PI_F;
  heading = heading * 180 / PI_F;

  heading += offset;


    // Convert to range (0, 360)
    heading = (heading > 0.0 ? heading : (360.0 + heading));


//  if ((heading > 0 && heading < 15) || (heading > 95 && heading < 110) || (heading > -80 && heading < -70))
  if (n == 27)// || n == 51)
//  if (heading > 95 && heading < 110)
  {
//    printf("%d:  Heading: %3.6f \t Pitch: %3.6f \t Roll: %3.6f\n", n, heading, pitch, roll);
    printf("%d:  Heading: %3.6f \t Pitch: %3.6f \t Roll: %3.6f\n", n, heading, CFangleY, CFangleX);
  }
//  else
  {
//    printf("%d:  Heading: %3.2f \t Pitch: %3.2f \t Roll: %3.2f\n", n, heading, pitch, roll);
  }
  return heading;
}
