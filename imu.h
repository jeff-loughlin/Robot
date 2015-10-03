#ifndef __IMU__
#define __IMU__

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
#include <string>
bool selectDevice(int i2c, int addr, const char *name);
bool writeToDevice(int i2c, char * buf, int len);
bool readADXL345(int & fd,short & , short &, short &);
bool readL3G4200D(int & fd,short & , short &, short &);
bool readHMC5883L(int & fd,short & , short &, short &);
bool init(int &fd);
const short  ADXL345 = 0x53;
const short  L3G4200D = 0x69;
const short  HMC5883L = 0x1e;
//
#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23  // CTRL_REG4 controls the full-scale range, among other things:
#define CTRL_REG5 0x24  // CTRL_REG5 controls high-pass filtering of outputs


#endif // __IMU__
