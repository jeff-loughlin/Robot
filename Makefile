# the compiler: gcc for C program, define as g++ for C++
CPP = g++

OBJS = robot.o kalman.o PID_v1.o geocoords.o imu.o

# compiler flags:
#  -g    adds debugging information to the executable file
#  -Wall turns on most, but not all, compiler warnings
CFLAGS  = -g -Wall -DDEFAULTDEVICE=\"/dev/ttyUSB0\"


# the build target executable:
TARGET = robot

all: $(TARGET) ctrl

$(TARGET): $(OBJS)
	$(CPP) -g -lm -lusb-1.0 -lc -lgps -lRTIMULib -pthread -o $@ $^

%.o : %.cpp include.h
	$(CPP) $(CFLAGS) -o $@ -c $<


# Some other stuff we need - manual control program and the weather app
ctrl: ctrl.cpp
	$(CPP) $(CFLAGS) -o ctrl ctrl.cpp

weather: weather.cpp
	$(CPP) $(CFLAGS) -ljson -lcurl -o weather weather.cpp

intensity: intensity.cpp
	$(CPP) $(CFLAGS) -ljpeg -o intensity intensity.cpp

clean:
	$(RM) $(TARGET) $(OBJS)
