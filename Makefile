# the compiler: gcc for C program, define as g++ for C++
CPP = g++

OBJS = robot.o kalman.o PID_v1.o geocoords.o imu.o

# compiler flags:
#  -g    adds debugging information to the executable file
#  -Wall turns on most, but not all, compiler warnings
CFLAGS  = -g -Wall -DDEFAULTDEVICE=\"/dev/ttyACM0\"


# the build target executable:
TARGET = robot

all: $(TARGET) ctrl

$(TARGET): $(OBJS)
	$(CPP) -g -lm -lusb-1.0 -lc -lgps -pthread -o $@ $^

%.o : %.cpp include.h
	$(CPP) $(CFLAGS) -o $@ -c $<

ctrl: ctrl.cpp
	$(CPP) $(CFLAGS) -o ctrl ctrl.cpp

clean:
	$(RM) $(TARGET) $(OBJS)
