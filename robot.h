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

// Constants for the pan/tilt camera servos
#define PAN_SERVO 1
#define TILT_SERVO 2
//#define PAN_MIN -40
//#define PAN_MAX 160
//#define TILT_MIN 20
//#define TILT_MAX 165
//#define PAN_MID 100 /*90*/
//#define TILT_MID 130
#define PAN_MIN -90
#define PAN_MAX 60
#define TILT_MIN -60
#define TILT_MAX 60
#define PAN_MID 0 /*90*/
#define TILT_MID 0


struct CatModeCtrl
{
    static const int maxX = 50;
    static const int minX = -50;
    static const int maxY = -30;
    static const int minY = -60;
    int x;
    int y;
    int intervalX;
    int intervalY;
    int moveCnt;
};


// Subsumption task control class
class ControlMode
{
public:
    bool active;
    int leftMotorPower;
    int rightMotorPower;

    ControlMode() {active = false; leftMotorPower = 0; rightMotorPower = 0; };
};


void panTiltTo(long pan, long tilt);
static void RotateDegrees(int degrees);

