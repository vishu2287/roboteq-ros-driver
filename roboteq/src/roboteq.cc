
/*
The MIT License (MIT)
Copyright (c) 2014 Jorge Claro
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/*
- devicepath (string)
- Default: none
- The serial port to be used.
- baud (integer)
- Default: 9600
- The baud rate to be used.
- encoder_ppr
- Default: 500
- The number of pulses per revolution for the encoders. Optional if no encoders are present.

- wheel_circumference
- Default: 1 meter
- The wheel circumference in meters. Optional if no encoders present.

- axle_length
- Default: 1 meter
- The distance between the centers of the two wheels. Optional if no encoders present.

- gear_ratio
- Default: 1
- The gear ratio from the motor to the wheel (the number of motor revolutions per one revolution of the wheel). Optional if no encoders present.

- controller_current_limit
- Default: 105 amperes
- The maximum current to the motors. See the Roboteq manual.

- acceleration
- Default: 0x20
- The acceleration time constant. See the Roboteq manual.

- encoder_time_base
- Default: 0x16
- The encoder time base. Optional if no encoders are present. See the Roboteq manual.

- encoder_distance_divider
- Default: 0x08
- The encoder distance divider. Optional if no encoders are present. See the Roboteq manual.

- invert_directions
- Default: false (off)
- Invert the motor directions, useful if you find the motors turning in the opposite direction from what you intended...

- rc_mode_on_shutdown
- Default: true (on)
- Set the motor controller to RC mode when the driver shuts down.
@author Pablo Rivera rivera@cse.unr.edu
@author Mike Roddewig mrroddew@mtu.edu
*/

#include <time.h>
#include <sys/time.h>
#include <assert.h>
#include <stdio.h>
#include <termios.h>
#include <sys/ioctl.h> // ioctl
#include <unistd.h> // close(2),fcntl(2),getpid(2),usleep(3),execvp(3),fork(2)
#include <netdb.h> // for gethostbyname(3)
#include <netinet/in.h>  // for struct sockaddr_in, htons(3)
#include <sys/types.h>  // for socket(2)
#include <sys/socket.h>  // for socket(2)
#include <signal.h>  // for kill(2)
#include <fcntl.h>  // for fcntl(2)
#include <string.h>  // for strncpy(3),memcpy(3)
#include <stdlib.h>  // for atexit(3),atoi(3)
#include <pthread.h>  // for pthread stuff
#include <math.h>
#include <float.h>
#include <signal.h>


#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <std_msgs/Bool.h>

// settings
#define SERIAL_BUFF_SIZE		128
#define MAX_MOTOR_SPEED			127
#define ROBOTEQ_CON_TIMEOUT		10	// seconds to time-out on setting RS-232 mode
#define ROBOTEQ_DEFAULT_BAUD		9600
#define INPUT_SWITCHES_FUNCTIONS 	0x00	// sets the input switches to none.

// Default parameter settings
#define DEFAULT_CONTROLLER_CURRENT_LIMIT	22	// Amperes
#define DEFAULT_ACCELERATION			0x30	// About 2 second from stop to full speed.
#define DEFAULT_ENCODER_TIME_BASE		0x16
#define DEFAULT_ENCODER_DISTANCE_DIVIDER	0x08
#define DEFAULT_GEAR_RATIO			1
#define DEFAULT_WHEEL_CIRCUMFERENCE		1
#define DEFAULT_AXLE_LENGTH			1
#define DEFAULT_ENCODER_PPR			500
#define DEFAULT_INVERT_DIRECTIONS		false
#define DEFAULT_RC_MODE_ON_SHUTDOWN		true
#define DEFAULT_KP                              1.0
#define DEFAULT_KI                              0.0
#define DEFAULT_KD                              0.0
#define DEFAULT_IMAX                            10.0
#define DEFAULT_IMIN                            -10.0

// Parameter addresses
#define CHANNEL1_OPERATING_MODE_ADDRESS	0x80
#define CHANNEL2_OPERATING_MODE_ADDRESS 	0x81
#define CONTROLLER_IDENTIFICATION_ADDRESS 	0x8A
#define CONTROLLER_STATUS_ADDRESS 		0x89
#define INPUT_CONTROL_MODE_ADDRESS		0x00
#define MOTOR_CONTROL_MODE_ADDRESS		0x01
#define CURRENT_LIMIT_ADDRESS			0x02
#define ACCELERATION_ADDRESS			0x03
#define INPUT_SWITCHES_FUNCTION_ADDRESS	0x04
#define ENCODER1_TIME_BASE_ADDRESS		0xA2
#define ENCODER2_TIME_BASE_ADDRESS		0xA3
#define ENCODER_DISTANCE_DIVIDER_ADDRESS	0xA5
#define EXPONENTIATION_CHANNEL1_ADDRESS	0x07
#define EXPONENTIATION_CHANNEL2_ADDRESS	0x08
#define PID_PROPORTIONAL_GAIN1_ADDRESS		0x82
#define PID_PROPORTIONAL_GAIN2_ADDRESS		0x83
#define PID_INTEGRAL_GAIN1_ADDRESS		0x84
#define PID_INTEGRAL_GAIN2_ADDRESS		0x85
#define PID_DIFFERENTIAL_GAIN1_ADDRESS		0x86
#define PID_DIFFERENTIAL_GAIN2_ADDRESS		0x87
#define PID_PROPORTIONAL_GAINT_ADDRESS		0x11
#define PID_INTEGRAL_GAINT_ADDRESS              0x0F
#define PID_DIFFERENTIAL_GAINT_ADDRESS		0x10

// Constants

//#define MOTOR_CONTROL_MODE_CLOSED_LOOP	0xC0
#define MOTOR_CONTROL_MODE_CLOSED_LOOP		0xC4 //separate closed loop
//#define MOTOR_CONTROL_MODE_CLOSED_LOOP	0xC5 //mixed
#define MOTOR_CONTROL_MODE_OPEN_LOOP		0x01
#define MAX_PID_GAIN					63.0/8.0
#define EXPONENTIATION_LINEAR				0x00
#define EXPONENTIATION_STRONG_EXP			0x02
#define INPUT_CONTROL_MODE				0x01
#define ENC_BUFF_SIZE   8

#define SLEEP_TIME 20000
#ifndef CRTSCTS
#ifdef IHFLOW
#ifdef OHFLOW
#define CRTSCTS ((IHFLOW) | (OHFLOW))
#endif
#endif
#endif

// Our Constants
#define ANGLE_TOLERANCE 0.08 //radians aprox. = 1 degree
#define MAX_ANGLE       0.45
#define MIN_FLOAT 	0.000001

// *************************************
// some assumptions made by this driver:

// ROBOTEQ is in "mixed mode" where
// channel 1 is translation
// channel 2 is rotation

// ROBOTEQ is set to be in RC mode by default

// the robot is a skid-steer vehicle where
// left wheel(s) are on one output,
// right wheel(s) on the other.
// directionality is implied by the following
// macros (FORWARD,REVERSE,LEFT,RIGHT)
// so outputs may need to be switched

// *************************************

int roboteq_fd;
char serialin_buff[SERIAL_BUFF_SIZE];
char serialout_buff[SERIAL_BUFF_SIZE];
const char* devicepath;
int roboteq_baud;
double speed_to_rpm;
double turning_circumference;
char encoder_present;
unsigned char motor_control_mode;
double max_forward_velocity;
double max_rotational_velocity;

// Config parameters.
int controller_current_limit;
unsigned char controller_current_limit_value;
int acceleration;
int encoder_time_base;
int encoder_distance_divider;
int encoder_ppr;
double wheel_circumference;
double axle_length;
double speed_per_tick;
double rad_per_tick;
double gear_ratio;
bool rc_mode_on_shutdown;
bool invert_directions;

struct SPid {
    double dState;		// Last position input
    double iState;		// Integrator state
    double iMax, iMin;	// Maximum and minimum allowable integrator state
}pid;

float position_enc;

double get_time(void);
int checkConfigParameter(char, int, int);

void UpdatePositionData();
int WriteMotorVelocity(double, double, double);
int ConvertEncoder(int ret);

geometry_msgs::Twist position_data;
geometry_msgs::Twist current_position;

double cur_time, time_old, time_dif;
int switched;
double Kp, Kd, Ki;
int count_enc_1;
int count_enc_2;
float encoder_angle;

double last_desired_angle;
int right_turn;
int left_turn;
bool setup_ok=false;

//Cmd_vel Parameters
double forward_velocity_x;
double turnrate_factor;
double rotational_velocity;

void init_default_parameters(){
    devicepath="/dev/ttyUSB0";
    gear_ratio=87.967;
    axle_length=0.54;
    wheel_circumference=2.135;
    encoder_ppr=500;
    encoder_time_base=2;
    encoder_distance_divider=63;
    acceleration=DEFAULT_ACCELERATION;
    roboteq_baud=ROBOTEQ_DEFAULT_BAUD;
    rc_mode_on_shutdown=DEFAULT_RC_MODE_ON_SHUTDOWN;
    invert_directions=false;
    Kp=3.0;
    Ki=0.6;
    Kd=0.05;
    pid.iMax = DEFAULT_IMAX;
    pid.iMin = DEFAULT_IMIN;
    pid.dState = 0.0;
    pid.iState = 0.0;
    controller_current_limit=DEFAULT_CONTROLLER_CURRENT_LIMIT;
}

int Set_channels_PIDvalues() {
    int ret, error=0;
    char command_status;
    
    if (checkConfigParameter('^', PID_PROPORTIONAL_GAIN1_ADDRESS, (unsigned char) (Kp * 8)) != 0) {
        ROS_WARN("Error setting channel one proportional gain");
        error=1;
        //return -1;
    }
    else
        ROS_INFO("Set channel one proportional gain successfully");
    
    if (checkConfigParameter('^', PID_PROPORTIONAL_GAIN2_ADDRESS, (unsigned char) (Kp * 8)) != 0) {
        ROS_WARN("Error setting channel two proportional gain");
        error=1;
        //return -1;
    }
    else
        ROS_INFO("Set channel two proportional gain successfully");
    
    if (checkConfigParameter('^', PID_INTEGRAL_GAIN1_ADDRESS, (unsigned char) (Ki * 8)) != 0) {
        ROS_WARN("Error setting channel one integral gain");
        error=1;
        //return -1;
    }
    else
        ROS_INFO("Set channel one integral gain successfully");
    
    if (checkConfigParameter('^', PID_INTEGRAL_GAIN2_ADDRESS, (unsigned char) (Ki * 8)) != 0) {
        ROS_WARN("Error setting channel two integral gain");
        error=1;
        //return -1;
    }
    else
        ROS_INFO("Set channel two integral gain successfully");
    
    if (checkConfigParameter('^', PID_DIFFERENTIAL_GAIN1_ADDRESS, (unsigned char) (Kd * 8)) != 0) {
        ROS_WARN("Error setting channel one differential gain");
        error=1;
        //return -1;
    }
    else
        ROS_INFO("Set channel one diferential gain successfully");
    
    if (checkConfigParameter('^', PID_DIFFERENTIAL_GAIN2_ADDRESS, (unsigned char) (Kd * 8)) != 0) {
        ROS_WARN("Error setting channel two differential gain");
        error=1;
        //return -1;
    }
    else
        ROS_INFO("Set channel two differential gain successfully");
    
    // Sending ^FF to apply new settings
    strcpy(serialout_buff, "^FF\r");
    write(roboteq_fd, serialout_buff, strlen(serialout_buff));
    tcdrain(roboteq_fd);
    usleep(SLEEP_TIME);
    
    // Checking if new settings where applyed sucessfully
    memset(serialin_buff, 0, SERIAL_BUFF_SIZE);
    ret = read(roboteq_fd, serialin_buff, SERIAL_BUFF_SIZE - 1);
    serialin_buff[SERIAL_BUFF_SIZE - 1] = 0x00; // Null terminate our buffer to make sure sscanf doesn't run amok.
    
    ret = sscanf(serialin_buff, "%*3c %1c", &command_status);
    
    if (command_status != '+') {
        ROS_WARN("Falied to apply new PID settings");
        error=1;
    }
    else{
        ROS_INFO("New PID settings applied successfully");
    }
    
    if(error==1)
    return -1;
    
    return 0;
}

int MainQuit() {
    int error=0;
    
    // Shut off the motors.
    WriteMotorVelocity(0.0, 0.0, 0.0);
    
    if (rc_mode_on_shutdown == true) {
        
        //printf("Turning to radio mode\n");
        ROS_INFO("Turning to radio mode");
        
        // Reset the input exponentiation mode to strong exponential.
        if (checkConfigParameter('^', EXPONENTIATION_CHANNEL1_ADDRESS, EXPONENTIATION_STRONG_EXP) != 0) {
            //printf("Error setting channel 1 exponentiation to strong expoential\n");
            ROS_ERROR("Error setting channel 1 exponentiation to strong expoential");
            error=1;
        }
        else{
            //printf("Set channel 1 exponentiation to strong expoential successfully\n");
            ROS_INFO("Set channel 1 exponentiation to strong expoential successfully");
        }
        
        if (checkConfigParameter('^', EXPONENTIATION_CHANNEL2_ADDRESS, EXPONENTIATION_STRONG_EXP) != 0) {
            //printf("Error setting channel 2 exponentiation to strong expoential\n");
            ROS_ERROR("Error setting channel 2 exponentiation to strong expoential");
            error=1;
        }
        else{
            //printf("Set channel 2 exponentiation to strong expoential successfully\n");
            ROS_INFO("Set channel 2 exponentiation to strong expoential successfully");
        }
        
        // Set roboteq to radio mode
        //printf("Set roboteq in radio mode\n");
        ROS_INFO("Set roboteq in radio mode");
        strcpy(serialout_buff, "^00 00\r");
        write(roboteq_fd, serialout_buff, strlen(serialout_buff));
        tcdrain(roboteq_fd);
        usleep(SLEEP_TIME);
        
        /*
        //strcpy(serialout_buff, "^01 01\r"); // OPEN LOOP MIXED MODE
        //strcpy(serialout_buff, "^01 C5\r");  // CLOSED LOOP MIXED MODE (usar este)
        write(roboteq_fd, serialout_buff, strlen(serialout_buff));
        tcdrain(roboteq_fd);
        usleep(SLEEP_TIME);
        */
        
        //printf("Set roboteq in closed loop mode\n");
        ROS_INFO("Set roboteq in closed loop mode");
        motor_control_mode = MOTOR_CONTROL_MODE_CLOSED_LOOP;
        if (checkConfigParameter('^', MOTOR_CONTROL_MODE_ADDRESS, motor_control_mode) != 0) {
            //printf("Error setting motor control mode\n");
            ROS_ERROR("Error setting motor control mode");
            error=1;
            //return -1;
        }
        else{
            //printf("Set Motor control mode successfully\n");
            ROS_INFO("Set Motor control mode successfully");
        }
        
        // Reset de controller to apply new settings
        //printf("Rebooting the roboteq\n");
        ROS_INFO("Rebooting the roboteq");
        strcpy(serialout_buff, "%rrrrrr\r");
        write(roboteq_fd, serialout_buff, strlen(serialout_buff));
        tcdrain(roboteq_fd);
        sleep(2);
        
        tcflush(roboteq_fd, TCIFLUSH); // Clear all the reboot crap out of the input buffer (after a reset)
        
        //printf("RADIO MODE ON\n");
        ROS_INFO("RADIO MODE ON");
    }
    
    close(roboteq_fd);
    
    if(error)
    return -1;
    
    return 0;
}


int MainSetup() {
    int returned_value;
    int ret;
    int i;
    int error=0;
    
    position_enc = 0;
    
    time_old = 0;
    cur_time = 0;
    time_dif = 0;
    switched = 0;
    
    // Compute the controler current limit value to send to the microcontroller.
    if (controller_current_limit < 1) {
        ROS_ERROR("The current limit must be greater than 0.");
        return -1;
    } else if (controller_current_limit <= 30) {
        controller_current_limit_value = 0;
        controller_current_limit_value += (30 - controller_current_limit) << 4;
    } else if (controller_current_limit <= 45) {
        controller_current_limit_value = 1;
        controller_current_limit_value += (45 - controller_current_limit) << 4;
    } else if (controller_current_limit <= 60) {
        controller_current_limit_value = 2;
        controller_current_limit_value += (60 - controller_current_limit) << 4;
    } else if (controller_current_limit <= 75) {
        controller_current_limit_value = 3;
        controller_current_limit_value += (75 - controller_current_limit) << 4;
    } else if (controller_current_limit <= 90) {
        controller_current_limit_value = 4;
        controller_current_limit_value += (90 - controller_current_limit) << 4;
    } else if (controller_current_limit <= 105) {
        controller_current_limit_value = 5;
        controller_current_limit_value += (105 - controller_current_limit) << 4;
    } else if (controller_current_limit <= 120) {
        controller_current_limit_value = 6;
        controller_current_limit_value += (120 - controller_current_limit) << 4;
    } else {
        ROS_ERROR("The current limit value must be less than 120 A.");
        return -1;
    }
    
    //memset(&position_data, 0, sizeof (position_data));
    //memset(&power_data, 0, sizeof (power_data));
    roboteq_fd = -1;
    
    roboteq_fd = open(devicepath, O_RDWR | O_NDELAY);
    if (roboteq_fd == -1) {
        ROS_ERROR("Unable to configure roboteq serial port at: %s Baud:%d", devicepath, roboteq_baud);
        return -1;
    } else {
        struct termios options;
        tcgetattr(roboteq_fd, &options);
        
        // default is 9600 unless otherwise specified
        
        if (roboteq_baud == 4800) {
            cfsetispeed(&options, B4800);
            cfsetospeed(&options, B4800);
        } else if (roboteq_baud == 19200) {
            cfsetispeed(&options, B19200);
            cfsetospeed(&options, B19200);
        } else if (roboteq_baud == 38400) {
            cfsetispeed(&options, B38400);
            cfsetospeed(&options, B38400);
        } else {
            cfsetispeed(&options, B9600);
            cfsetospeed(&options, B9600);
        }
        
        // set to 7bit even parity, no flow control
        options.c_cflag |= (CLOCAL | CREAD);
        options.c_cflag |= PARENB;
        options.c_cflag &= ~PARODD;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS7;
        options.c_cflag &= ~CRTSCTS;
        
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // non-canonical
        
        tcsetattr(roboteq_fd, TCSANOW, &options);
        ioctl(roboteq_fd, TCIOFLUSH, 2);
        tcflush(roboteq_fd, TCIFLUSH);
        ROS_INFO("Roboteq serial port configured successfully at %s Baud:%d", devicepath, roboteq_baud);
    }
    
    // Compute the encoder speed to RPM conversion factor.
    speed_to_rpm = (60.0 * 1000000.0) / (((double) encoder_ppr) * 4.0 * 256.0 * (((double) encoder_time_base) + 1.0));
    //speed_to_rpm = (60.0 * 1000000.0) / (((double) encoder_ppr) * 256.0 * (((double) encoder_time_base) + 1.0));
    
    // Compute the speed value to m/s conversion factor.
    speed_per_tick = (speed_to_rpm * wheel_circumference) / (gear_ratio * 60);
    
    // Compute the turning circumference.
    turning_circumference = 2.0 * M_PI * axle_length;
    
    // Compute the speed value to rad/s conversion factor.
    rad_per_tick = (2.0 * M_PI * speed_per_tick) / turning_circumference;
    
    max_forward_velocity = speed_per_tick * 127;
    max_rotational_velocity = rad_per_tick * 127;
    
    ROS_INFO("Computed maximum forward velocity of %f m/s.", max_forward_velocity);
    ROS_INFO("Computed maximum rotational velocity of %f rad/s.", max_rotational_velocity);
    
    // Set roboteq to radio mode
    ROS_INFO("Set the roboteq in radio mode");
    strcpy(serialout_buff, "^00 00\r");
    write(roboteq_fd, serialout_buff, strlen(serialout_buff));
    tcdrain(roboteq_fd);
    usleep(SLEEP_TIME);
    
    ROS_INFO("Rebooting the roboteq");
    strcpy(serialout_buff, "%rrrrrr\r");
    write(roboteq_fd, serialout_buff, strlen(serialout_buff));
    tcdrain(roboteq_fd);
    sleep(2); // Sleep for two seconds to give the controller sufficient time to reboot.
    
    tcflush(roboteq_fd, TCIFLUSH); // Clear all the reboot crap out of the input buffer (after a reset).
    
    // initialize RoboteQ to RS-232 mode
    ROS_INFO("Initializing roboteq in RS-232 mode");
    strcpy(serialout_buff, "\r");
    for (i = 0; i < 10; i++) {
        write(roboteq_fd, serialout_buff, strlen(serialout_buff));
        tcdrain(roboteq_fd);
        usleep(SLEEP_TIME);
    }
    
    // Receives OK Message from roboteq
    memset(serialin_buff, 0, SERIAL_BUFF_SIZE);
    ret = read(roboteq_fd, serialin_buff, SERIAL_BUFF_SIZE - 1);
    serialin_buff[SERIAL_BUFF_SIZE - 1] = 0x00; // Null terminate our buffer to make sure sscanf doesn't run amok.
    
    if(strstr(serialin_buff, "OK")!=NULL)
    ROS_INFO("Received OK from the controller");
    else{
        ROS_ERROR("Did not Receive OK message from the controller");
        return -1;
    }
    
    // Disable the watchdog timer.
    ROS_INFO("Disabling the watchdog timer");
    strcpy(serialout_buff, "^00 01\r");
    write(roboteq_fd, serialout_buff, strlen(serialout_buff));
    tcdrain(roboteq_fd);
    usleep(SLEEP_TIME);
    
    ROS_INFO("Rebooting the roboteq");
    strcpy(serialout_buff, "%rrrrrr\r");
    write(roboteq_fd, serialout_buff, strlen(serialout_buff));
    tcdrain(roboteq_fd);
    sleep(2); // Sleep for two seconds to give the controller sufficient time to reboot.
    
    tcflush(roboteq_fd, TCIFLUSH); // Clear all the reboot crap out of the input buffer (after a reset).
    
    // Read controller model to make sure the serial link is ok.
    ROS_INFO("Reading the controller model");
    strcpy(serialout_buff, "^8A\r");
    write(roboteq_fd, serialout_buff, strlen(serialout_buff));
    tcdrain(roboteq_fd);
    usleep(SLEEP_TIME);
    
    memset(serialin_buff, 0, SERIAL_BUFF_SIZE);
    ret = read(roboteq_fd, serialin_buff, SERIAL_BUFF_SIZE - 1);
    serialin_buff[SERIAL_BUFF_SIZE - 1] = 0x00; // Null terminate our buffer to make sure sscanf doesn't run amok.
    
    ret = sscanf(serialin_buff, "%*3c %2X", &returned_value);
    if (ret != 1) {
        ROS_ERROR("Unable to communicate with the controller! Check the serial device");
        return -1;
    } else {
        if (returned_value & 0x01) {
            ROS_INFO("AX500 found");
        } else if (returned_value & 0x02) {
            ROS_INFO("AX1500 found");
        } else if (returned_value & 0x04) {
            ROS_INFO("AX2500 found");
        } else if (returned_value & 0x08) {
            ROS_INFO("AX3500 found");
        } else {
            ROS_WARN("Unknown controller found"); // Weird...this shouldn't happen.
        }
        
        if (returned_value & 0x20) {
            ROS_INFO("Encoder present");
            encoder_present = 1;
        } else {
            encoder_present = 0;
        }
        
        if (returned_value & 0x40) {
            ROS_INFO("Short circuit detection capable");
        }
    }
    
    if (encoder_present == 1) {
        motor_control_mode = MOTOR_CONTROL_MODE_CLOSED_LOOP;
    } else {
        motor_control_mode = MOTOR_CONTROL_MODE_OPEN_LOOP;
    }
    
    // Set configuration parameters
    if (checkConfigParameter('^', MOTOR_CONTROL_MODE_ADDRESS, motor_control_mode) != 0) {
        ROS_ERROR("Error setting motor control mode");
        error=1;
        //return -1;
    }
    else
        ROS_INFO("Set Motor control mode successfully");
    
    if (checkConfigParameter('^', CURRENT_LIMIT_ADDRESS, controller_current_limit_value) != 0) {
        ROS_ERROR("Error setting controller current limit");
        error=1;
        //return -1;
    }
    else
        ROS_INFO("Set motor controler current limit successfully");
    
    if (checkConfigParameter('^', ACCELERATION_ADDRESS, acceleration) != 0) {
        ROS_ERROR("Error setting acceleration profile");
        error=1;
        //return -1;
    }
    else
        ROS_INFO("Set acceleration profile successfully");
    
    if (checkConfigParameter('^', EXPONENTIATION_CHANNEL1_ADDRESS, EXPONENTIATION_LINEAR) != 0) {
        ROS_ERROR("Error setting channel 1 exponentiation to linear");
        error=1;
        //return -1;
    }
    else
        ROS_INFO("Set channel 1 exponentiation to linear successfully");
    
    if (checkConfigParameter('^', EXPONENTIATION_CHANNEL2_ADDRESS, EXPONENTIATION_LINEAR) != 0) {
        ROS_ERROR("Error setting channel 2 exponentiation to linear");
        error=1;
        //return -1;
    }
    else
        ROS_INFO("Set channel 2 exponentiation to linear successfully");
    
    if (encoder_present != 0) {
        
        // Set encoder parameters.
        if (checkConfigParameter('*', ENCODER1_TIME_BASE_ADDRESS, encoder_time_base) != 0) {
            ROS_ERROR("Error setting encoder one time base");
            error=1;
            //return -1;
        }
        else
            ROS_INFO("Set encoder one time base successfully");
        
        if (checkConfigParameter('*', ENCODER2_TIME_BASE_ADDRESS, encoder_time_base) != 0) {
            ROS_ERROR("Error setting encoder two time base");
            error=1;
            //return -1;
        }
        else
            ROS_INFO("Set encoder two time base successfully");
        
        if (checkConfigParameter('*', ENCODER_DISTANCE_DIVIDER_ADDRESS, encoder_distance_divider) != 0) {
            ROS_ERROR("Error setting encoder distance divider");
            error=1;
            //return -1;
        }
        else
            ROS_INFO("Set encoder distance divider successfully");
    }
    
    // Reboot the controller to apply new settings (if not yet...)
    strcpy(serialout_buff, "%rrrrrr\r");
    write(roboteq_fd, serialout_buff, strlen(serialout_buff));
    tcdrain(roboteq_fd);
    sleep(2); // Sleep for two seconds to give the controller sufficient time to reboot.
    
    tcflush(roboteq_fd, TCIFLUSH); // Clear all the reboot crap out of the input buffer (after a reset).
    
    count_enc_1 = 0;
    count_enc_2 = 0;
    
    time_t rawtime;
    time(&rawtime);
    struct tm * timeinfo;
    timeinfo = localtime(&rawtime);
    char encpath[100];
    char path_aux[5];
    int year, month;
    
    //Define pid gains in controller
    if (checkConfigParameter('^', PID_PROPORTIONAL_GAINT_ADDRESS, (unsigned char) (Kp * 8)) != 0) {
        ROS_WARN("Error setting total proportional gain");
        error=1;
        //return -1;
    }
    else
        ROS_INFO("Set total proportional gain sucessfully");
    
    if (checkConfigParameter('^', PID_INTEGRAL_GAINT_ADDRESS, (unsigned char) (Ki * 8)) != 0) {
        ROS_WARN("Error setting total integral gain");
        error=1;
        //return -1;
    }
    else
        ROS_INFO("Set total integral gain sucessfully");
    
    if (checkConfigParameter('^', PID_DIFFERENTIAL_GAINT_ADDRESS, (unsigned char) (Kd * 8)) != 0) {
        ROS_WARN("Error setting total differential gain");
        error=1;
        //return -1;
    }
    else
        ROS_INFO("Set total differential gain sucessfully");
    
    Set_channels_PIDvalues();
    
    /*
    // Reboot the controller to apply new settings (if not yet...)
    strcpy(serialout_buff, "%rrrrrr\r");
    write(roboteq_fd, serialout_buff, strlen(serialout_buff));
    tcdrain(roboteq_fd);
    sleep(2); // Sleep for two seconds to give the controller sufficient time to reboot.
    tcflush(roboteq_fd, TCIFLUSH); // Clear all the reboot crap out of the input buffer (after a reset).
    */
    
    encoder_angle = 0;
    last_desired_angle = 0.0;
    right_turn = 0;
    left_turn = 0;
    
    setup_ok=true;
    
    if(error)
    return -1;
    
    return 0;
}

// Check the value of the configuration parameter located at the given address,
// set it to the value if it does not equal the provided value.

int checkConfigParameter(char prefix, int address, int value) {
    unsigned int returned_value;
    char command_status;
    int ret;
    
    sprintf(serialout_buff, "%c%.2X\r", prefix, address);
    write(roboteq_fd, serialout_buff, strlen(serialout_buff));
    tcdrain(roboteq_fd);
    usleep(SLEEP_TIME);
    
    memset(serialin_buff, 0, SERIAL_BUFF_SIZE);
    ret = read(roboteq_fd, serialin_buff, SERIAL_BUFF_SIZE - 1);
    serialin_buff[SERIAL_BUFF_SIZE - 1] = 0x00; // Null terminate our buffer to make sure sscanf doesn't run amok.
    
    ret = sscanf(serialin_buff, "%*3c %2X", &returned_value);
    if (ret != 1) {
        returned_value = value + 1;
        // Yes, it's hackish. BUT, for some reason if you try and read a value that has not been set instead of returning
        // say -1 the Roboteq controller will return an error which will screw this function up. But, once you write to
        // the address everything is just peachy. So, we basically ensure that we write to the address. And we do look
        // for an error further down the line.
    }
    
    if ((char) returned_value != value) {
        sprintf(serialout_buff, "%c%.2X %.2X\r", prefix, address, value);
        write(roboteq_fd, serialout_buff, strlen(serialout_buff));
        tcdrain(roboteq_fd);
        usleep(SLEEP_TIME);
        
        // Check that the command was received ok.
        
        memset(serialin_buff, 0, SERIAL_BUFF_SIZE);
        ret = read(roboteq_fd, serialin_buff, SERIAL_BUFF_SIZE - 1);
        serialin_buff[SERIAL_BUFF_SIZE - 1] = 0x00; // Null terminate our buffer to make sure sscanf doesn't run amok.
        
        ret = sscanf(serialin_buff, "%*6c %1c", &command_status);
        
        if (ret != 1 || command_status != '+') {
            ROS_WARN("Error writing command in flash");
            return -2; // An error occured writing the command.
        }
    }
    return 0;
}


double get_time(void) {
    struct timeval curr;
    gettimeofday(&curr, NULL);
    
    return curr.tv_usec;
}

int WriteMotorVelocity(double forward_velocity_x, double turnrate_factor, double rotational_velocity) {
    //unsigned char forward_value, rotational_value;
    unsigned char Vif_value, Vof_value;
    char returned_value;
    int ret;
    double pidTerm = 0.0;
    double desired_angle = 0.0;
    
    desired_angle = rotational_velocity;
    ROS_DEBUG("For_vel_x %f turn_fact %f rot_vel %f",forward_velocity_x,turnrate_factor,rotational_velocity);
    
    /***** Code for setting curve speed in controller TF 02/03/2010 *****/
    
    double Rsteer = 0.0;
    //double Rback = 0.0;
    //double Rrobot = 0.0;
    double L = 0.81;
    double B = 0.52;
    //double Rwheel = 0.33;
    double Vif = 0.0;
    double Vof = 0.0;
    double PID_mul_factor = 0.0;
    
    pidTerm = turnrate_factor;
    //ROS_DEBUG("pidTerm %f\n", pidTerm);
    
    if (forward_velocity_x != 0.0) {
        
        if (fabs(rotational_velocity) > MIN_FLOAT) {
            //ROS_DEBUG("rotational_velocity %f\n", rotational_velocity);
            Rsteer = L / sin(rotational_velocity);
            
            //Rback = sqrt((pow(Rsteer, 2) - pow(L, 2)));
            //Rrobot = sqrt((pow(Rback, 2) + pow((L / 2), 2)));
            
            if (forward_velocity_x > 0.0){
                
                Vof = ((Rsteer + B / 2) / Rsteer) * forward_velocity_x;
                Vif = ((Rsteer - B / 2) / Rsteer) * forward_velocity_x;
            }
            else if (forward_velocity_x < 0.0){
                
                Vif = ((Rsteer + B / 2) / Rsteer) * forward_velocity_x;
                Vof = ((Rsteer - B / 2) / Rsteer) * forward_velocity_x;
            }
            
        } else {
            
            //ROS_DEBUG("Rotational velocity zero!\n");
            Vof = forward_velocity_x;
            Vif = Vof;
        }
        
        // If desired angle changed
        if (last_desired_angle != desired_angle) {
            //ROS_DEBUG("desired angle changed: last %f new %f\n", this->last_desired_angle, desired_angle);
            
            if (encoder_angle <= desired_angle) {
                //ROS_DEBUG("right turn!\n");
                right_turn = 1;
                left_turn = 0;
                
            } else if (encoder_angle > desired_angle) {
                //ROS_DEBUG("left turn!\n");
                right_turn = 0;
                left_turn = 1;
            }
            
            last_desired_angle = desired_angle;
        }
        
        //Speed up steering angle using turnrate factor
        //printf("Enc angle %f rot vel %f\n",encoder_angle,rotational_velocity);
        
        
        ROS_DEBUG("Before pidTerm Vof: %f Vif: %f\n", Vof, Vif);
        
        if (right_turn) {
            
            PID_mul_factor = 1.0 + pidTerm;
            ROS_DEBUG("Turning right with PID_mul_factor %f\n", PID_mul_factor);
            
            if (forward_velocity_x > 0.0){
                Vof = Vof * PID_mul_factor;
                Vif = Vif;
            }
            else if (forward_velocity_x < 0.0){
                Vif = Vif * PID_mul_factor;
                Vof = Vof;
            }
            
        } else if (left_turn) {
            
            PID_mul_factor = 1.0 + pidTerm;
            ROS_DEBUG("Turning left with PID_mul_factor %f\n", PID_mul_factor);
            // Caso crescente
            
            if (forward_velocity_x > 0.0){
                Vif = Vif * PID_mul_factor;
                Vof = Vof;
            }
            else if (forward_velocity_x < 0.0){
                Vof = Vof * PID_mul_factor;
                Vif = Vif;
            }
            
        }
        
        ROS_DEBUG("After pidTerm Vof: %f Vif: %f\n", Vof, Vif);
        
        if (fabs(Vif) > max_forward_velocity) {
            Vif_value = 0x7F;
        } else {
            Vif_value = (unsigned char) (fabs(Vif) / speed_per_tick);
        }
        
        if (fabs(Vof) > max_forward_velocity) {
            Vof_value = 0x7F;
        } else {
            Vof_value = (unsigned char) (fabs(Vof) / speed_per_tick);
        }
        ROS_DEBUG("VIF_value %d VOF_value %d Max Velocity %f speed per tick %f\n", Vif_value, Vof_value, max_forward_velocity, speed_per_tick);
        
    } else {
        
        Vif_value = (unsigned char) (fabs(forward_velocity_x) / speed_per_tick);
        Vof_value = Vif_value;
    }
    
    //printf("Channel A ->   %f Channel B ->  %f      Frwd -> %f\n",Vif, Vof,forward_velocity);
    
    /**************************************************/
    
    // Check if we need to invert the velocities.
    if (invert_directions == true) {
        forward_velocity_x = -forward_velocity_x;
        rotational_velocity = -rotational_velocity;
    }
    
    // Write forward velocity and check result.
    double time1,time2;
    time1=get_time();
    
    if (switched == 0) {
        
        if (forward_velocity_x < 0) {
            sprintf(serialout_buff, "!a%.2X\r", Vif_value);
        } else {
            sprintf(serialout_buff, "!A%.2X\r", Vif_value);
        }
        
        write(roboteq_fd, serialout_buff, strlen(serialout_buff));
        tcdrain(roboteq_fd);
        usleep(SLEEP_TIME);
        
        memset(serialin_buff, 0, SERIAL_BUFF_SIZE);
        ret = read(roboteq_fd, serialin_buff, SERIAL_BUFF_SIZE - 1);
        serialin_buff[SERIAL_BUFF_SIZE - 1] = 0x00; // Null terminate our buffer to make sure sscanf doesn't run amok.
        
        ret = sscanf(serialin_buff, "%*4c %1c", &returned_value);
        
        // to avoid unused warning...
        if (ret==ret){};
        
        if (returned_value != '+') {
            // Some kind of error happened.
            ROS_WARN("Error writing forward velocity command on channel one");
            //return -1;
        }
        else{
            ROS_DEBUG("Forward velocity command set successully on channel one");
        }
        // Write rotational velocity and check result.
        if (forward_velocity_x < 0) {
            sprintf(serialout_buff, "!b%.2X\r", Vof_value);
        } else {
            sprintf(serialout_buff, "!B%.2X\r", Vof_value);
        }
        
        write(roboteq_fd, serialout_buff, strlen(serialout_buff));
        tcdrain(roboteq_fd);
        usleep(SLEEP_TIME);
        
        memset(serialin_buff, 0, SERIAL_BUFF_SIZE);
        ret = read(roboteq_fd, serialin_buff, SERIAL_BUFF_SIZE - 1);
        serialin_buff[SERIAL_BUFF_SIZE - 1] = 0x00; // Null terminate our buffer to make sure sscanf doesn't run amok.
        
        ret = sscanf(serialin_buff, "%*4c %1c", &returned_value);
        
        if (returned_value != '+') {
            // Some kind of error happened.
            ROS_WARN("Error writing rotational velocity command on channel one");
            //return -1;
        }
        else{
            ROS_DEBUG("Rotational velocity command set successully on channel one");
        }
        switched = 1;
    } else {
        if (forward_velocity_x < 0) {
            sprintf(serialout_buff, "!b%.2X\r", Vof_value);
        } else {
            sprintf(serialout_buff, "!B%.2X\r", Vof_value);
        }
        
        
        write(roboteq_fd, serialout_buff, strlen(serialout_buff));
        tcdrain(roboteq_fd);
        usleep(SLEEP_TIME);
        
        memset(serialin_buff, 0, SERIAL_BUFF_SIZE);
        ret = read(roboteq_fd, serialin_buff, SERIAL_BUFF_SIZE - 1);
        serialin_buff[SERIAL_BUFF_SIZE - 1] = 0x00; // Null terminate our buffer to make sure sscanf doesn't run amok.
        
        ret = sscanf(serialin_buff, "%*4c %1c", &returned_value);
        
        if (returned_value != '+') {
            // Some kind of error happened.
            ROS_WARN("Error writing forward velocity command on channel two (switched)");
            //return -1;
        }
        else{
            ROS_DEBUG("Forward velocity command set successully on channel two (switched)");
        }
        // Write rotational velocity and check result.
        if (forward_velocity_x < 0) {
            sprintf(serialout_buff, "!a%.2X\r", Vif_value);
        } else {
            sprintf(serialout_buff, "!A%.2X\r", Vif_value);
        }
        
        write(roboteq_fd, serialout_buff, strlen(serialout_buff));
        tcdrain(roboteq_fd);
        usleep(SLEEP_TIME);
        
        memset(serialin_buff, 0, SERIAL_BUFF_SIZE);
        ret = read(roboteq_fd, serialin_buff, SERIAL_BUFF_SIZE - 1);
        serialin_buff[SERIAL_BUFF_SIZE - 1] = 0x00; // Null terminate our buffer to make sure sscanf doesn't run amok.
        
        ret = sscanf(serialin_buff, "%*4c %1c", &returned_value);
        
        if (returned_value != '+') {
            // Some kind of error happened.
            ROS_WARN("Error writing rotational velocity command on channel two (switched)");
            //return -1;
        }
        else{
            ROS_DEBUG("Rotational velocity command set successully on channel two (switched)");
        }
        switched = 0;
    }
    
    time2=get_time();
    
    ROS_DEBUG("To order Vof: %f Vif: %f Time diff %f",Vof, Vif,(double) (time2-time1));
    
    return 0;
}

void UpdatePositionData() {
    int ret;
    int speed1_value, speed2_value;
    int encoder1_count, encoder2_count;
    double rpm1, rpm2;
    double speed1, speed2, speed_diff;
    double distance1, distance2, distance_diff;
    
    // Ok, the math for this section is a bit tricky. Wheel 1 is assumed to be on the right
    // and a positive rotational velocity indicates the robot is turning left. To determine
    // the rotational velocity we compute the difference in wheel velocities and then
    // compute the rotational velocity by converting m/s to rad/s on a circle of radius
    // axle length.
    
    /*
    // Read in the current speed value.
    strcpy(serialout_buff, "?Z\r");
    write(roboteq_fd, serialout_buff, strlen(serialout_buff));
    tcdrain(roboteq_fd);
    usleep(SLEEP_TIME);
    memset(serialin_buff, 0, SERIAL_BUFF_SIZE);
    ret = read(roboteq_fd, serialin_buff, SERIAL_BUFF_SIZE - 1);
    serialin_buff[SERIAL_BUFF_SIZE - 1] = 0x00;
    ret = sscanf(serialin_buff, "%*2c %2X %2X", &speed1_value, &speed2_value);
    if (ret != 2) {
    return; // Well, best not to update the data without any data...
    }
    // Compute the speed (in m/s).
    speed1 = ((double) speed1_value) * speed_per_tick;
    speed2 = ((double) speed2_value) * speed_per_tick;
    //printf("Reading ->  %2X	    %2X\n",speed1_value,speed2_value);
    if (invert_directions == true) {
    speed1 = -speed1;
    speed2 = -speed2;
    }
    speed_diff = speed1 - speed2;
    position_data->angular.z = (speed_diff / turning_circumference) * (2.0 * M_PI);
    position_data->linear.x = 0.0;
    if (fabs(speed1) > fabs(speed2)) {
    position_data->linear.y = speed2;
    } else {
    position_data->linear.y = speed1;
    }
    */
    
    // Compute a new position. We don't use the speed data.
    tcflush(roboteq_fd, TCIOFLUSH);
    
    // Get the encoder counters. We use the relative mode.
    strcpy(serialout_buff, "?Q4\r");
    write(roboteq_fd, serialout_buff, strlen(serialout_buff));
    tcdrain(roboteq_fd);
    usleep(SLEEP_TIME);
    
    memset(serialin_buff, 0, SERIAL_BUFF_SIZE);
    ret = read(roboteq_fd, serialin_buff, SERIAL_BUFF_SIZE - 1);
    
    //Function to parse buff for encoder_count
    encoder1_count = ConvertEncoder(ret);
    
    tcflush(roboteq_fd, TCIOFLUSH);
    
    strcpy(serialout_buff, "?Q5\r");
    write(roboteq_fd, serialout_buff, strlen(serialout_buff));
    tcdrain(roboteq_fd);
    usleep(SLEEP_TIME);
    
    memset(serialin_buff, 0, SERIAL_BUFF_SIZE);
    ret = read(roboteq_fd, serialin_buff, SERIAL_BUFF_SIZE - 1);
    
    //Function to parse buff for encoder_count
    encoder2_count = ConvertEncoder(ret);
    
    if (invert_directions == false) {
        encoder1_count = -encoder1_count;
        encoder2_count = -encoder2_count;
    }
    
    time_old = cur_time;
    cur_time = get_time();
    
    if (cur_time < time_old) {
        time_dif = 1000000 - (time_old - cur_time);
    } else {
        time_dif = cur_time - time_old;
    }
    
    //printf(" Time: %f  Time old : %f Time dif : %f ",cur_time,time_old,time_dif);
    
    speed1 = (encoder1_count * 0.000048635 * 0.25) / (time_dif / 1000000);
    speed2 = (encoder2_count * 0.000048635 * 0.25) / (time_dif / 1000000);
    
    
    speed_diff = speed1 - speed2;
    
    position_data.angular.z = (speed_diff / turning_circumference) * (2.0 * M_PI);
    position_data.linear.x = speed2;
    position_data.linear.y = speed1;
    
    //  printf("Reading ->  %d	    %d   time_dif %f speed1 %f speed2 %f\n", encoder1_count, encoder2_count,(float)time_dif, speed1, speed2);
    
    
    // NOTE!!! There could be a bug here. If the driver is pre-empted or the delay
    // between reading encoder one and two is too long then there will be an error
    // in the computed position. It seems odd that Roboteq wouldn't provide a
    // command that would read out the values in both encoders simultaneously
    // but they don't. Solution, anyone?
    
    //printf("%f	%f\n",(double)(encoder1_count),(double)(encoder2_count));
    
    rpm1 = (((double) encoder1_count) / ((double) encoder_ppr)) / gear_ratio;
    rpm2 = (((double) encoder2_count) / ((double) encoder_ppr)) / gear_ratio;
    
    distance1 = rpm1 * wheel_circumference;
    distance2 = rpm2 * wheel_circumference;
    
    if (invert_directions == true) {
        distance1 = -distance1;
        distance2 = -distance2;
    }
    
    distance_diff = distance1 - distance2;
    
    // We make an approximation here. Technically, if the speed in the wheels was different
    // then the robot was turning in a circle. Which means I would have to try and calculate
    // the arc but I really don't want to so I assume the motion is linear. If the position is
    // updated often enough this really shouldn't matter, but then again, why are you
    // relying on this data to be accurate? I would also need an accurate time since the last
    // measurement which is rather difficult to obtain accurately (preemption, anyone?).
    
    if (fabs(distance1) > fabs(distance2)) {
        
        count_enc_1 += encoder1_count;
        count_enc_2 += encoder2_count;
        
        current_position.linear.x += encoder2_count * 0.000048635 * 0.25;
        current_position.linear.y += encoder1_count * 0.000048635 * 0.25;
    } else {
        
        // uma volta - 175594
        count_enc_1 += encoder1_count;
        count_enc_2 += encoder2_count;
        
        current_position.linear.x += encoder2_count * 0.000048635 * 0.25;
        current_position.linear.y += encoder1_count * 0.000048635 * 0.25;
    }
    
    current_position.angular.z += (distance_diff / turning_circumference) * (2.0 * M_PI);
    current_position.angular.z = fmod(current_position.angular.z, (2.0 * M_PI)); // Constrain the angle to be within 2*pi.
    
}

int ConvertEncoder(int ret) {
    
    int k, j, t;
    char enc_buff[ENC_BUFF_SIZE];
    int enc_count;
    int stop_pos;
    
    if (ret >= 5) {
        
        if (('0' <= serialin_buff[4]) && (serialin_buff[4] <= '7')) {
            
            // printf("ret POS %d\n", ret);
            
            serialin_buff[SERIAL_BUFF_SIZE - 1] = 0x00;
            ret = sscanf(serialin_buff, "%*3c %X", &enc_count);
            if (ret != 1) {
                return -1;
            }
            
            enc_count = -enc_count;
        } else {
            
            // printf("ret NEG %d\n", ret);
            
            // 13 is the max number of chars to read representing a number in the form of 8000 0000
            // the four first chars are to be ignored regarding the actual encoder value
            // In the end there is a char that is also ignored
            
            stop_pos = ENC_BUFF_SIZE - (ret - 5); //size of the buf minus the nr of chars to read
            
            //  printf("stop pos %d ret %d\n",stop_pos,ret);
            for (t = 0; t < stop_pos; t++) {
                enc_buff[t] = 'F';
            }
            
            // printf("enc buf %s\n",enc_buff);
            
            k = 4; // move the index to the start of data on serialin_buff
            if (t <= ENC_BUFF_SIZE) {
                for (j = t; j < ENC_BUFF_SIZE; j++) {
                    enc_buff[j] = serialin_buff[k];
                    k++;
                    //printf("enc buf %s t %d j %d k %d\n",enc_buff,t,j,k);
                }
            }
            
            serialin_buff[SERIAL_BUFF_SIZE - 1] = 0x00;
            ret = sscanf(enc_buff, "%X", &enc_count);
            if (ret != 1) {
                return -1;
            }
            // printf("enc_count converted %X\n",enc_count);
            
            enc_count = ~enc_count;
            
            // printf("Negative enc_count converted %X\n",enc_count);
            enc_count += 1;
            
            //printf("Negative enc_count converted %X\n",enc_count);
        }
    }
    //else
    //   printf("\n Lower than 5!\n\n");
    
    return enc_count;
}


void cmd_velCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    if(setup_ok==true){
        forward_velocity_x=msg->linear.x;
        turnrate_factor=msg->linear.y;
        rotational_velocity=msg->angular.z;
        
        ROS_DEBUG("Forward_velocity %f turnrate_factor %f rotational_velocity %f", forward_velocity_x, turnrate_factor, rotational_velocity);
        WriteMotorVelocity(forward_velocity_x, turnrate_factor, rotational_velocity);
        if (encoder_present != 0)
            UpdatePositionData();
    }
}

void encoder_msgCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    if(setup_ok==true){
        encoder_angle=msg->angular.z;
        ROS_DEBUG("Encoder SteerAngle %f", encoder_angle);
    }
}


void mySigintHandler(int sig){
    if(setup_ok)
    MainQuit();
    ros::shutdown();
}

int main(int argc, char** argv) {
    double old_Kp;
    double old_Ki;
    double old_Kd;
    bool pid_changed=false;
    
    //Node Setup
    ros::init(argc, argv, "roboteq");
    ros::NodeHandle n;
    
    //Cmd_vel Subscriber
    ros::Subscriber cmd_vel_sub=n.subscribe("roboteq_cmd_vel", 1, cmd_velCallback);
    
    //Encoder Subscriber
    ros::Subscriber encoder_msg_sub=n.subscribe("io_steer_angle", 1, encoder_msgCallback);
    
    //roboteq RAW vel Publisher
    ros::Publisher raw_data_pub = n.advertise<geometry_msgs::Twist>("roboteq_raw_vel", 1);
    
    //roboteq Estimated Position Publisher
    ros::Publisher estimated_pos_pub = n.advertise<geometry_msgs::Twist>("roboteq_estimated_pos", 1);
    
    //Init default Parameters
    ROS_INFO("Starting roboteq and setting default parameters");
    init_default_parameters();
    setup_ok=false;
    
    //Spinner
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    //Get Parameter
    std::string devicepath_str;
    n.param<std::string>("/roboteq/devicepath", devicepath_str, "/dev/ttyUSB0");
    devicepath=devicepath_str.c_str();  //Convert string to const char*
    n.param("/roboteq/roboteq_baud", roboteq_baud, ROBOTEQ_DEFAULT_BAUD);
    n.param("/roboteq/acceleration", acceleration, DEFAULT_ACCELERATION);
    n.param("/roboteq/rc_mode_on_shutdown", rc_mode_on_shutdown, DEFAULT_RC_MODE_ON_SHUTDOWN);
    n.param("/roboteq/gear_ratio", gear_ratio, 87.967);
    n.param("/roboteq/axle_length", axle_length, 0.54);
    n.param("/roboteq/wheel_circumference", wheel_circumference, 2.135);
    n.param("/roboteq/encoder_ppr", encoder_ppr, 500);
    n.param("/roboteq/encoder_time_base", encoder_time_base, 2);
    n.param("/roboteq/encoder_distance_divider", encoder_distance_divider, 63);
    n.param("/roboteq/invert_directions", invert_directions, false);
    n.param("/roboteq/Kp", Kp, 3.0);
    n.param("/roboteq/Ki", Ki, 0.6);
    n.param("/roboteq/Kd", Kd, 0.05);
    n.param("/roboteq/iMax", pid.iMax, 10.0);
    n.param("/roboteq/iMin", pid.iMin, -10.0);
    n.param("/roboteq/controller_current_limit", controller_current_limit, DEFAULT_CONTROLLER_CURRENT_LIMIT);
    
    
    if(roboteq_baud < 0.0)
    ROS_WARN("Must specify positive value for roboteq_baud");
        
    if ((Kp < 0 || Kp > MAX_PID_GAIN) || (Ki < 0 || Ki > MAX_PID_GAIN) || (Kd < 0 || Kd > MAX_PID_GAIN))
        ROS_WARN("Invalid PID gain parameter(s)");
    
    if(acceleration < 0.0)
    ROS_WARN("Must specify positive value for acceleration");
        
    if(pid.iMax < 0.0)
    ROS_WARN("Must specify positive value for iMax");
        
    if(pid.iMin > 0.0)
    ROS_WARN("Must specify negative value for iMin");
        
    if(controller_current_limit < 0.0)
    ROS_WARN("Must specify positive value for controller_current_limit");
        
    if (wheel_circumference < 0.0)
        ROS_WARN("Invalid wheel_circunference");
    
    if (gear_ratio < 0.0)
        ROS_WARN("Invalid gear_ratio");
    
    if (axle_length < 0.0)
        ROS_WARN("Invalid axle_length");
    
    if (encoder_ppr < 0.0)
        ROS_WARN("Invalid encoder_ppr");
    
    if (encoder_time_base < 0.0)
        ROS_WARN("Invalid encoder_time_base");
    
    if (encoder_distance_divider < 0)
        ROS_WARN("Invalid encoder_distance_divider");
    
    while(ros::ok()){
        old_Kp=Kp;
        old_Ki=Ki;
        old_Kd=Kd;
        
        n.getParam("/roboteq/Kp", Kp);
        n.getParam("/roboteq/Ki", Ki);
        n.getParam("/roboteq/Kd", Kd);
        
        if(Kp!=old_Kp)
        pid_changed=true;
        if(Ki!=old_Kp)
        pid_changed=true;
        if(Kp!=old_Kp)
        pid_changed=true;
        
        if(pid_changed){
            if ((Kp < 0 || Kp > MAX_PID_GAIN) || (Ki < 0 || Ki > MAX_PID_GAIN) || (Kd < 0 || Kd > MAX_PID_GAIN)){
                ROS_WARN("Invalid PID gain parameter(s)");
                Kp=old_Kp;
                Ki=old_Ki;
                Kd=old_Kd;
            }
            else
                Set_channels_PIDvalues;
        }
        
        if(!setup_ok){
            //roboteq Initialization
            if(MainSetup()==-1){
                //ROS_ERROR("Unable to init roboteq");
                ros::Duration(1).sleep();
                //return -1;
            }else{
                ROS_INFO("Roboteq initialized successfully");
            }
        }else{
            // Publish position_data
            raw_data_pub.publish(position_data);
            ROS_DEBUG("Sending RAW data: X: %f Y: %f Z: %f", position_data.linear.x, position_data.linear.y, position_data.angular.z);
            
            // Publish current_position
            estimated_pos_pub.publish(current_position);
            ROS_DEBUG("Estimated Position: X: %f Y: %f Z: %f", current_position.linear.x, current_position.linear.y, current_position.angular.z);
        }
        signal(SIGINT, mySigintHandler);
        
        /*
        if(!ros::ok()){
        MainQuit();
        break;
        }
        */
        
    }
    spinner.stop();
    return 0;
}
