#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include <wiringPi.h>
#include <sys/time.h>
#include "ros/ros.h"


#define PWM_IN 23
#define PWM_OUT1 13
#define PWM_OUT2 6
#define PWM_OUT3 5
#define PWM_OUT4 0
#define PWM_OUT5 22
#define PWM_OUT6 27
#define PWM_OUT7 17


static bool sync_flag = FALSE;
static bool meas_flag = FALSE;
static int cnt = 0;
struct timeval sync_time, ex_time, curr_time;



void sync_flag_handle(void){
    sync_flag = TRUE;
    gettimeofday(&sync_time, NULL);
}



void measure_handle(void){
    gettimeofday(&sync_time, NULL);
    sync_flag = TRUE;
    cnt++;
}



void pinmode_setup() {
    pinMode(PWM_IN, INPUT);
    pinMode(PWM_OUT1, OUTPUT);
    pinMode(PWM_OUT2, OUTPUT);
    pinMode(PWM_OUT3, OUTPUT);
    pinMode(PWM_OUT4, OUTPUT);
    pinMode(PWM_OUT5, OUTPUT);
    pinMode(PWM_OUT6, OUTPUT);
    pinMode(PWM_OUT7, OUTPUT);
}


void pinout_write(int state) {
    digitalWrite(PWM_OUT1, state);
    digitalWrite(PWM_OUT2, state);
    digitalWrite(PWM_OUT3, state);
    digitalWrite(PWM_OUT4, state);
    digitalWrite(PWM_OUT5, state);
    digitalWrite(PWM_OUT6, state);
    digitalWrite(PWM_OUT7, state);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "sensor_trigger_node");
    ros::NodeHandle nh;


    uint32_t period_in1; // Calculated period (us)
    uint32_t period_in2; // Calculated period (us)
    long elapsed_us = 0;    
    int offset_time ; //u second
    int i;
    int pulse_width;
    int period_out;
    int multiplier; 
    float duty_out; // 0 ~ 1

    
    nh.param("/sensor_trigger/multiplier", multiplier, 3);
    nh.param("/sensor_trigger/duty_out", duty_out, 0.5f);
    nh.param("/sensor_trigger/offset_time", offset_time, 10000);

    
    ROS_INFO("-------------------------- ");
    ROS_INFO("<Parameters Setting>");
    ROS_INFO("Input Pin: %d", PWM_IN);
    ROS_INFO("Multplier: %d", multiplier);
    ROS_INFO("Duty Out: %.2f", duty_out);
    ROS_INFO("Offset Time: %d us", offset_time);
    ROS_INFO("-------------------------- ");


    wiringPiSetupGpio();
    pinmode_setup();
    pinout_write(LOW);
    
    gettimeofday(&ex_time, NULL);   
    
    wiringPiISR(PWM_IN, INT_EDGE_RISING, &measure_handle);

    ROS_INFO("Starting Input Measurement ");
    
    
    while(!meas_flag) { // Wait until input signal measurement is done
        if (sync_flag && cnt == 2) {
            ex_time = sync_time;
            sync_flag = FALSE;
        }

        else if (sync_flag && cnt == 3) {
            curr_time = sync_time;
            period_in1 = (curr_time.tv_sec - ex_time.tv_sec) * 1000000 +
                    (curr_time.tv_usec - ex_time.tv_usec);
        }

        else if (sync_flag && cnt == 4) {
            period_in2 = (sync_time.tv_sec - curr_time.tv_sec) * 1000000 +
                    (sync_time.tv_usec - curr_time.tv_usec);
            
            period_in1 = (period_in1 + period_in2) / 2; //average
            meas_flag = TRUE;

            ROS_INFO("Input Measurement Finished");
        }
    }

    wiringPiISR(PWM_IN, INT_EDGE_RISING, &sync_flag_handle);

    sync_flag = FALSE;
    
    period_out = (period_in1 / multiplier) ;
    pulse_width = period_out * duty_out;
        
    ROS_INFO("-------------------------- ");
    ROS_INFO("<Measurement Result>");
    ROS_INFO("Input Signal period: %u us", period_in1);
    ROS_INFO("Input Signal Frequency: %u Hz", 1000000 / period_in1);
    ROS_INFO("Output Signal Period: %d us", period_out);
    ROS_INFO("Output Signal Frequency: %d Hz", 1000000 / period_out);
    ROS_INFO("Output Signal Pulse Width: %d us", pulse_width);
    ROS_INFO("Output Signal Offset: %d us", offset_time);
    ROS_INFO("-------------------------- ");

    while (ros::ok()) {
        while (!sync_flag) {} // Wait until sync flag is True

        i = 0;
        sync_flag = FALSE;
        
        gettimeofday(&curr_time, NULL);
        elapsed_us = (curr_time.tv_sec - sync_time.tv_sec) * 1000000 +
                    (curr_time.tv_usec - sync_time.tv_usec);
        
        
        while (elapsed_us <= offset_time) {
            gettimeofday(&curr_time, NULL);
            elapsed_us = (curr_time.tv_sec - sync_time.tv_sec) * 1000000 +
                        (curr_time.tv_usec - sync_time.tv_usec);
        }
        
        ex_time = curr_time;
        
        while (i < 3) {
            gettimeofday(&curr_time, NULL);
            elapsed_us = (curr_time.tv_sec - ex_time.tv_sec) * 1000000 +
                         (curr_time.tv_usec - ex_time.tv_usec);
 
            if (elapsed_us <= pulse_width) {
                pinout_write(HIGH);
            }

            else if (elapsed_us <= period_out * 0.999){
                pinout_write(LOW);
            }

            else {
                pinout_write(LOW);
                ex_time = curr_time;
                i++;
            }
        }
    }
}



