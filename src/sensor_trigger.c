#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include "pigpiod_if2.h"
#include <wiringPi.h>
#include <sys/time.h>

#define PWM_IN 23

static uint32_t prev_rise_tick = 0;  // Previous pulse rise time tick value
static uint32_t period_in = 0;           // Calculated period (us)
static bool sync_flag = FALSE;
static int flag_cnt = 0;
static int callback_id;
static bool meas_flag = FALSE;
static int cnt = 0;
int pulse_width;
int period_out;
int multiple = 3; 
float duty_out = 0.5; // 0 ~ 1
struct timeval sync_time, ex_time, curr_time;



void rising_flag(void){
    sync_flag = TRUE;
    gettimeofday(&sync_time, NULL);
}




// Callback function for measuring PWM input
void pwm_cbfunc(int pi, unsigned user_gpio, unsigned level, uint32_t tick) {
    static bool first_edge = true;

    if (level == 1) {  // rising edge
        if (!first_edge) {
            period_in = (tick - prev_rise_tick) ;  // Calculating period
            cnt +=1;
        }
        prev_rise_tick = tick;
    }

    else if (level == 0 && first_edge) {  // falling edge for first edge
        period_in = tick - prev_rise_tick;  // Calculating period
        prev_rise_tick = tick;
        first_edge = false;
        //wiringPiISR(pwm_in, INT_EDGE_RISING, &flag_handle);
    }

    if (cnt >= 2) {
        
        period_out = (period_in / multiple) ;
        pulse_width = period_out * duty_out;
        
        /*
        printf("Pulse Start \n");
        printf("Input PWM period: %u us \n", period_in);
        printf("Output Pulse Period: %d \n", period_out);
        printf("Output Pulse width: %d \n", pulse_width);
        */
        callback_cancel(callback_id);
        meas_flag = TRUE;
    }
}



int main(int argc, char **argv)
{
    long elapsed_us = 0;    
    int offset_time = 10000; //u-second
    bool offset_flag = FALSE;
    int i;
    int pulse_cnt = 0;
    
    wiringPiSetupGpio();
    pinMode(13, OUTPUT);
    digitalWrite(13, LOW);
    pinMode(PWM_IN, INPUT);
    gettimeofday(&ex_time, NULL);   

    int pi = pigpio_start(0, 0);
    if (pi < 0) {
        fprintf(stderr, "pigpio initialization failed (%d)\n", pi);
        return pi;
    }
    
    // Set up callback for PWM input 
    callback_id =  callback(pi, PWM_IN, EITHER_EDGE, pwm_cbfunc);
    wiringPiISR(PWM_IN, INT_EDGE_RISING, &rising_flag);
    //wiringPiISR(PWM_IN, INT_EDGE_FALLING, &off);
    //sync_flag = TRUE;


    while(!sync_flag && !meas_flag) {}
      
    sync_flag = FALSE;
    

    
    while (1) {
        i = 0;
        
        while (!sync_flag) {}
        
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
                digitalWrite(13, HIGH);
            }
            else if (elapsed_us <= period_out * 0.999){
                digitalWrite(13, LOW);
            }
            else {
                digitalWrite(13, LOW);
                ex_time = curr_time;
                i++;
                pulse_cnt++;
            }
        }
    }
}

