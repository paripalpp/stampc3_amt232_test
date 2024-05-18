#pragma once
#include "Arduino.h"
#include <esp_timer.h>

struct pid_param_t
{
    float k_p;
    float k_i;
    float k_d;
};


class PID
{
private:
    float integral;
    float error_prev;
    int64_t t_prev_us;

    pid_param_t gain;
    float integral_max;
    bool circular;
    float circular_min;
    float circular_max;
public:
    PID(float k_i, float k_p, float k_d, float integral_max, float first_feedback, float first_target, bool circular, float circular_min, float circular_max);
    PID(pid_param_t gain, float integral_max, float first_feedback, float first_target, bool circular, float circular_min, float circular_max);
    float process(float feedback, float target);
    void reset();
    pid_param_t get_param();
    void set_params(pid_param_t gain);
};

PID::PID(float k_i, float k_p, float k_d, float integral_max = 1, float first_feedback = 0, float first_target = 0, bool circular = false, float circular_min = 0, float circular_max = 1)
{
    this->gain.k_i = k_i;
    this->gain.k_p = k_p;
    this->gain.k_d = k_d;
    this->integral_max = integral_max;
    this->circular = circular;
    this->circular_min = circular_min;
    this->circular_max = circular_max;
    this->t_prev_us = esp_timer_get_time();
    this->error_prev = first_target - first_feedback;
}

PID::PID(pid_param_t gain, float integral_max = 1, float first_feedback = 0, float first_target = 0, bool circular = false, float circular_min = 0, float circular_max = 1)
{
    this->gain = gain;
    this->integral_max = integral_max;
    this->circular = circular;
    this->circular_min = circular_min;
    this->circular_max = circular_max;
    this->t_prev_us = esp_timer_get_time();
    this->error_prev = first_target - first_feedback;
}

float PID::process(float feedback, float target){
    int64_t t_us = esp_timer_get_time();
    float delta = (float)(t_us - t_prev_us) * 1.0E-6f;
    float error;
    if(this->circular){
        error = target - feedback;
        error = (error > (this->circular_max - this->circular_min)/2) ? error - (this->circular_max - this->circular_min) : (error < -(this->circular_max - this->circular_min)/2) ? error + (this->circular_max - this->circular_min) : error;
    }else{
        error = target - feedback;
    }
    this->integral += error * delta;
    if(this->integral > this->integral_max) this->integral = this->integral_max;
    if(this->integral < -this->integral_max) this->integral = -this->integral_max;    
    float difference = (error - error_prev) / delta;
    this->t_prev_us = t_us;
    return this->gain.k_p * error + this->gain.k_i * this->integral + this->gain.k_d * difference;
}

void PID::reset(){
    this->integral = 0;
    this->t_prev_us = esp_timer_get_time();
    this->error_prev = 0;
}

pid_param_t PID::get_param(){
    return this->gain;
}

void PID::set_params(pid_param_t gain){
    this->gain = gain;
}
