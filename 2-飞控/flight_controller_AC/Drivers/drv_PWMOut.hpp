#pragma once

#include <stdint.h>

void set_MainMotorCount( uint8_t count );
void MainMotor_PWM_Out( double out[8] );
void Aux_PWM_Out( double out, uint8_t ind );

void MainMotor_PullDownAll();
void PWM_PullDownAll();
void PWM_PullUpAll();

void init_drv_PWMOut();