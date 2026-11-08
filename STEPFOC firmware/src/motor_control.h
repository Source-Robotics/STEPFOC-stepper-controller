/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    motor_control.h
  * @brief   This file provides code for all the function prototypes for motor_control.cpp
  * @author Petar Crnjak
  ******************************************************************************
  * @attention
  *
  * Copyright (c) Source robotics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

 /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MOTOR_CONT_
#define MOTOR_CONT_

#include <Arduino.h>
#include <stdio.h>
#include "hw_init.h"
#include "pwm_init.h"
#include "utils.h"
#include "stm32f1xx_hal.h"
#include "adc_init.h"
#include "iodefs.h"
#include "qfplib-m3.h"
#include "SerialPacketParser.h"
#include "constants.h"
#include "motor_control.h"
#include "common.h"
#include "temperature_table.h"


const float CURRENT_SENSE_CONSTANT = ((ADC_MIDPOINT / ADC_MIDPOINT_BIT) / SENSE_RESISTOR) / CURRENT_AMP_GAIN;
const float CURRENT_SENSE_CONSTANT_mV = ((ADC_MIDPOINT / ADC_MIDPOINT_BIT) / SENSE_RESISTOR) / CURRENT_AMP_GAIN * 1000;
//const int VOLTAGE_SENSE_CONSTANT = 

bool parityCheck(uint16_t data);
void Collect_data();
float Get_current(int adc_value);
int Get_voltage_mV(int adc_value);
int Get_current_mA(int adc_value);
int Get_ADC_Value(int current_mA);
void Position_mode();
void Velocity_mode();
void Torque_mode();
void Update_IT_callback_calib();
void IT_callback(void);
void Sense_zero_calibrate();
void Enable_drive(void);
void Calib_report(Stream &Serialport);
void PD_mode();
void Open_loop_speed_stepper(float speed, int voltage);
void Open_loop_move_steps(int steps, int voltage, int microsteps_per_cycle, int step_interval_us);
void LED_status(uint32_t ms);
void Phase_order();
void Collect_data2();
void Gripper_mode();
void Calibrate_gripper();
void Get_first_encoder();
void Voltage_Torque_mode();
void Brake_Coast();
void Brake_Coast_As(bool brake);
void Hall_Indexing_mode();
int Calibrate_Angle_Offset();
int Calibrate_Angle_Offset2();
int Calibrate_Angle_Offset_Align();
void Motion_wake(); // what a motion frame does to the bridge before it moves (cogcal.cpp)
bool Current_gains_from_RL();       // current PI gains from R, L and current_control_bandwidth
float Current_bandwidth_in_force(); // the bandwidth, Hz, the current gains in force give
int Speed_estimate(int velocity_raw, int d_ticks); // the speed the loops use (parameter 50 picks it)
void Speed_pll_gains();            // the speed PLL's gains from speed_pll_bandwidth
bool Sense_trim_from_h1(float a, float b, int32_t *d1, int32_t *d2); // zero errors behind a 1st harmonic (cogcal)
extern float pwm_per_mV;           // PWM_MAX / Vbus, set where Vbus is read (Collect_data, Collect_data2)
int Pwm_from_volts(float u);       // PWM_MAX x (1 - |u| / Vbus), with this loop's factor
void Loop_profile_add(uint32_t t0, uint32_t t1, uint32_t t2, uint32_t t3, uint32_t t4, uint32_t t5);
int32_t Loop_profile_x10(int k);   // a section's average time, 0.1 us (parameters 66-70)
uint16_t Encoder_spi16(uint16_t out); // the encoder's SPI, register path when parameter 71 is on
void Loop_profile_begin();         // the DWT cycle counter on (setup)
void Comm_delay_update();          // the commutation delay rotation for this loop (parameter 64)
float Velocity_pi(float err_p, float err_i); // the velocity PI, clamped (batch 18)
float Vel_int_setpoint(float position_error, float speed_ff); // the integral's speed setpoint, deadband 75
void Input_filter(int32_t pos_in, float vel_in, float *position_error, float *speed); // parameter 74
void Input_filter_reset();
extern float Cal_saved_current_limit; // the current limit when #Cal started (batch 22d)
void Cal_restore_current_limit();     // put it back when the calibration ends
void Foc_frame(float theta);       // Id, Iq and the inverse transform at this electrical angle (encal.cpp)


#endif
