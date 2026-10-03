/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    common.cpp
  * @brief   This file provides code where we define existance of global structures
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/

#include "common.h"

/// Global structure declaration
Measure controller;
_FOC FOC;
int16_t Capture_vel[CAPTURE_LEN];
int16_t Capture_iq[CAPTURE_LEN];
int16_t Capture_phase[CAPTURE_LEN];
volatile uint8_t Velocity_window = VELOCITY_WINDOW_DEFAULT;
volatile uint8_t Ripple_harmonic[RIPPLE_SLOTS];
volatile int16_t Ripple_a[RIPPLE_SLOTS];
volatile int16_t Ripple_b[RIPPLE_SLOTS];
PID_par PID;
GRIPPER_STRUCT Gripper;