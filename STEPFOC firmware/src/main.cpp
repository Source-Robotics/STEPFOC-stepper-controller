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
#include <SPI.h>
#include "common.h"
#include "foc.h"
#include "temperature_table.h"
#include <Wire.h>
#include <I2C_eeprom.h>
#include "communication.h"
#include "EEPROM.h"
#include "CAN.h"
#include "communication_CAN.h"
#include "bootloader_config.h"
#include <IWatchdog.h>

// Define serial port
#ifdef Serial
#undef Serial // Arduino core's WSerial.h also defines Serial (as Serial2); this board uses Serialx
#endif
#define Serial Serialx
HardwareSerial Serialx(RX_COM, TX_COM); // PA3, PA2 RX,TX

// Define timer for PWM
HardwareTimer *MyTim2;
ADC_HandleTypeDef hadc1;

void setup()
{
  Bootloader_SetVectorTable();

  SPI.setMOSI(MOSI);
  SPI.setMISO(MISO);
  SPI.setSCLK(CLK);
  SPI.setClockDivider(SPI_CLOCK_DIV8); // High speed (128 / 8 = 16 MHz SPI_1 speed)
  Init_Digital_Outputs();
  Init_Digital_Inputs();

  // Init/Read Hal sensors
  controller.additional1_var = digitalReadFast(ADDITIONAL1);
  controller.additional2_var = digitalReadFast(ADDITIONAL2);

  // Init system
  HAL_Init();
  SystemClock_Config();
  TM_SystemClock_Config(CLOCK_PRESCALE); // 15, Clock speed = 8*14 = 112Mhz

  // Init ADC
  MX_ADC1_Init();
  HAL_ADCEx_Calibration_Start(&hadc1);

  // Begin serial
  Serial.begin(SERIAL_SPEED);

  digitalWrite(EEPROM_WP, LOW);
  Init_EEPROM();
  // Write_config();
  read_config();
  delay(100);

  // Begin PWM 50000 = 25Khz -> center aligned
  // PWM duty for 25khz goes from 0 - 5120 if CPU clock is 128 Mhz
  // PWM resolution = 112Mhz / 25khz = 4480; Scales to 2'13 on the pwm hardware
  MyTim2 = pwm_high(PWM_CH1, PWM_FREQ * 2);
  MyTim2 = pwm_high(PWM_CH2, PWM_FREQ * 2);
  MyTim2 = pwm_high(PWM_CH3, PWM_FREQ * 2);
  MyTim2 = pwm_high(PWM_CH4, PWM_FREQ * 2);
  pwm_align(MyTim2);

  // Get initial encoder value!
  Get_first_encoder();

  // Collect data before entering the ISR
  // This fixes velocity error
  for (int i = 0; i < 20; i++)
  {
    Collect_data();
    delayMicroseconds(20);
  }

  // Init our control loop
  Ticker_init(TIM3, LOOP_FREQ, IT_callback);

  delay(20);

  Setup_CAN_bus();

  // Hardware watchdog: resets the board if the control ISR ever stops running
  // (e.g. a wedged loop) instead of leaving PWM driving stale duty indefinitely.
  // Kicked every tick from IT_callback()/Update_IT_callback_calib(), so 100ms is
  // a wide margin under normal operation but catches a real hang quickly.
  IWatchdog.begin(100000);
}

//////////////////////////////////////////////////////////////////////////////

void loop()
{

  uint32_t ms = HAL_GetTick();

  if (controller.LED_ON_OFF == 1)
  {

    LED_status(ms);
  }

  static uint32_t last_time = 0;

  if ((ms - last_time) > 500) // run every x ms
  {
    // The angle-offset lock+sweep (Calibrate_Angle_Offset_Align) can run for tens of
    // seconds with no other serial output; let the user know it's still progressing.
    if (controller.Align_stage > 0)
    {
      Serial.print("Calibrating angle offset... stage ");
      Serial.print(controller.Align_stage);
      Serial.print("/13, sweep point ");
      Serial.println(controller.Align_point);
    }

    last_time = ms;
  }

  /*
   HANDLE CALIBRATION REPORT
  */
  Calib_report(Serial);
  /*
   UART CLI INTERFACE
  */
  UART_protocol(Serial);
  Cyclic_UART(Serial, ms);
  /*
  HANDLE CAN BUS
  */
  if (controller.CAN_init_error == 0)
  {
    CAN_protocol(Serial);
    Capture_stream_CAN();
    CAN_heartbeat(ms);
    CAN_watchdog(ms);
  }
}
