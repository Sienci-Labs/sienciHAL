/*
  flexi_hal_map.h - driver code for STM32F4xx ARM processors

  Part of grblHAL

  Copyright (c) 2022 Expatria Technologies

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#if N_ABC_MOTORS > 2
#error "Axis configuration is not supported!"
#endif

#if !defined(STM32F412Vx) || HSE_VALUE != 25000000
#error "This board has STM32F446 processor with a 25MHz crystal, select a corresponding build!"
#endif

#define BOARD_NAME "Longboard32"
#define BOARD_URL "Sienci Longboard32"

#undef I2C_ENABLE

#define I2C_ENABLE 1
#define I2C_FASTMODE
#define HAS_IOPORTS

#define HAS_BOARD_INIT


#if MODBUS_ENABLE
#define SERIAL2_MOD 3
#define MODBUS_SERIAL_PORT 2
#endif

#if MPG_MODE == 1
#define MPG_MODE_PORT           GPIOA
#define MPG_MODE_PIN            4
#endif

// Define step pulse output pins.
#define X_STEP_PORT             GPIOE
#define X_STEP_PIN              2
#define Y_STEP_PORT             GPIOE
#define Y_STEP_PIN              3
#define Z_STEP_PORT             GPIOE
#define Z_STEP_PIN              4
#define STEP_OUTMODE            GPIO_BITBAND
//#define STEP_PINMODE            PINMODE_OD // Uncomment for open drain outputs

// Define step direction output pins.
#define X_DIRECTION_PORT        GPIOE
#define X_DIRECTION_PIN         7
#define Y_DIRECTION_PORT        GPIOE
#define Y_DIRECTION_PIN         8
#define Z_DIRECTION_PORT        GPIOE
#define Z_DIRECTION_PIN         9
#define DIRECTION_OUTMODE       GPIO_BITBAND
//#define DIRECTION_PINMODE       PINMODE_OD // Uncomment for open drain outputs

// Define stepper driver enable/disable output pin.
#define X_ENABLE_PORT               GPIOD
#define X_ENABLE_PIN                4
#define Y_ENABLE_PORT               GPIOC
#define Y_ENABLE_PIN                7
#define Z_ENABLE_PORT               GPIOD
#define Z_ENABLE_PIN                3
//#define STEPPERS_ENABLE_PORT       GPIOA
//#define STEPPERS_ENABLE_PIN        14
#define STEPPERS_ENABLE_OUTMODE    GPIO_BITBAND

// Define homing/hard limit switch input pins.
#define X_LIMIT_PORT            GPIOB
#define X_LIMIT_PIN             5
#define Y_LIMIT_PORT            GPIOB
#define Y_LIMIT_PIN             9
#define Z_LIMIT_PORT            GPIOC
#define Z_LIMIT_PIN             13
#define LIMIT_INMODE            GPIO_BITBAND

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS > 0
#define M3_AVAILABLE
#define M3_STEP_PORT            GPIOE
#define M3_STEP_PIN             5
#define M3_DIRECTION_PORT       GPIOE
#define M3_DIRECTION_PIN        10
#define M3_LIMIT_PORT           GPIOB
#define M3_LIMIT_PIN            6
#define M3_ENABLE_PORT          GPIOC
#define M3_ENABLE_PIN           7
#endif

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS == 2
#define M4_AVAILABLE
#define M4_STEP_PORT            GPIOE
#define M4_STEP_PIN             6
#define M4_DIRECTION_PORT       GPIOE
#define M4_DIRECTION_PIN        11
#define M4_LIMIT_PORT           GPIOE
#define M4_LIMIT_PIN            14
#define M4_ENABLE_PORT          GPIOC
#define M4_ENABLE_PIN           10
#endif

// Define spindle enable and spindle direction output pins.
#define SPINDLE_ENABLE_PORT     GPIOE
#define SPINDLE_ENABLE_PIN      13
#define SPINDLE_DIRECTION_PORT  GPIOB
#define SPINDLE_DIRECTION_PIN   1
#define SPINDLE_OUTMODE         GPIO_BITBAND

// Define spindle PWM output pin.
#define SPINDLE_PWM_PORT_BASE   GPIOA_BASE
#define SPINDLE_PWM_PIN         8

// Define flood and mist coolant enable output pins.
#define COOLANT_FLOOD_PORT      GPIOD
#define COOLANT_FLOOD_PIN       7
#define COOLANT_MIST_PORT       GPIOB
#define COOLANT_MIST_PIN        4
#define COOLANT_OUTMODE         GPIO_BITBAND

#define AUXOUTPUT0_PORT         GPIOC
#define AUXOUTPUT0_PIN          14
#define AUXOUTPUT1_PORT         GPIOC
#define AUXOUTPUT1_PIN          0
#define AUXOUTPUT2_PORT         GPIOC
#define AUXOUTPUT2_PIN          1
#define AUXOUTPUT3_PORT         GPIOC
#define AUXOUTPUT3_PIN          4

/*modbus direction pin*/
#define AUXOUTPUT4_PORT         GPIOB
#define AUXOUTPUT4_PIN          0
#define MODBUS_DIR_AUX          4

/* temporary defines for additional laser pins*/
#define AUXOUTPUT5_PORT         GPIOC //laser PWM pin
#define AUXOUTPUT5_PIN          6     //laser PWM pin
#define AUXOUTPUT6_PORT         GPIOC //laser EN pin
#define AUXOUTPUT6_PIN          12    //laser EN pin
#define AUXOUTPUT_OUTMODE       GPIO_BITBAND

#define AUXINPUT0_PORT          GPIOD
#define AUXINPUT0_PIN           12
#define AUXINPUT1_PORT          GPIOD
#define AUXINPUT1_PIN           13
#define AUXINPUT2_PORT          GPIOD
#define AUXINPUT2_PIN           14
#define AUXINPUT3_PORT          GPIOD
#define AUXINPUT3_PIN           15

/*maco pins, aux for now*/
#define AUXINPUT4_PORT          GPIOE
#define AUXINPUT4_PIN           1
#define AUXINPUT5_PORT          GPIOE
#define AUXINPUT5_PIN           0

// Define user-control controls (cycle start, reset, feed hold) input pins.
#define RESET_PORT              GPIOB
#define RESET_PIN               12
#define FEED_HOLD_PORT          GPIOC
#define FEED_HOLD_PIN           8
#define CYCLE_START_PORT        GPIOC
#define CYCLE_START_PIN         11
/* no safety door*/


#define CONTROL_INMODE          GPIO_BITBAND

// Define probe switch input pin.
#define PROBE_PORT              GPIOB
#define PROBE_PIN               7

#if I2C_STROBE_ENABLE
#define I2C_STROBE_PORT         GPIOD
#define I2C_STROBE_PIN          10
#endif

#define TMC_STEALTHCHOP 0

// SPI2 is used: GPIOB pin 12 (SCK) GPIOC pin 2 (MISO) and 3 (MOSI)
#define MOTOR_CSX_PORT              GPIOD
#define MOTOR_CSX_PIN               11
#define MOTOR_CSY_PORT              GPIOE
#define MOTOR_CSY_PIN               15
#define MOTOR_CSZ_PORT              GPIOD
#define MOTOR_CSZ_PIN               9

#ifdef  M3_AVAILABLE
#define MOTOR_CSM3_PORT             GPIOE
#define MOTOR_CSM3_PIN              12
#endif

#if SDCARD_ENABLE
#define SD_CS_PORT              GPIOA
#define SD_CS_PIN               3
#define SPI_PORT                1 // GPIOA, SCK_PIN = 5, MISO_PIN = 6, MOSI_PIN = 7
#endif
