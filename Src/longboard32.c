/*
  longboard32.c - driver code for STM32F4xx ARM processors on SuperLongBoard board

  Part of grblHAL

   Copyright (C) Sienci Labs Inc.
  
   This file is part of the SuperLongBoard family of products.
  
   This source describes Open Hardware and is licensed under the "CERN-OHL-S v2"

   You may redistribute and modify this source and make products using
   it under the terms of the CERN-OHL-S v2 (https://ohwr.org/cern_ohl_s_v2.t). 
   This source is distributed WITHOUT ANY EXPRESS OR IMPLIED WARRANTY,
   INCLUDING OF MERCHANTABILITY, SATISFACTORY QUALITY AND FITNESS FOR A 
   PARTICULAR PURPOSE. Please see the CERN-OHL-S v2 for applicable conditions.
   
   As per CERN-OHL-S v2 section 4, should You produce hardware based on this 
   source, You must maintain the Source Location clearly visible on the external
   case of the CNC Controller or other product you make using this source.
  
   You should have received a copy of the CERN-OHL-S v2 license with this source.
   If not, see <https://ohwr.org/project/cernohl/wikis/Documents/CERN-OHL-version-2>.
   
   Contact for information regarding this program and its license
   can be sent through gSender@sienci.com or mailed to the main office
   of Sienci Labs Inc. in Waterloo, Ontario, Canada.
*/

#include "driver.h"

#if defined(BOARD_LONGBOARD32)

#include <math.h>
#include <string.h>

#include "main.h"
#include "i2c.h"
#include "grbl/protocol.h"
#include "grbl/settings.h"
#include "grbl/nvs_buffer.h"

#include "trinamic/tmc2660.h"

SPI_HandleTypeDef hspi2 = {0};
GPIO_InitTypeDef GPIO_InitStruct = {0};

TIM_MasterConfigTypeDef sMasterConfig = {0};
TIM_ClockConfigTypeDef sClockSourceConfig = {0};
TIM_IC_InitTypeDef sConfigIC = {0};

#if SLB_MOTOR_ALARM

TIM_HandleTypeDef htim2;

struct AlarmFunctionCall {
    GPIO_TypeDef* port;
    int pin;
};

struct AlarmFunctionCall alarmgpio[] = {
    {MOTOR_SGX_PORT, MOTOR_SGX_PIN}, 
    {MOTOR_SGY1_PORT, MOTOR_SGY1_PIN}, 
    {MOTOR_SGZ_PORT, MOTOR_SGZ_PIN},  
    {MOTOR_SGY2_PORT, MOTOR_SGY2_PIN},  
    {MOTOR_SGA_PORT, MOTOR_SGA_PIN}  
};

static on_execute_realtime_ptr on_execute_realtime, on_execute_delay;
static on_reset_ptr on_reset;

static uint32_t debounce_ms = 0;
static uint32_t polling_ms = 0;
#define DEBOUNCE_DELAY 25
#define ALARM_THRESHOLD 3

#define N_MOTOR_ALARMS 5

static int8_t val[N_MOTOR_ALARMS] = {0};
static bool motor_alarm_active = false;

typedef struct {
    axes_signals_t     enable;
    axes_signals_t     invert;
} motor_alarm_settings_t;

static axes_signals_t motor_alarm_pins;

static nvs_address_t nvs_address;
motor_alarm_settings_t motor_alarms;

static const setting_detail_t motor_alarm_settings[] = {
    { 744, Group_Stepper, "Motor Alarm enable", NULL, Format_AxisMask, NULL, NULL, NULL, Setting_IsExpanded, &motor_alarms.enable.mask, NULL, NULL},
    { 745, Group_Stepper, "Motor Alarm invert", NULL, Format_AxisMask, NULL, NULL, NULL, Setting_IsExpanded, &motor_alarms.invert.mask, NULL, NULL},
};

#ifndef NO_SETTINGS_DESCRIPTIONS

static const setting_descr_t motor_alarm_descriptions[] = {
    { 744, "Enables the motor alarm" },
    { 745, "Inverts motor alarm signal" },
};

#endif

// Hal settings API
// Restore default settings and write to non volatile storage (NVS).
static void motor_alarm_settings_restore (void)
{
    memset(&motor_alarms, 0, sizeof(motor_alarm_settings_t));
    motor_alarms.enable.value = 7;
    motor_alarms.invert.value = 0;

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&motor_alarms, sizeof(motor_alarm_settings_t), true);
}

// Write settings to non volatile storage (NVS).
static void motor_alarm_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&motor_alarm_settings, sizeof(motor_alarm_settings_t), true);
}

// Load settings from volatile storage (NVS)
static void motor_alarm_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&motor_alarm_settings, nvs_address, sizeof(motor_alarm_settings_t), true) != NVS_TransferResult_OK)
        motor_alarm_settings_restore();

}

static setting_details_t setting_details = {
    .settings = motor_alarm_settings,
    .n_settings = sizeof(motor_alarm_settings) / sizeof(setting_detail_t),
#ifndef NO_SETTINGS_DESCRIPTIONS
    .descriptions = motor_alarm_descriptions,
    .n_descriptions = sizeof(motor_alarm_descriptions) / sizeof(setting_descr_t),
#endif
    .save = motor_alarm_settings_save,
    .load = motor_alarm_settings_load,
    .restore = motor_alarm_settings_restore
};

#endif

#include "trinamic/common.h"

static SPI_HandleTypeDef spi_port = {
    .Instance = SPI2,
    .Init.Mode = SPI_MODE_MASTER,
    .Init.Direction = SPI_DIRECTION_2LINES,
    .Init.DataSize = SPI_DATASIZE_8BIT,
    .Init.CLKPolarity = SPI_POLARITY_LOW,
    .Init.CLKPhase = SPI_PHASE_1EDGE,
    .Init.NSS = SPI_NSS_SOFT,
    .Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32,
    .Init.FirstBit = SPI_FIRSTBIT_MSB,
    .Init.TIMode = SPI_TIMODE_DISABLE,
    .Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE,
    .Init.CRCPolynomial = 10
};

static struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} cs[TMC_N_MOTORS_MAX];

// XXXXX replace with something better...
inline static void delay (void)
{
    volatile uint32_t dly = 20;

    while(--dly)
        __ASM volatile ("nop");
}

static uint8_t tmc2660_spi_put_byte (uint8_t byte)
{
    spi_port.Instance->DR = byte;

    while(!__HAL_SPI_GET_FLAG(&spi_port, SPI_FLAG_TXE));
    while(!__HAL_SPI_GET_FLAG(&spi_port, SPI_FLAG_RXNE));

    __HAL_SPI_CLEAR_OVRFLAG(&spi_port);

    return (uint8_t)spi_port.Instance->DR;
}

TMC_spi_status_t tmc2660_spi_read (trinamic_motor_t driver, TMC2660_spi_datagram_t *safe_datagram, TMC2660_spi_datagram_t *datagram)
{
    TMC_spi_status_t status;
    uint32_t gram = 0;
    
    #if 1

    while(__HAL_SPI_GET_FLAG(&spi_port, SPI_FLAG_BSY));

    DIGITAL_OUT(cs[driver.id].port, cs[driver.id].pin, 0);
    delay();
    gram = ((safe_datagram->addr.value << 16) | safe_datagram->payload.value);
    datagram->payload.data[2] = tmc2660_spi_put_byte(gram>>16 & 0xFF);
    datagram->payload.data[1] = tmc2660_spi_put_byte(gram>>8 & 0xFF);
    datagram->payload.data[0] = tmc2660_spi_put_byte(gram & 0xFF);

    delay();
    DIGITAL_OUT(cs[driver.id].port, cs[driver.id].pin, 1);

    datagram->payload.value = (datagram->payload.value>>4) & 0xFFFFFF;

    // Initialize the result
    /*uint32_t result = 0;

    // Reverse the bits
    for (int i = 0; i < 20; i++) {
        // Shift the result to the left
        result <<= 1;

        // Extract the rightmost bit from the bottom20Bits and add it to the result
        result |= (datagram->payload.value & 1);

        // Shift the bottom20Bits to the right
        datagram->payload.value >>= 1;
    }

    datagram->payload.value = result;
    */

    status = 1;
    #endif
    return status;
}

TMC_spi_status_t tmc2660_spi_write (trinamic_motor_t driver, TMC2660_spi_datagram_t *datagram)
{
    TMC_spi_status_t status;
    uint32_t gram = 0;
#if 1
    while(__HAL_SPI_GET_FLAG(&spi_port, SPI_FLAG_BSY));
    DIGITAL_OUT(cs[driver.id].port, cs[driver.id].pin, 0);
    delay();
    //gram = ((datagram->addr.value << 16) | datagram->payload.value);
    gram = ((datagram->addr.value << 16) | datagram->payload.value);

    status = tmc2660_spi_put_byte(gram>>16 & 0xFF);
    status = tmc2660_spi_put_byte(gram>>8 & 0xFF);
    status = tmc2660_spi_put_byte(gram & 0xFF);

    delay();
    DIGITAL_OUT(cs[driver.id].port, cs[driver.id].pin, 1);

    return status;
#endif
}

static void add_cs_pin (xbar_t *gpio, void *data)
{
    if (gpio->group == PinGroup_MotorChipSelect)
      switch (gpio->function) {

        case Output_MotorChipSelectX:
            cs[X_AXIS].port = (GPIO_TypeDef *)gpio->port;
            cs[X_AXIS].pin = gpio->pin;
            break;

        case Output_MotorChipSelectY:
            cs[Y_AXIS].port = (GPIO_TypeDef *)gpio->port;
            cs[Y_AXIS].pin = gpio->pin;
            break;

        case Output_MotorChipSelectZ:
            cs[Z_AXIS].port = (GPIO_TypeDef *)gpio->port;
            cs[Z_AXIS].pin = gpio->pin;
            break;

        case Output_MotorChipSelectM3:
            cs[3].port = (GPIO_TypeDef *)gpio->port;
            cs[3].pin = gpio->pin;
            break;

        case Output_MotorChipSelectM4:
            cs[4].port = (GPIO_TypeDef *)gpio->port;
            cs[4].pin = gpio->pin;
            break;

        default:
            break;
    }
}

#if TRINAMIC_ENABLE

static void if_init (uint8_t motors, axes_signals_t enabled)
{
    static bool init_ok = false;

    UNUSED(motors);

    if (!init_ok) {

        __HAL_RCC_SPI2_CLK_ENABLE();

        GPIO_InitTypeDef GPIO_InitStruct = {
            .Pin = GPIO_PIN_2|GPIO_PIN_3,
            .Mode = GPIO_MODE_AF_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
            .Alternate = GPIO_AF5_SPI2
        };
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        GPIO_InitTypeDef GPIO_InitStruct2 = {
            .Pin = GPIO_PIN_13,
            .Mode = GPIO_MODE_AF_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
            .Alternate = GPIO_AF5_SPI2
        };
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct2);        

        static const periph_pin_t sck = {
            .function = Output_SCK,
            .group = PinGroup_SPI,
            .port = GPIOB,
            .pin = 13,
            .mode = { .mask = PINMODE_OUTPUT }
        };

        static const periph_pin_t sdo = {
            .function = Output_MOSI,
            .group = PinGroup_SPI,
            .port = GPIOC,
            .pin = 3,
            .mode = { .mask = PINMODE_NONE }
        };

        static const periph_pin_t sdi = {
            .function = Input_MISO,
            .group = PinGroup_SPI,
            .port = GPIOC,
            .pin = 2,
            .mode = { .mask = PINMODE_NONE }
        };

        HAL_SPI_Init(&spi_port);
        __HAL_SPI_ENABLE(&spi_port);

        hal.periph_port.register_pin(&sck);
        hal.periph_port.register_pin(&sdo);
        hal.periph_port.register_pin(&sdi);
        hal.enumerate_pins(true, add_cs_pin, NULL);
    }
}
#endif

#if SLB_MOTOR_ALARM

#if SLB_MOTOR_ALARM_IRQ

static void warning_msg_motor (uint_fast16_t state)
{
    GPIO_InitTypeDef GPIO_Init2 = {
        .Speed = GPIO_SPEED_FREQ_HIGH,
        .Mode = GPIO_MODE_INPUT,
        .Pull = GPIO_NOPULL
    };  

    HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_1);
    HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_2);
    HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_3);
    HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_4);

    //set all the pins to input
    GPIO_Init2.Pin = (1 << MOTOR_SGX_PIN);
    HAL_GPIO_Init(MOTOR_SGX_PORT, &GPIO_Init2);
    GPIO_Init2.Pin = (1 << MOTOR_SGY1_PIN);
    HAL_GPIO_Init(MOTOR_SGY1_PORT, &GPIO_Init2);
    GPIO_Init2.Pin = (1 << MOTOR_SGZ_PIN);
    HAL_GPIO_Init(MOTOR_SGZ_PORT, &GPIO_Init2);
    GPIO_Init2.Pin = (1 << MOTOR_SGY2_PIN);
    HAL_GPIO_Init(MOTOR_SGY2_PORT, &GPIO_Init2);
    GPIO_Init2.Pin = (1 << MOTOR_SGA_PIN);
    HAL_GPIO_Init(MOTOR_SGA_PORT, &GPIO_Init2);    

    //this read functions as IRQ debounce, additional filtering could be added as needed.
    if(!DIGITAL_IN(MOTOR_SGX_PORT, MOTOR_SGX_PIN)){
        report_message("Motor Error on X Axis!", Message_Warning);   
        system_set_exec_alarm(Alarm_MotorFault);
    }

    if(!DIGITAL_IN(MOTOR_SGY1_PORT, MOTOR_SGY1_PIN)){
        report_message("Motor Error on Y1 Axis!", Message_Warning);   
        system_set_exec_alarm(Alarm_MotorFault);
    }
    
    if(!DIGITAL_IN(MOTOR_SGZ_PORT, MOTOR_SGZ_PIN)){
        report_message("Motor Error on Z Axis!", Message_Warning);   
        system_set_exec_alarm(Alarm_MotorFault);
    }

    if(!DIGITAL_IN(MOTOR_SGY2_PORT, MOTOR_SGY2_PIN)){
        report_message("Motor Error on Y2 Axis!", Message_Warning);   
        system_set_exec_alarm(Alarm_MotorFault);
    }

    if(!DIGITAL_IN(MOTOR_SGA_PORT, MOTOR_SGA_PIN)){
        report_message("Motor Error on A Axis!", Message_Warning);   
        system_set_exec_alarm(Alarm_MotorFault);
    }              

    motor_alarm_pins.value = 0; //clear the alarm mask
    
    //reset the timer pins
    GPIO_Init2.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_Init2.Mode = GPIO_MODE_AF_PP;
    GPIO_Init2.Pull = GPIO_NOPULL;
    GPIO_Init2.Alternate = GPIO_AF1_TIM2;

    GPIO_Init2.Pin = (1 << MOTOR_SGX_PIN);
    HAL_GPIO_Init(MOTOR_SGX_PORT, &GPIO_Init2);
    GPIO_Init2.Pin = (1 << MOTOR_SGY1_PIN);
    HAL_GPIO_Init(MOTOR_SGY1_PORT, &GPIO_Init2);
    GPIO_Init2.Pin = (1 << MOTOR_SGZ_PIN);
    HAL_GPIO_Init(MOTOR_SGZ_PORT, &GPIO_Init2);
    GPIO_Init2.Pin = (1 << MOTOR_SGY2_PIN);
    HAL_GPIO_Init(MOTOR_SGY2_PORT, &GPIO_Init2);

    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);    

    //Configure GPIO pin : PD2   
    GPIO_Init2.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_Init2.Pull = GPIO_NOPULL;
    GPIO_Init2.Pin = (1 << MOTOR_SGA_PIN);
    HAL_GPIO_Init(MOTOR_SGA_PORT, &GPIO_Init2);
}


void EXTI2_IRQHandler(void)
{
    uint32_t ifg = __HAL_GPIO_EXTI_GET_IT(1<<2);
    if(ifg)
        __HAL_GPIO_EXTI_CLEAR_IT(ifg);

    protocol_enqueue_rt_command(warning_msg_motor);
    motor_alarm_pins.b = 1;

}

void TIM2_IRQHandler(void){

  /*           @arg TIM_IT_CC1:  Capture/Compare 1 interrupt
  *            @arg TIM_IT_CC2:  Capture/Compare 2 interrupt
  *            @arg TIM_IT_CC3:  Capture/Compare 3 interrupt
  *            @arg TIM_IT_CC4:  Capture/Compare 4 interrupt
  */  

    if   (__HAL_TIM_GET_FLAG(&htim2,TIM_IT_CC3)) {
        motor_alarm_pins.z = 1;
    } if (__HAL_TIM_GET_FLAG(&htim2,TIM_IT_CC4)) {
        motor_alarm_pins.a = 1;
    } if (__HAL_TIM_GET_FLAG(&htim2,TIM_IT_CC2)) {
        motor_alarm_pins.y = 1;
    } if (__HAL_TIM_GET_FLAG(&htim2,TIM_IT_CC1)) {
        motor_alarm_pins.x = 1;
    }

    protocol_enqueue_rt_command(warning_msg_motor);
    __HAL_TIM_CLEAR_IT(&htim2,0xFF);                       
    __HAL_TIM_SET_COUNTER(&htim2, 0);  // reset the counter
      
}
#endif

#if SLB_MOTOR_ALARM_POLLED

static void alarm_reset (void)
{
    if(on_reset)
        on_reset();

    motor_alarm_active = false;
}

static void execute_alarm (sys_state_t state)
{   
    
    if(!motor_alarm_active){

        if(motor_alarm_pins.x){
            report_message("Motor Error on X Axis!", Message_Warning);   
        }

        if(motor_alarm_pins.y){
            report_message("Motor Error on Y1 Axis!", Message_Warning);   
        }
        
        if(motor_alarm_pins.z){
            report_message("Motor Error on Z Axis!", Message_Warning);   
        }

        if(motor_alarm_pins.a){
            report_message("Motor Error on Y2 Axis!", Message_Warning);   
        }

        if(motor_alarm_pins.b){
            report_message("Motor Error on A Axis!", Message_Warning);   
        }

        if(motor_alarm_pins.value > 0){
            system_set_exec_alarm(Alarm_EStop);
        }
        motor_alarm_active = true;
    }         
    
}

static void poll_alarms (void){
    
    bool enable = 0;
    bool invert = 0;
    bool gpio = 0;

    uint32_t ms = hal.get_elapsed_ticks();
    if((ms < polling_ms + DEBOUNCE_DELAY) || (motor_alarms.enable.value == 0))
        return;

    int_fast8_t idx = N_MOTOR_ALARMS;
    do {
        idx--;

        switch (idx) {
            case 0:
            enable = motor_alarms.enable.x;
            invert = motor_alarms.invert.x;
            break;
            case 1:
            enable = motor_alarms.enable.y;
            invert = motor_alarms.invert.y;
            break;
            case 2:
            enable = motor_alarms.enable.z;
            invert = motor_alarms.invert.z;
            break;
            case 3:
            enable = motor_alarms.enable.y;
            invert = motor_alarms.invert.y;
            break;
            case 4:
            enable = motor_alarms.enable.a;
            invert = motor_alarms.invert.a;
            break;                                                
        }
        
        gpio = !DIGITAL_IN(alarmgpio[idx].port, alarmgpio[idx].pin);

        if(invert)
            gpio = !gpio;
        
        //if(!DIGITAL_IN(alarmgpio[idx].port, alarmgpio[idx].pin)) //apply enable here
        if(gpio && enable) //apply enable here
            val[idx]++;
        else
            val[idx]--;

        if(val[idx] > ALARM_THRESHOLD)
            val[idx] = ALARM_THRESHOLD;

        if(val[idx] < 0)
            val[idx] = 0;                   
        
        if (val[idx] >= ALARM_THRESHOLD)
            motor_alarm_pins.mask |= 1 << idx;
        else if (val[idx] == 0)
            motor_alarm_pins.mask &= ~(1 << idx);
    } while(idx >= 0);

    if(motor_alarm_pins.value > 0)
        protocol_enqueue_rt_command(execute_alarm);
    else
        motor_alarm_active = false;

    polling_ms = ms;    
}

static void alarm_poll_realtime (sys_state_t grbl_state)
{
    on_execute_realtime(grbl_state);

    poll_alarms();
}

static void alarm_poll_delay (sys_state_t grbl_state)
{
    on_execute_delay(grbl_state);

    poll_alarms();
}

#endif

#endif

void board_init (void)
{
    axes_signals_t i = {0};
    #if KEYPAD_ENABLE
    i2c_port = I2C_GetPort();
    #endif

    //set PA13 to output, it is AF SWDIO on reset.
#if AUXOUTPUT7_PIN    
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    __HAL_RCC_GPIOA_CLK_ENABLE();
#endif    

#if TRINAMIC_ENABLE
    static trinamic_driver_if_t driver_if = {.on_drivers_init = if_init};

    trinamic_if_init(&driver_if);
#endif    

#if SIENCI_LASER_PWM

    GPIO_InitStruct.Pin = 1 << LASER_ENABLE_PIN; 
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(LASER_ENABLE_PORT, &GPIO_InitStruct);
    __HAL_RCC_GPIOC_CLK_ENABLE();

    LASER_PWM_CLOCK_ENA();

    GPIO_InitTypeDef GPIO_Init = {
        .Speed = GPIO_SPEED_FREQ_HIGH,
        .Mode = GPIO_MODE_OUTPUT_PP
    };    

    GPIO_Init.Pin = (1 << LASER_PWM_PIN);
    GPIO_Init.Mode = GPIO_MODE_AF_PP;
    GPIO_Init.Pull = GPIO_NOPULL;
    GPIO_Init.Alternate = LASER_PWM_AF;
    HAL_GPIO_Init(LASER_PWM_PORT, &GPIO_Init);
#endif

//initialize I/O for sensorless homing.
#if TMC_POLL_STALLED
    GPIO_InitTypeDef GPIO_Init2 = {
        .Speed = GPIO_SPEED_FREQ_HIGH,
        .Mode = GPIO_MODE_INPUT,
        .Pull = GPIO_NOPULL
    };  

    GPIO_Init2.Pin = (1 << MOTOR_SGX_PIN);
    HAL_GPIO_Init(MOTOR_SGX_PORT, &GPIO_Init2);
    GPIO_Init2.Pin = (1 << MOTOR_SGY1_PIN);
    HAL_GPIO_Init(MOTOR_SGY1_PORT, &GPIO_Init2);
    GPIO_Init2.Pin = (1 << MOTOR_SGZ_PIN);
    HAL_GPIO_Init(MOTOR_SGZ_PORT, &GPIO_Init2);
    GPIO_Init2.Pin = (1 << MOTOR_SGY2_PIN);
    HAL_GPIO_Init(MOTOR_SGY2_PORT, &GPIO_Init2);            

    __HAL_RCC_GPIOA_CLK_ENABLE();
#endif

//initialize I/O clocks for motor alarms
#if SLB_MOTOR_ALARM

#if SLB_MOTOR_ALARM_IRQ
    motor_alarm_pins.value = 0;
    motor_alarm_polarity.value = 0;

    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();    

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = (1 << MOTOR_SGA_PIN);
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MOTOR_SGA_PORT, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
  
  /* TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFFFFFF;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1);
  HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2);
  HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3);
  HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4);
  

    GPIO_InitTypeDef GPIO_Init2 = {
        .Speed = GPIO_SPEED_FREQ_LOW,
        .Mode = GPIO_MODE_AF_PP,
        .Pull = GPIO_NOPULL,
        .Alternate = GPIO_AF1_TIM2
    };  

    GPIO_Init2.Pin = (1 << MOTOR_SGX_PIN);
    HAL_GPIO_Init(MOTOR_SGX_PORT, &GPIO_Init2);
    GPIO_Init2.Pin = (1 << MOTOR_SGY1_PIN);
    HAL_GPIO_Init(MOTOR_SGY1_PORT, &GPIO_Init2);
    GPIO_Init2.Pin = (1 << MOTOR_SGZ_PIN);
    HAL_GPIO_Init(MOTOR_SGZ_PORT, &GPIO_Init2);
    GPIO_Init2.Pin = (1 << MOTOR_SGY2_PIN);
    HAL_GPIO_Init(MOTOR_SGY2_PORT, &GPIO_Init2);

    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);

    /* TIM2 interrupt Init */
    HAL_NVIC_SetPriority(TIM2_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);

#endif
#if SLB_MOTOR_ALARM_POLLED

    GPIO_InitTypeDef GPIO_Init2 = {
        .Speed = GPIO_SPEED_FREQ_HIGH,
        .Mode = GPIO_MODE_INPUT,
        .Pull = GPIO_NOPULL
    };  

    //set all the pins to input, no IRQs here.
    GPIO_Init2.Pin = (1 << MOTOR_SGX_PIN);
    HAL_GPIO_Init(MOTOR_SGX_PORT, &GPIO_Init2);
    GPIO_Init2.Pin = (1 << MOTOR_SGY1_PIN);
    HAL_GPIO_Init(MOTOR_SGY1_PORT, &GPIO_Init2);
    GPIO_Init2.Pin = (1 << MOTOR_SGZ_PIN);
    HAL_GPIO_Init(MOTOR_SGZ_PORT, &GPIO_Init2);
    GPIO_Init2.Pin = (1 << MOTOR_SGY2_PIN);
    HAL_GPIO_Init(MOTOR_SGY2_PORT, &GPIO_Init2);
    GPIO_Init2.Pin = (1 << MOTOR_SGA_PIN);
    HAL_GPIO_Init(MOTOR_SGA_PORT, &GPIO_Init2);             

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    //add polling of button state to realtime and delay chains
    on_reset = grbl.on_reset;
    grbl.on_reset = alarm_reset;

    on_execute_realtime = grbl.on_execute_realtime;
    grbl.on_execute_realtime = alarm_poll_realtime;

    on_execute_delay = grbl.on_execute_delay;
    grbl.on_execute_delay = alarm_poll_delay; 

    if((nvs_address = nvs_alloc(sizeof(motor_alarm_settings_t)))) {
        settings_register(&setting_details);
    }      

#endif
#endif

}


#endif //BOARD_LONGBOARD32
