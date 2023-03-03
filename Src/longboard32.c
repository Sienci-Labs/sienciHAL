/*
  flexi_hal.c - driver code for STM32F4xx ARM processors on Flexi-HAL board

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

#include "driver.h"

#if defined(BOARD_LONGBOARD32)

#include <math.h>
#include <string.h>

#include "main.h"
#include "i2c.h"
#include "grbl/protocol.h"
#include "grbl/settings.h"

SPI_HandleTypeDef hspi2 = {0};
GPIO_InitTypeDef GPIO_InitStruct = {0};

#if KEYPAD_ENABLE || EEPROM_ENABLE

static uint8_t keycode = 0;
static keycode_callback_ptr keypad_callback = NULL;
static bool pendant_tx_active = 0;

static FMPI2C_HandleTypeDef *i2c_port;


void HAL_FMPI2C_MemRxCpltCallback(FMPI2C_HandleTypeDef *hi2c)
{
    if(keypad_callback && keycode != 0) {
        keypad_callback(keycode);
        keypad_callback = NULL;
    }
}

void HAL_FMPI2C_MasterTxCpltCallback(FMPI2C_HandleTypeDef *hi2c)
{
    pendant_tx_active = 0;
}

// called from stream drivers while tx is blocking, returns false to terminate

#if 0
bool flexi_stream_tx_blocking (void)
{
    // TODO: Restructure st_prep_buffer() calls to be executed here during a long print.

    grbl.on_execute_realtime(state_get());

    return !(sys.rt_exec_state & EXEC_RESET);
}
#endif

#endif //keypad enable

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
    volatile uint32_t dly = 100;

    while(--dly)
        __ASM volatile ("nop");
}

static uint8_t spi_get_byte (void)
{
    spi_port.Instance->DR = 0xFF; // Writing dummy data into Data register

    while(!__HAL_SPI_GET_FLAG(&spi_port, SPI_FLAG_RXNE));

    return (uint8_t)spi_port.Instance->DR;
}

static uint8_t spi_put_byte (uint8_t byte)
{
    spi_port.Instance->DR = byte;

    while(!__HAL_SPI_GET_FLAG(&spi_port, SPI_FLAG_TXE));
    while(!__HAL_SPI_GET_FLAG(&spi_port, SPI_FLAG_RXNE));

    __HAL_SPI_CLEAR_OVRFLAG(&spi_port);

    return (uint8_t)spi_port.Instance->DR;
}

TMC_spi_status_t tmc_spi_read (trinamic_motor_t driver, TMC_spi_datagram_t *datagram)
{
    TMC_spi_status_t status;

    DIGITAL_OUT(cs[driver.id].port, cs[driver.id].pin, 0);

    datagram->payload.value = 0;

    datagram->addr.write = 0;
    spi_put_byte(datagram->addr.value);
    spi_put_byte(0);
    spi_put_byte(0);
    spi_put_byte(0);
    spi_put_byte(0);

    DIGITAL_OUT(cs[driver.id].port, cs[driver.id].pin, 1);
    delay();
    DIGITAL_OUT(cs[driver.id].port, cs[driver.id].pin, 0);

    status = spi_put_byte(datagram->addr.value);
    datagram->payload.data[3] = spi_get_byte();
    datagram->payload.data[2] = spi_get_byte();
    datagram->payload.data[1] = spi_get_byte();
    datagram->payload.data[0] = spi_get_byte();

    DIGITAL_OUT(cs[driver.id].port, cs[driver.id].pin, 1);

    return status;
}

TMC_spi_status_t tmc_spi_write (trinamic_motor_t driver, TMC_spi_datagram_t *datagram)
{
    TMC_spi_status_t status;
    uint32_t gram = 0;

    DIGITAL_OUT(cs[driver.id].port, cs[driver.id].pin, 0);
    delay();
    /*datagram->addr.write = 1;
    status = spi_put_byte(datagram->addr.value);
    spi_put_byte(datagram->payload.data[3]);
    spi_put_byte(datagram->payload.data[2]);
    spi_put_byte(datagram->payload.data[1]);
    spi_put_byte(datagram->payload.data[0]);*/

    gram = ((datagram->addr.value << 16) | datagram->payload.value);

    spi_put_byte(gram>>16 & 0xFF);
    spi_put_byte(gram>>8 & 0xFF);
    spi_put_byte(gram & 0xFF);

    delay();
    DIGITAL_OUT(cs[driver.id].port, cs[driver.id].pin, 1);

    return status;
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

        case Output_MotorChipSelectM5:
            cs[5].port = (GPIO_TypeDef *)gpio->port;
            cs[5].pin = gpio->pin;
            break;

        default:
            break;
    }
}

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

void board_init (void)
{
    axes_signals_t i = {0};
    #if KEYPAD_ENABLE
    i2c_port = I2C_GetPort();
    #endif

    static trinamic_driver_if_t driver_if = {.on_drivers_init = if_init};

    trinamic_if_init(&driver_if);
 //   if_init(4, i);

    /* set default values to TMC2660 drivers*/
/*
    for (i=0; i < 4; i++){

    tmc2660_writeInt(i, TMC2660_DRVCONF,  0x000205);
	tmc2660_writeInt(i, TMC2660_DRVCTRL,  0x080000);
	tmc2660_writeInt(i, TMC2660_CHOPCONF, 0x0A0747);
	tmc2660_writeInt(i, TMC2660_SMARTEN,  0x0C0214);
	tmc2660_writeInt(i, TMC2660_SGCSCONF, 0x0EF00E); //always append E

    }
*/
    //add some init code for neopixel?

    

}

#endif //BOARD_LONGBOARD32
