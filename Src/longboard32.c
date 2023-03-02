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
#include "tmc/ic/TMC2660/TMC2660.h"

#if defined(BOARD_LONGBOARD32)

#include <math.h>
#include <string.h>

#include "main.h"
#include "i2c.h"
#include "grbl/protocol.h"
#include "grbl/settings.h"

#if KEYPAD_ENABLE

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

#endif

void tmc2660_writeInt(uint8_t motor, uint8_t address, int value)
{	UNUSED(motor);

	// tmc2660_writeDatagram(address, 0xFF & (value>>24), 0xFF & (value>>16), 0xFF & (value>>8), 0xFF & (value>>0));
}

uint32_t tmc2660_readInt(uint8_t motor, uint8_t address)
{
    return 0;
}
void tmc2660_readWrite(uint8_t motor, uint32_t value)
{}

void board_init (void)
{
    uint8_t i;
    #if KEYPAD_ENABLE
    i2c_port = I2C_GetPort();
    #endif

    /* set default values to TMC2660 drivers*/

    for (i=0; i < 4; i++){

    tmc2660_writeInt(i, TMC2660_DRVCONF,  0x000205);
	tmc2660_writeInt(i, TMC2660_DRVCTRL,  0x080000);
	tmc2660_writeInt(i, TMC2660_CHOPCONF, 0x0A0747);
	tmc2660_writeInt(i, TMC2660_SMARTEN,  0x0C0214);
	tmc2660_writeInt(i, TMC2660_SGCSCONF, 0x0EF00E); //always append E

    //add some init code for neopixel?

    }

}

#endif
