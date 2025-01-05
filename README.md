# AD9833
 Frequency generator code in C++

This version was created for MCUs like NUCLEO-C031C6 from ST.
I have used the STM32CubeIDE to generate the driver code.

## functions
	//constructor that needs a handle to the SPI driver.
	AD9833(SPI_HandleTypeDef *hspi)


/*********************************************************************//**
 * @brief Selects the type of output.
 *
 * @param out_mode - type of output
 *                   Example AD9833       : 0 - Sinusoid.
 *                                          1 - Triangle.
 *                                          2 - DAC Data MSB/2.
 *                                          3 - DAC Data MSB.
 *
**************************************************************************/

bool AD9833::SetOutMode(OUTMODE out_mode)

/**********************************************************************//**
 * Loads a frequency value in a register.
 *
 * @param register_number - Number of the register (0 / 1).
 * @param frequency_value - Frequency value in Hz.
 *
**************************************************************************/

bool AD9833::SetFrequency( REGISTER register_number, uint32_t frequency_value)

/*********************************************************************//**
 * Loads a phase value in a register.
 *
 * @param register_number - Number of the register (0 / 1).
 * @param phase_value     - Phase value.
 *
*************************************************************************/

bool AD9833::SetPhase(REGISTER register_number, float phase_value)


## example usage

#include "ad9833.h"

SPI_HandleTypeDef hspi1;

	AD9833 ad9833(&hspi1);
	ad9833.Init();
  	ad9833.SetOutMode(MODE_SQUARE);
	ad9833.SetFrequency(REG0, 100); 
