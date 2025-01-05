/*
 * AD9833.cpp
 *
 *  Created on: Dec 18, 2024
 *      Author: Per
 *
 *  Purpose: Interface to AD9833 frequency generator
 *
 *  Taken from Analog Devices web site and stripped of other devices and rewritten in C++.
 *
 *  Copyright 2013(c) Analog Devices, Inc. See license at end of file.
 *  Copyright 2024(c) Per
 *
 *  The SPI_HandleTypeDef *hspi is defined by the STM32CubeIDE when generating code STM32 devices.
 *
 *  For hardware see https://github.com/city028/AD9833/tree/master
 */

#include "AD9833.h"


/*******************************************
 * constructor
 *
 * m_FrequencyConst = 2^28 / 25*10^6 = 10.7374182
 *
 * hspi: pointer to SPI definition
 *******************************************/
AD9833::AD9833(SPI_HandleTypeDef *hspi) : m_hspi(hspi), m_controlReg(0),m_FrequencyConst(10.7374182f), m_phaseConst(651.8986469f)

{
	for(uint8_t i=0; i<sizeof(m_rx_buffer);i++)
		m_rx_buffer[i]=0;

}

/*******************************************
 * destructor
 *
 *******************************************/
AD9833::~AD9833()
{
}

/*******************************************
 * Init
 *
 *******************************************/
void AD9833::Init(void)
{

	/* Initialize board. */
	m_controlReg = AD9833_CTRLRESET;
	Tx_spi(m_controlReg);
	HAL_Delay(1);

	SetFrequency(REG0, 700);
	SetFrequency(REG1, 0);
	SetPhase(REG0, 0);
	SetPhase(REG1, 0);
	SelectFrequencyReg(REG0);
	SelectPhaseReg(REG0);
	SetOutMode(OUTMODE::MODE_SIN);

	m_controlReg &= ~AD9833_CTRLRESET;
	Tx_spi(m_controlReg); //remove reset
}

/**************************************************************************//**
 *  Write data to/from SPI.
 *
 *  data: what to send
 *  wordCount: how much
 *
******************************************************************************/
bool AD9833::Write(uint16_t data)
{

	if(HAL_SPI_Transmit(m_hspi, (uint8_t *)&data, 1, 10) == HAL_OK)
	{
		return true;
	}

	return false;
}


/**************************************************************************//**
 *  Transmits 16 bits on SPI.
 *
 *  value - Data which will be transmitted.
 *
******************************************************************************/
bool AD9833::Tx_spi(int16_t value)
{
	uint16_t spi_data = 0;

	if(!Write(value))
	{
		/* Initialize board. */
		spi_data |= AD9833_CTRLRESET;
		Tx_spi(spi_data);
		HAL_Delay(1);
		spi_data &= ~ AD9833_CTRLRESET;
		Tx_spi(spi_data);

		return false;
	}

	return true;
}


/**************************************************************************//**
 * @brief Selects the type of output.
 *
 * @param out_mode - type of output
 *                   Example AD9833       : 0 - Sinusoid.
 *                                          1 - Triangle.
 *                                          2 - DAC Data MSB/2.
 *                                          3 - DAC Data MSB.
 *
******************************************************************************/
bool AD9833::SetOutMode(OUTMODE out_mode)
{
	uint16_t spi_data = 0;
	bool status = 0;

	spi_data = (m_controlReg & ~(AD9833_CTRLMODE | AD9833_CTRLOPBITEN | AD9833_CTRLDIV2));

	switch (out_mode)
	{
	case MODE_TRIANGLE:     // Triangle
		spi_data |= AD9833_CTRLMODE;
		break;
	case MODE_SQUARE2:     // DAC Data MSB/2
		spi_data |= AD9833_CTRLOPBITEN;
		break;
	case MODE_SQUARE:     // DAC Data MSB
		spi_data |= AD9833_CTRLOPBITEN | AD9833_CTRLDIV2;
		break;
	default:    // Sinusoid
		break;
	}

	status =Tx_spi(spi_data);
	m_controlReg = spi_data;

	return status;
}


/**************************************************************************//**
 * Loads a frequency value in a register.
 *
 * @param register_number - Number of the register (0 / 1).
 * @param frequency_value - Frequency value in Hz.
 *
******************************************************************************/
bool AD9833::SetFrequency( REGISTER register_number, uint32_t frequency_value)
{
	uint32_t ul_freq_register;
	uint16_t i_freq_lsb, i_freq_msb;
	bool ok1,ok2;

	ul_freq_register = (uint32_t)(frequency_value * m_FrequencyConst);
	i_freq_lsb = (ul_freq_register & 0x0003FFF);
	i_freq_msb = ((ul_freq_register & 0xFFFC000) >> 14);
	m_controlReg |= AD9833_CTRLB28;

	Tx_spi(m_controlReg);

	if (register_number == REG0)
	{
		ok1=Tx_spi(BIT_F0ADDRESS + i_freq_lsb);
		ok2=Tx_spi(BIT_F0ADDRESS + i_freq_msb);
	} else
	{
		ok1=Tx_spi(BIT_F1ADDRESS + i_freq_lsb);
		ok2=Tx_spi(BIT_F1ADDRESS + i_freq_msb);
	}

	m_controlReg &= ~AD9833_CTRLB28;
	Tx_spi(m_controlReg);

	return ok1 && ok2;
}


/**************************************************************************//**
 * Loads a phase value in a register.
 *
 * @param register_number - Number of the register (0 / 1).
 * @param phase_value     - Phase value.
 *
******************************************************************************/
bool AD9833::SetPhase(REGISTER register_number, float phase_value)
{
	uint16_t phase_calc;
	bool ok;

	phase_calc = (uint16_t)(phase_value * m_phaseConst);
	if (register_number == REG0)
	{
		ok=Tx_spi(BIT_P0ADDRESS + phase_calc);
	} else
	{
		ok=Tx_spi(BIT_P1ADDRESS + phase_calc);
	}

	return ok;
}


/**************************************************************************//**
 * Enable / Disable the sleep function.
 *
 * sleep_mode - type of sleep
 *                              0 - No power-down.
 *                              1 - DAC powered down.
 *                              2 - Internal clock disabled.
 *                              3 - DAC powered down and Internal
 *                                  clock disabled.
 *
******************************************************************************/
bool AD9833::SetSleepMode(SLEEP_MODE sleep_mode)
{
	uint16_t spi_data = 0;

	spi_data = (m_controlReg & ~(AD9833_CTRLSLEEP12 | AD9833_CTRLSLEEP1));

	switch (sleep_mode)
	{
	case DAC_SLEEP:     // DAC powered down
		spi_data |= AD9833_CTRLSLEEP12;
		break;
	case CLOCK_SLEEP:     // Internal clock disabled
		spi_data |= AD9833_CTRLSLEEP1;
		break;
	case FULL_SLEEP:     // DAC powered down and Internal clock disabled
		spi_data |= AD9833_CTRLSLEEP1 | AD9833_CTRLSLEEP12;
		break;
	default:    // No power-down
		break;
	}
	bool ok=Tx_spi(spi_data);
	m_controlReg = spi_data;

	return ok;
}


/**************************************************************************//**
 * @brief Select the frequency register to be used.
 *
 * @param dev      - The device structure.
 * @param freq_reg - Number of frequency register. (0 / 1)
 *
 * @return None.
******************************************************************************/
bool AD9833::SelectFrequencyReg(REGISTER freq_reg)
{
	uint16_t spi_data = 0;
	bool ok;

	spi_data = (m_controlReg & ~AD9833_CTRLFSEL);
	// Select soft the working frequency register according to parameter
	if (freq_reg == REG1)
	{
		spi_data |= AD9833_CTRLFSEL;
	}
	ok=Tx_spi(spi_data);
	m_controlReg = spi_data;

	return ok;
}

/**************************************************************************//**
 * @brief Select the phase register to be used.
 *
 * @param dev       - The device structure.
 * @param phase_reg - Number of phase register. (0 / 1)
 *
 * @return None.
******************************************************************************/
bool AD9833::SelectPhaseReg(REGISTER phase_reg)
{
	uint16_t spi_data = 0;
	bool ok;

	spi_data = (m_controlReg & ~AD9833_CTRLPSEL);
	// Select soft the working phase register according to parameter
	if (phase_reg == REG1)
	{
		spi_data |= AD9833_CTRLPSEL;
	}
	ok=Tx_spi(spi_data);
	m_controlReg = spi_data;

	return ok;
}


/**************************************************************************//**
*   @file   ad9833.c
*   @brief  Implementation of ad9833 Driver for Microblaze processor.
*   @author Lucian Sin (Lucian.Sin@analog.com)
*
*******************************************************************************
* Copyright 2013(c) Analog Devices, Inc.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* 3. Neither the name of Analog Devices, Inc. nor the names of its
*    contributors may be used to endorse or promote products derived from this
*    software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES, INC. “AS IS” AND ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
* EVENT SHALL ANALOG DEVICES, INC. BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
* OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
* LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
* NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
* EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
* *****************************************************************************/
