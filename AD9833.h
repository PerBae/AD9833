/*
 * AD9833.h
 *
 *  Created on: Dec 18, 2024
 *      Author: Per
 *
 *  Copyright: see cpp-file and license.
 */

#ifndef AD9833_H_
#define AD9833_H_

#include "stm32c0xx_hal.h"
#include "stm32c0xx_hal_def.h"
#include "stm32c0xx_hal_spi.h"
/*
 *                   Example AD9833	: 0 - Sinusoid.
 *                                    1 - Triangle.
 *                                    2 - DAC Data MSB/2.
 *                                    3 - DAC Data MSB.
*/
enum OUTMODE {MODE_SIN=0,MODE_TRIANGLE,MODE_SQUARE2,MODE_SQUARE};
/*
*                              0 - No power-down.
*                              1 - DAC powered down.
*                              2 - Internal clock disabled.
*                              3 - DAC powered down and Internal
*                                  clock disabled.
*/
enum SLEEP_MODE {NO_SLEEP=0,DAC_SLEEP,CLOCK_SLEEP,FULL_SLEEP};

#define AD9833_CTRLB28          (1 << 13)
#define AD9833_CTRLHLB          (1 << 12)
#define AD9833_CTRLFSEL         (1 << 11)
#define AD9833_CTRLPSEL         (1 << 10)
#define AD9834_CTRLPINSW        (1 << 9)
#define AD9833_CTRLRESET        (1 << 8)
#define AD9833_CTRLSLEEP1       (1 << 7)
#define AD9833_CTRLSLEEP12      (1 << 6)
#define AD9833_CTRLOPBITEN      (1 << 5)
#define AD9834_CTRLSIGNPIB      (1 << 4)
#define AD9833_CTRLDIV2         (1 << 3)
#define AD9833_CTRLMODE         (1 << 1)

#define BIT_F0ADDRESS           0x4000      // Frequency Register 0 address.
#define BIT_F1ADDRESS           0x8000      // Frequency Register 1 address.
#define BIT_P0ADDRESS           0xC000      // Phase Register 0 address.
#define BIT_P1ADDRESS           0xE000      // Phase Register 1 address.

enum REGISTER {REG0=0,REG1};

class AD9833
{
public:
	AD9833(SPI_HandleTypeDef *hspi);
	virtual ~AD9833();
	void Init(void);
	bool SetOutMode(OUTMODE out_mode);
	bool SetFrequency( REGISTER register_number, uint32_t frequency_value);
	bool SetPhase(REGISTER register_number, float phase_value);
	bool SetSleepMode(SLEEP_MODE sleep_mode);
	bool SelectFrequencyReg(REGISTER freq_reg);
	bool SelectPhaseReg(REGISTER phase_reg);

private:
	AD9833();
	bool Write(uint16_t data);
	bool Tx_spi(int16_t value);

	SPI_HandleTypeDef *m_hspi;
	uint8_t m_rx_buffer[4];
	uint16_t m_controlReg;
	float m_FrequencyConst;
	float m_phaseConst;

};

#endif /* AD9833_H_ */
