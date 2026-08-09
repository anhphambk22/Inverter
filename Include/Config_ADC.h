/*
 * Config_ADC.h
 *
 *  Created on: 2015. 3. 2.
 *      Author: Administrator
 */

#include "F28x_Project.h"     // Device Headerfile and Examples Include File

#ifndef CONFIG_ADC_H_
#define CONFIG_ADC_H_

extern void conf_ADC_A(void);
extern void conf_ADC_B(void);
extern void conf_ADC_C(void);
extern void conf_ADC_D(void);
void ConfigueDAC(void);
void Config_DAC_PINB1(void);

float32 scale_testADC(float32 in_dsp, float32 offset, float32 gain);

#endif /* CONFIG_ADC_H_ */
