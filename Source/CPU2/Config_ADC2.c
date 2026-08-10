/*
 * Config_ADC.c
 *
 *  Created on: 2015. 3. 2.
 *      Author: Administrator
 */
//#include "F28x_Project.h"     // Device Headerfile and Examples Include File
#include "Config_ADC.h"     // Device Headerfile and Examples Include File

void conf_ADC_B(void)
{
	Uint16 acqps;

	EALLOW;
	AdcbRegs.ADCCTL2.bit.PRESCALE = 6; 		// ADCCLK = SYSCLK / 4, SYSCLK = 200MHz
	AdcbRegs.ADCCTL2.bit.RESOLUTION = ADC_RESOLUTION_12BIT;	// 12비트 분해능 모드
	AdcbRegs.ADCCTL2.bit.SIGNALMODE = ADC_SIGNALMODE_SINGLE;	// Single-Ended 모드
	AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;	// ADC Interrupt Pulse Position: 변환종료 후 발생
	AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;		// ADC 시동(Power-up)
	DELAY_US(1000);							// ADC가 시동되는 동안 1ms 지연
	EDIS;

	// ADC SOC(채널, S/H시간, 트리거소스) 및 인터럽트 설정
	EALLOW;

	//분해능에 따른 minimum acquisition window (S/H 시간) 설정
	if(AdcbRegs.ADCCTL2.bit.RESOLUTION == ADC_RESOLUTION_12BIT ){
		acqps = 14;		//75ns = (ACQPS+1)/SYSCLK
	}
	else { //resolution is 16-bit
		acqps = 63;		 //320ns = (ACQPS+1)/SYSCLK
	}

	AdcbRegs.ADCSOC0CTL.bit.CHSEL = 2;  //SOC0 : ADCINA2 채널 변환
	AdcbRegs.ADCSOC1CTL.bit.CHSEL = 1;
	AdcbRegs.ADCSOC2CTL.bit.CHSEL = 0;
	AdcbRegs.ADCSOC3CTL.bit.CHSEL = 3;

	AdcbRegs.ADCSOC0CTL.bit.ACQPS = acqps;
	AdcbRegs.ADCSOC1CTL.bit.ACQPS = acqps;
	AdcbRegs.ADCSOC2CTL.bit.ACQPS = acqps;
	AdcbRegs.ADCSOC3CTL.bit.ACQPS = acqps;

	AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 5;
	AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = 7;
	AdcbRegs.ADCSOC2CTL.bit.TRIGSEL = 9;
	AdcbRegs.ADCSOC3CTL.bit.TRIGSEL = 5;

	AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 3; 	// EOC3가 ADCINT1를 발생
	AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1;	// ADCINT1 활성화
	AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;	// ADCINT1 flag 클리어 확인

	EDIS;
}

void conf_ADC_D(void)
{
	Uint16 acqps;

	EALLOW;
	AdcdRegs.ADCCTL2.bit.PRESCALE = 6; 		// ADCCLK = SYSCLK / 4, SYSCLK = 200MHz
	AdcdRegs.ADCCTL2.bit.RESOLUTION = ADC_RESOLUTION_12BIT;	// 12비트 분해능 모드
	AdcdRegs.ADCCTL2.bit.SIGNALMODE = ADC_SIGNALMODE_SINGLE;	// Single-Ended 모드
	AdcdRegs.ADCCTL1.bit.INTPULSEPOS = 1;	// ADC Interrupt Pulse Position: 변환종료 후 발생
	AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1;		// ADC 시동(Power-up)
	DELAY_US(1000);							// ADC가 시동되는 동안 1ms 지연
	EDIS;

	// ADC SOC(채널, S/H시간, 트리거소스) 및 인터럽트 설정
	EALLOW;

	//분해능에 따른 minimum acquisition window (S/H 시간) 설정
	if(AdcdRegs.ADCCTL2.bit.RESOLUTION == ADC_RESOLUTION_12BIT){
		acqps = 14;		//75ns = (ACQPS+1)/SYSCLK
	}
	else { //resolution is 16-bit
		acqps = 63;		 //320ns = (ACQPS+1)/SYSCLK
	}

	AdcdRegs.ADCSOC0CTL.bit.CHSEL = 0;  //SOC0 : ADCINA0 채널 변환
	AdcdRegs.ADCSOC1CTL.bit.CHSEL = 1;
	AdcdRegs.ADCSOC2CTL.bit.CHSEL = 2;
	AdcdRegs.ADCSOC3CTL.bit.CHSEL = 3;
	AdcdRegs.ADCSOC4CTL.bit.CHSEL = 4;

	AdcdRegs.ADCSOC0CTL.bit.ACQPS = acqps;
	AdcdRegs.ADCSOC1CTL.bit.ACQPS = acqps;
	AdcdRegs.ADCSOC2CTL.bit.ACQPS = acqps;
	AdcdRegs.ADCSOC3CTL.bit.ACQPS = acqps;
	AdcdRegs.ADCSOC4CTL.bit.ACQPS = acqps;

	AdcdRegs.ADCSOC0CTL.bit.TRIGSEL = 5;
	AdcdRegs.ADCSOC1CTL.bit.TRIGSEL = 5;
	AdcdRegs.ADCSOC2CTL.bit.TRIGSEL = 5;
	AdcdRegs.ADCSOC3CTL.bit.TRIGSEL = 5;
	AdcdRegs.ADCSOC4CTL.bit.TRIGSEL = 5;		//13

	AdcdRegs.ADCINTSEL1N2.bit.INT1SEL = 4; 	// EOC0가 ADCINT1를 발생
	AdcdRegs.ADCINTSEL1N2.bit.INT1E = 1;	// ADCINT1 활성화
	AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;	// ADCINT1 flag 클리어 확인
	EDIS;
}


/*
void conf_ADC(void)
{
	Uint16 acqps;

	EALLOW;
	AdcaRegs.ADCCTL2.bit.PRESCALE = 6; 		// ADCCLK = SYSCLK / 4.5, SYSCLK = 200MHz
	AdcaRegs.ADCCTL2.bit.RESOLUTION = ADC_RESOLUTION_12BIT;	// 12비트 분해능 모드
	AdcaRegs.ADCCTL2.bit.SIGNALMODE = ADC_SIGNALMODE_SINGLE;	// Single-Ended 모드
	AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;	// ADC Interrupt Pulse Position: 변환종료 후 발생
	AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;		// ADC 시동(Power-up)
	DELAY_US(1000);							// ADC가 시동되는 동안 1ms 지연
	EDIS;

	// ADC SOC(채널, S/H시간, 트리거소스) 및 인터럽트 설정
	EALLOW;

	//분해능에 따른 minimum acquisition window (S/H 시간) 설정
	if(ADC_RESOLUTION_12BIT == AdcaRegs.ADCCTL2.bit.RESOLUTION){
		acqps = 14;		//75ns = (ACQPS+1)/SYSCLK
	}
	else { //resolution is 16-bit
		acqps = 63;		 //320ns = (ACQPS+1)/SYSCLK
	}

	// 여기부터  //
	AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0;  //SOC0 : ADCINA0 채널 변환
	AdcaRegs.ADCSOC1CTL.bit.CHSEL = 1;
	AdcaRegs.ADCSOC2CTL.bit.CHSEL = 2;
	AdcaRegs.ADCSOC3CTL.bit.CHSEL = 3;  //SOC0 : ADCINA0 채널 변환
	AdcaRegs.ADCSOC4CTL.bit.CHSEL = 4;
	AdcaRegs.ADCSOC5CTL.bit.CHSEL = 5;

	AdcaRegs.ADCSOC6CTL.bit.CHSEL = 14;
	AdcaRegs.ADCSOC7CTL.bit.CHSEL = 15;


	AdcaRegs.ADCSOC0CTL.bit.ACQPS = acqps;
	AdcaRegs.ADCSOC1CTL.bit.ACQPS = acqps;
	AdcaRegs.ADCSOC2CTL.bit.ACQPS = acqps;
	AdcaRegs.ADCSOC3CTL.bit.ACQPS = acqps;
	AdcaRegs.ADCSOC4CTL.bit.ACQPS = acqps;
	AdcaRegs.ADCSOC5CTL.bit.ACQPS = acqps;

	AdcaRegs.ADCSOC6CTL.bit.ACQPS = acqps;
	AdcaRegs.ADCSOC7CTL.bit.ACQPS = acqps;


	AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5;
	AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 5;
	AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = 5;
	AdcaRegs.ADCSOC3CTL.bit.TRIGSEL = 5;
	AdcaRegs.ADCSOC4CTL.bit.TRIGSEL = 5;
	AdcaRegs.ADCSOC5CTL.bit.TRIGSEL = 5;

	AdcaRegs.ADCSOC6CTL.bit.TRIGSEL = 5;
	AdcaRegs.ADCSOC7CTL.bit.TRIGSEL = 5;


	AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0; 	// EOC0가 ADCINT1를 발생
	AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;	// ADCINT1 활성화
	AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;	// ADCINT1 flag 클리어 확인
	EDIS;
}

void conf_ADC_C(void)
{
	Uint16 acqps;

	EALLOW;
	AdccRegs.ADCCTL2.bit.PRESCALE = 6; 		// ADCCLK = SYSCLK / 4, SYSCLK = 200MHz
	AdccRegs.ADCCTL2.bit.RESOLUTION = ADC_RESOLUTION_12BIT;	// 12비트 분해능 모드
	AdccRegs.ADCCTL2.bit.SIGNALMODE = ADC_SIGNALMODE_SINGLE;	// Single-Ended 모드
	AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;	// ADC Interrupt Pulse Position: 변환종료 후 발생
	AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;		// ADC 시동(Power-up)
	DELAY_US(1000);							// ADC가 시동되는 동안 1ms 지연
	EDIS;

	// ADC SOC(채널, S/H시간, 트리거소스) 및 인터럽트 설정
	EALLOW;

	//분해능에 따른 minimum acquisition window (S/H 시간) 설정
	if(AdccRegs.ADCCTL2.bit.RESOLUTION == ADC_RESOLUTION_12BIT){
		acqps = 14;		//75ns = (ACQPS+1)/SYSCLK
	}
	else { //resolution is 16-bit
		acqps = 63;		 //320ns = (ACQPS+1)/SYSCLK
	}

	AdccRegs.ADCSOC0CTL.bit.CHSEL = 2;  //SOC0 : ADCINA0 채널 변환
	AdccRegs.ADCSOC1CTL.bit.CHSEL = 3;
	AdccRegs.ADCSOC2CTL.bit.CHSEL = 4;

	AdccRegs.ADCSOC0CTL.bit.ACQPS = acqps;
	AdccRegs.ADCSOC1CTL.bit.ACQPS = acqps;
	AdccRegs.ADCSOC2CTL.bit.ACQPS = acqps;

	AdccRegs.ADCSOC0CTL.bit.TRIGSEL = 5;
	AdccRegs.ADCSOC1CTL.bit.TRIGSEL = 5;
	AdccRegs.ADCSOC2CTL.bit.TRIGSEL = 5;

	AdccRegs.ADCINTSEL1N2.bit.INT1SEL = 2; 	// EOC0가 ADCINT1를 발생
	AdccRegs.ADCINTSEL1N2.bit.INT1E = 1;	// ADCINT1 활성화
	AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;	// ADCINT1 flag 클리어 확인
	EDIS;
}
*/




