
#include "F28x_Project.h"     // Device Headerfile and Examples Include File
#include "Config_ADC.h"     // Device Headerfile and Examples Include File

void conf_ADC_A(void)
{
    Uint16 acqps;

    EALLOW;
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6;      // ADCCLK = SYSCLK / 4.5, SYSCLK = 200MHz
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
//    AdcaRegs.ADCCTL2.bit.RESOLUTION = ADC_RESOLUTION_16BIT;         // 16비트 분해능 모드
//    AdcaRegs.ADCCTL2.bit.SIGNALMODE = ADC_SIGNALMODE_DIFFERENTIAL;  // Differential 모드
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;   // ADC Interrupt Pulse Position: 변환종료 후 발생
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;      // ADC 시동(Power-up)
    DELAY_US(1500);                         // ADC가 시동되는 동안 1ms 지연
    EDIS;
    EALLOW;

    if(ADC_RESOLUTION_12BIT == AdcaRegs.ADCCTL2.bit.RESOLUTION){
        acqps = 14;     //75ns = (ACQPS+1)/SYSCLK
    }
    else { //resolution is 16-bit
        acqps = 63;    // 63;      //320ns = (ACQPS+1)/SYSCLK
    }
    acqps = 25;// DrDuc
//SETUP CHANNEL SELLECT Adcx_CHSELy --> pin xy
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0;  //SOC0 : ADCINA0 채널 변환
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 1;
    AdcaRegs.ADCSOC2CTL.bit.CHSEL = 2;
    AdcaRegs.ADCSOC3CTL.bit.CHSEL = 3;
    AdcaRegs.ADCSOC4CTL.bit.CHSEL = 4;
    AdcaRegs.ADCSOC5CTL.bit.CHSEL = 5;
    AdcaRegs.ADCSOC6CTL.bit.CHSEL = 15;

//5 adca pin

    AdcaRegs.ADCSOC0CTL.bit.ACQPS = acqps;  // SOC 0 will use sample duration of acqps SYSCLK cycles
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = acqps;
    AdcaRegs.ADCSOC2CTL.bit.ACQPS = acqps;
    AdcaRegs.ADCSOC3CTL.bit.ACQPS = acqps;
    AdcaRegs.ADCSOC4CTL.bit.ACQPS = acqps;
    AdcaRegs.ADCSOC5CTL.bit.ACQPS = acqps;

    //AdcbRegs.ADCSOC5CTL.bit.ACQPS = acqps;
    AdcaRegs.ADCSOC6CTL.bit.ACQPS = acqps;


    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 0x05;  // determine the PWM channel to start the conversion (page 1467 of spruhm8g)
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 0x05;     // 05h = EPWM1
    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = 0x05;
    AdcaRegs.ADCSOC3CTL.bit.TRIGSEL = 0x05;
    AdcaRegs.ADCSOC4CTL.bit.TRIGSEL = 0x05;
    AdcaRegs.ADCSOC5CTL.bit.TRIGSEL = 0x05;
    AdcaRegs.ADCSOC6CTL.bit.TRIGSEL = 0x05;

    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 6;  // EOC5가 ADCINT1를 발생
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;    // ADCINT1 활성화
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;  // ADCINT1 flag 클리어 확인
    EDIS;
}
 //below is for 1 pin DAC test
/*
void conf_ADC_A(void)
{
    Uint16 acqps;

    EALLOW;
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6;      // ADCCLK = SYSCLK / 4.5, SYSCLK = 200MHz
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
//    AdcaRegs.ADCCTL2.bit.RESOLUTION = ADC_RESOLUTION_16BIT;         // 16비트 분해능 모드
//    AdcaRegs.ADCCTL2.bit.SIGNALMODE = ADC_SIGNALMODE_DIFFERENTIAL;  // Differential 모드
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;   // ADC Interrupt Pulse Position: 변환종료 후 발생
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;      // ADC 시동(Power-up)
    DELAY_US(1500);                         // ADC가 시동되는 동안 1ms 지연
    EDIS;
    EALLOW;

    if(ADC_RESOLUTION_12BIT == AdcaRegs.ADCCTL2.bit.RESOLUTION){
        acqps = 14;     //75ns = (ACQPS+1)/SYSCLK
    }
    else { //resolution is 16-bit
        acqps = 63;    // 63;      //320ns = (ACQPS+1)/SYSCLK
    }
    acqps = 25;// DrDuc
//SETUP CHANNEL SELLECT Adcx_CHSELy --> pin xy
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 1;  //SOC0 : ADCINA0 채널 변환
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 2;
    AdcaRegs.ADCSOC2CTL.bit.CHSEL = 3;



    AdcaRegs.ADCSOC0CTL.bit.ACQPS = acqps;  // SOC 0 will use sample duration of acqps SYSCLK cycles
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = acqps;
    AdcaRegs.ADCSOC2CTL.bit.ACQPS = acqps;


    //AdcbRegs.ADCSOC5CTL.bit.ACQPS = acqps;
   // AdcaRegs.ADCSOC14CTL.bit.ACQPS = acqps;


    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 0x05;  // determine the PWM channel to start the conversion (page 1467 of spruhm8g)
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 0x05;     // 05h = EPWM1
    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = 0x05;


    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 2;  // EOC5가 ADCINT1를 발생
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;    // ADCINT1 활성화
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;  // ADCINT1 flag 클리어 확인
    EDIS;
}

*/
void conf_ADC_B(void)
{
    Uint16 acqps;

    EALLOW;
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6;      // ADCCLK = SYSCLK / 4, SYSCLK = 200MHz
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
//    AdcbRegs.ADCCTL2.bit.RESOLUTION = ADC_RESOLUTION_16BIT; // 12비트 분해능 모드
//    AdcbRegs.ADCCTL2.bit.SIGNALMODE = ADC_SIGNALMODE_DIFFERENTIAL;    // Single-Ended 모드
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;   // ADC Interrupt Pulse Position: 변환종료 후 발생
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;      // ADC 시동(Power-up)
    DELAY_US(1000);                         // ADC가 시동되는 동안 1ms 지연
    EDIS;
    EALLOW;
    if(AdcbRegs.ADCCTL2.bit.RESOLUTION == ADC_RESOLUTION_12BIT ){
        acqps = 14;     //75ns = (ACQPS+1)/SYSCLK
    }
    else { //resolution is 16-bit
        acqps = 63;      //320ns = (ACQPS+1)/SYSCLK
    }
    acqps = 25;// DrDuc

    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 0;
    AdcbRegs.ADCSOC1CTL.bit.CHSEL = 1;
    AdcbRegs.ADCSOC2CTL.bit.CHSEL = 2;
    AdcbRegs.ADCSOC3CTL.bit.CHSEL = 3;   // B3



    AdcbRegs.ADCSOC0CTL.bit.ACQPS = acqps;
    AdcbRegs.ADCSOC1CTL.bit.ACQPS = acqps;
    AdcbRegs.ADCSOC2CTL.bit.ACQPS = acqps;
    AdcbRegs.ADCSOC3CTL.bit.ACQPS = acqps;

    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 0x05;  //PWM1
    AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = 0x05;
    AdcbRegs.ADCSOC2CTL.bit.TRIGSEL = 0x05;
    AdcbRegs.ADCSOC3CTL.bit.TRIGSEL = 0x05;

    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 3;  // EOC3가 ADCINT1를 발생
    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1;    // ADCINT1 활성화
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;  // ADCINT1 flag 클리어 확인
    EDIS;
}

void conf_ADC_C(void)
{
    Uint16 acqps;

    EALLOW;
    AdccRegs.ADCCTL2.bit.PRESCALE = 6;      // ADCCLK = SYSCLK / 4, SYSCLK = 200MHz
    AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
//    AdccRegs.ADCCTL2.bit.RESOLUTION = ADC_RESOLUTION_16BIT; // 12비트 분해능 모드
//    AdccRegs.ADCCTL2.bit.SIGNALMODE = ADC_SIGNALMODE_DIFFERENTIAL;  // Single-Ended 모드
    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;   // ADC Interrupt Pulse Position: 변환종료 후 발생
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;      // ADC 시동(Power-up)
    DELAY_US(1000);                         // ADC가 시동되는 동안 1ms 지연
    EDIS;
    EALLOW;

    if(ADC_RESOLUTION_12BIT == AdcbRegs.ADCCTL2.bit.RESOLUTION){
        acqps = 14;     //75ns = (ACQPS+1)/SYSCLK
    }
    else { //resolution is 16-bit
        acqps = 63;      //320ns = (ACQPS+1)/SYSCLK
    }
    acqps = 25;

    AdccRegs.ADCSOC0CTL.bit.CHSEL = 2;
    AdccRegs.ADCSOC1CTL.bit.CHSEL = 3;
    AdccRegs.ADCSOC2CTL.bit.CHSEL = 4;

    AdccRegs.ADCSOC0CTL.bit.ACQPS = acqps;
    AdccRegs.ADCSOC1CTL.bit.ACQPS = acqps;
    AdccRegs.ADCSOC2CTL.bit.ACQPS = acqps;

    AdccRegs.ADCSOC0CTL.bit.TRIGSEL = 0x05;
    AdccRegs.ADCSOC1CTL.bit.TRIGSEL = 0x05;
    AdccRegs.ADCSOC2CTL.bit.TRIGSEL = 0x05;

    AdccRegs.ADCINTSEL1N2.bit.INT1SEL = 2;  //
    AdccRegs.ADCINTSEL1N2.bit.INT1E = 1;    //
    AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;  // ADCINT1 flag
    EDIS;
}

void conf_ADC_D(void)
{
    Uint16 acqps;

    EALLOW;
    AdcdRegs.ADCCTL2.bit.PRESCALE = 6;      // ADCCLK = SYSCLK / 4, SYSCLK = 200MHz
    AdcSetMode(ADC_ADCD, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
//    AdcdRegs.ADCCTL2.bit.RESOLUTION = ADC_RESOLUTION_16BIT; // 12비트 분해능 모드
//    AdcdRegs.ADCCTL2.bit.SIGNALMODE = ADC_SIGNALMODE_DIFFERENTIAL;    // Single-Ended 모드
    AdcdRegs.ADCCTL1.bit.INTPULSEPOS = 1;   // ADC Interrupt Pulse Position: 변환종료 후 발생
    AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1;      // ADC 시동(Power-up)
    DELAY_US(1000);                         // ADC가 시동되는 동안 1ms 지연
    EDIS;
    acqps = 25;
    // ADC SOC(채널, S/H시간, 트리거소스) 및 인터럽트 설정
    EALLOW;

    //분해능에 따른 minimum acquisition window (S/H 시간) 설정
    if(AdcdRegs.ADCCTL2.bit.RESOLUTION == ADC_RESOLUTION_12BIT){
        acqps = 14;     //75ns = (ACQPS+1)/SYSCLK
    }
    else { //resolution is 16-bit
        acqps = 63;      //320ns = (ACQPS+1)/SYSCLK
    }

    AdcdRegs.ADCSOC0CTL.bit.CHSEL = 1;  //SOC0 : ADCINA0 채널 변환
    AdcdRegs.ADCSOC1CTL.bit.CHSEL = 2;
    AdcdRegs.ADCSOC2CTL.bit.CHSEL = 3;
    //adc14-adc15
    AdcdRegs.ADCSOC3CTL.bit.CHSEL = 14;
    AdcdRegs.ADCSOC4CTL.bit.CHSEL = 15;

    AdcdRegs.ADCSOC0CTL.bit.ACQPS = acqps;
    AdcdRegs.ADCSOC1CTL.bit.ACQPS = acqps;
    AdcdRegs.ADCSOC2CTL.bit.ACQPS = acqps;
    ///
    AdcdRegs.ADCSOC3CTL.bit.ACQPS = acqps;
    AdcdRegs.ADCSOC4CTL.bit.ACQPS = acqps;

    AdcdRegs.ADCSOC0CTL.bit.TRIGSEL = 0x5;
    AdcdRegs.ADCSOC1CTL.bit.TRIGSEL = 0x5;
    AdcdRegs.ADCSOC2CTL.bit.TRIGSEL = 0x5;
    ////
    AdcdRegs.ADCSOC3CTL.bit.TRIGSEL = 0x5;
    AdcdRegs.ADCSOC4CTL.bit.TRIGSEL = 0x5;

/*   AdcdRegs.ADCSOC0CTL.bit.CHSEL = 0;  //SOC0 : ADCINA0 채널 변환
    AdcdRegs.ADCSOC1CTL.bit.CHSEL = 2;
    AdcdRegs.ADCSOC2CTL.bit.CHSEL = 4;

    AdcdRegs.ADCSOC0CTL.bit.ACQPS = acqps;
    AdcdRegs.ADCSOC1CTL.bit.ACQPS = acqps;
    AdcdRegs.ADCSOC2CTL.bit.ACQPS = acqps;

    AdcdRegs.ADCSOC0CTL.bit.TRIGSEL = 0x9;
    AdcdRegs.ADCSOC1CTL.bit.TRIGSEL = 0x9;
    AdcdRegs.ADCSOC2CTL.bit.TRIGSEL = 0x9;
*/
    AdcdRegs.ADCINTSEL1N2.bit.INT1SEL = 4;  // EOC0가 ADCINT1를 발생
    AdcdRegs.ADCINTSEL1N2.bit.INT1E = 1;    // ADCINT1 활성화
    AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;  // ADCINT1 flag 클리어 확인
    EDIS;
}

void ConfigueDAC(void)
{
EALLOW;
    CpuSysRegs.PCLKCR16.bit.DAC_A = 1; //enable ADC a
    DacaRegs.DACCTL.bit.DACREFSEL = 1; // Internal ref
    DacaRegs.DACCTL.bit.LOADMODE = 0; //load on sys clock
    DacaRegs.DACOUTEN.bit.DACOUTEN = 1; //enable out but
    DacaRegs.DACVALS.all = 0; // output = 0
    //=====================================================
    CpuSysRegs.PCLKCR16.bit.DAC_B = 1; //enable ADC a
    DacbRegs.DACCTL.bit.DACREFSEL = 1; // Internal ref
    DacbRegs.DACCTL.bit.LOADMODE = 0; //load on sys clock
    DacbRegs.DACOUTEN.bit.DACOUTEN = 1; //enable out but
    DacbRegs.DACVALS.all = 0; // output = 0
    EDIS;
}


void Config_DAC_PINB1(void){
    EALLOW;
        CpuSysRegs.PCLKCR16.bit.DAC_C = 1; //enable ADC a
        DaccRegs.DACCTL.bit.DACREFSEL = 1; // Internal ref
        DaccRegs.DACCTL.bit.LOADMODE = 0; //load on sys clock
        DaccRegs.DACOUTEN.bit.DACOUTEN = 1; //enable out but
        DaccRegs.DACVALS.all = 0; // output = 0
    EDIS;
}
