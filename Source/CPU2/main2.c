//###########################################################################
// FILE:   blinky_cpu02.c
// TITLE:  LED Blink Example for F2837xD.
//
// Dual Core Blinky Example.  This example demonstrates how to run a
// implement a standalone application on both cores.
//
//###########################################################################
// $TI Release: F2837xD Support Library v120 $
// $Release Date: Fri Aug 22 15:22:27 CDT 2014 $
//###########################################################################

#include "F28x_Project.h"     // Device Headerfile and Examples Include File
#include "F2837xD_Ipc_drivers.h"

#include "Config.h"
#include "PEFCL_28377_v1.h"
#include "def.h"
#include "Constant.h"
#include "func.h"
#include "math.h"


#ifdef _FLASH
// These are defined by the linker (see device linker command file)
extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadSize;
extern Uint16 RamfuncsRunStart;
#endif

interrupt void adcB_isr(void);
interrupt void adcD_isr(void);
interrupt void tripBDC_isr(void);

void BDC_ISR(Uint16 *operationMode);

union CHECK_FUALT{
	Uint16 		all;
	struct	CHECK_FUALT_BIT{
		Uint16	hwtrip:1;
		Uint16  overVLd:1;		// DC-AC 전압 프로텍션
		Uint16  overVLq:1;
		Uint16	overILd:1;
		Uint16	overILq:1;
		Uint16  overVb:1;		// DC-DC 배터리전압 프로텍션
		Uint16	overIb1:1;		// DC-DC 배터리전류 프로텍션
		Uint16	overIb2:1;		// DC-DC 배터리전류 프로텍션
		Uint16	overVdc:1;		// dc link 프로텍션
		Uint16	rsed:6;
	}bit;
};

union	CHECK_FUALT	checkFault2;

///////////////  variable for BDC		///////////////////////////////////////
struct COEF_1ST		coef1stLPF, coef1stLPF_LC;
struct Z_1ST		zVDCp, zVDCn, zVB, zDff;
struct PID_GAINS 	KiL, Kvb, Kvdc;
struct PID_ERRORS 	Evb, Evdc[2];

float32 openloop_duty_dc = 0.;
float32 cmd_openloop_duty_dc = 0.;


float32 scaleVDC, scaleI;		// scale 값
float32 rVDCp, rVDCn, VDCsen_p, VDCsen_n, VDCsen;

Uint16 goRUN_BDC = 0, startUp, startUpCnt;
Uint16 OperationMode_BDC = 0;
////////////////////////////////////////////////////////////////////////////////

Uint16 resetFault = 0;
Uint16 cntReset2 = 0;
Uint32 adcBIsrCnt = 0, adcDIsrCnt = 0;

Uint32 loopcnt;
Uint32 testDEAD=200;

//Uint32 IL1_delay = 1600, IL2_delay = 2000, IL3_delay = 1600;
//Uint32 IL1_delay = 500, IL2_delay = 1100, IL3_delay = 500;
//Uint32 IL1_delay = 480, IL2_delay = 540, IL3_delay = 480;
Uint32 IL1_delay = 1600, IL2_delay = 1600, IL3_delay = 1600;

float32 rVLinkP, rVLinkN, rVo, rIL[3], rVdcP, rVdcN;
float32 scaleIL, scaleVo, scaleVlink;
float32 Vdc_sen, IL_sen[3], Vo_sen;
float32 Voref = 0, cmdVo = 0, cmdIo = 0, ILref = 0, ILref_step = 0.;
float32 deltaV = 0;
float32 PIconout;
float32 Dff, Dhat[3], Duty[3],Duser[3];

struct PID_GAINS 	KiL, Kvo;
struct PID_ERRORS 	EiL[3], Evo;

struct Z_1ST		zVDC, zIL[3], zVo, zDff;

#define sizeGraph	300
float32 graphData1[sizeGraph];
unsigned int k = 0;
unsigned int x;


void main(void)
{

#ifdef _FLASH
  memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);
#endif
//============================================================================================
// 전역 인터럽트 비활성화
//--------------------------------------------------------------------------------------------
  DINT;

//============================================================================================

//============================================================================================
// 시스템 컨트롤 초기화
//--------------------------------------------------------------------------------------------
	InitSysCtrl();
//============================================================================================

//============================================================================================
// ePWM 모듈로 공급되는  PLLSYSCLK 분주비 설정 ( 0 : /1 , 1 : /2 , default : /2 )
//--------------------------------------------------------------------------------------------
	EALLOW;
	ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV = 0;
	EDIS;
//============================================================================================

//============================================================================================
// 인터럽트 초기화
//--------------------------------------------------------------------------------------------
	InitPieCtrl();
	IER = 0x0000;
	IFR = 0x0000;
	InitPieVectTable();

// Vector Remapping
	EALLOW;
	PieVectTable.ADCB1_INT = &adcB_isr;
	PieVectTable.EPWM1_TZ_INT = &tripBDC_isr;
	EDIS;

	EALLOW;
	CpuSysRegs.PCLKCR2.bit.EPWM1 = 1;
	CpuSysRegs.PCLKCR2.bit.EPWM2 = 1;
	CpuSysRegs.PCLKCR2.bit.EPWM3 = 1;

	CpuSysRegs.PCLKCR13.bit.ADC_B = 1;
	CpuSysRegs.PCLKCR13.bit.ADC_D = 1;
	EDIS;

	conf_ePWM_BDC(1, FREQ_SW_DCDC);
	conf_ePWM_BDC(2, FREQ_SW_DCDC);
	conf_ePWM_BDC(3, FREQ_SW_DCDC);
	//conf_ePWM_BDC(4, FREQ_SW_DCDC);

	updateDeadtime(1, testDEAD);
	updateDeadtime(2, testDEAD);
	updateDeadtime(3, testDEAD);

	updatePhase(1, 0.0);
	updatePhase(2, 120.0);
	//updatePhase(3, 240.0);
	updatePhase(3, -120.0);

	ePWM_SSync_SOCASEL(1, 0x06, IL1_delay);
	ePWM_SSync_SOCASEL(2, 0x06, IL2_delay);
	ePWM_SSync_SOCASEL(3, 0x06, IL3_delay);

	conf_ADC_B();
	conf_ADC_D();

	conf_ePWM_Trip(1, 0x2, 1);      // IL1,2,3 전류 하드웨어프로텍션 제거 0x02 -> 0x00 (180207)
	conf_ePWM_Trip(2, 0x2, 0);
	conf_ePWM_Trip(3, 0x2, 0);

	EALLOW;

	EPwm1Regs.TZSEL.bit.DCAEVT1 = 1;
	EPwm1Regs.DCTRIPSEL.bit.DCAHCOMPSEL = 3;
	EPwm1Regs.TZDCSEL.bit.DCAEVT1 = 2;
	EPwm1Regs.TZEINT.bit.DCAEVT1 = 1;

	EDIS;

	scaleIL 	= getScaleGain(1.0/2000.0*100.0, 10.0/16.0);
	scaleVo		= getScaleGain(8*0.47/(210.0*6+0.47), 11.0/3.9);
	scaleVlink	= getScaleGain(8*0.43/(210.0*6+0.43), 56.0/28.0);

	coef1stLPF = get1stLPFcoef(1000.0, FREQ_SAMPLING);
	coef1stLPF_LC = get1stLPFcoef(100.0, FREQ_SAMPLING);


	PieCtrlRegs.PIEIER1.bit.INTx2 = 1;	// PIE 인터럽트(ADCB1INT) 활성화
	PieCtrlRegs.PIEIER2.bit.INTx1 = 1;
	IER |= M_INT1; 		//Enable group 1 interrupts
	IER |= M_INT2;		//Enable group 2 interrupts

	EALLOW;
	CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
	EDIS;

	Dff = 0;

	zVDC.in = 0.0; 		zVDC.out = 0.0;
	zIL[0].in = 0.0;	zIL[0].out = 0.0;
	zIL[1].in = 0.0;	zIL[1].out = 0.0;
	zIL[2].in = 0.0;	zIL[2].out = 0.0;
	zVo.in = 0.0;		zVo.out = 0.0;
	zDff.in = 0.0;		zDff.out = 0.0;

	cmdVo = 50.0;
	cmdIo = 3.0;

	// 180110 - Vo : 0.015, 5.0 / IL - 0.001, 0.051
	//KiL.P = 0.01;		KiL.I = 0.1;		KiL.a = 1.0 / KiL.P; //원래
	//Kvo.P = 0.1;		Kvo.I = 3.0;		Kvo.a = 1.0 / Kvo.P;

	Kvo.P = 2.0;		Kvo.I = 30.0;		Kvo.a = 1.0 / Kvo.P;
	KiL.P = 0.001;		KiL.I = 0.05;		KiL.a = 1.0 / KiL.P; //수한


	EINT;  // Enable Global interrupt INTM
	ERTM;  // Enable Global realtime interrupt DBGM

	while(1){
		loopcnt++;
	}

}

Uint32 LEDCnt = 0;

interrupt void adcB_isr(void)
{
    LEDCnt++;
    if( (LEDCnt % 10000)==0 )   LED4_TOGGLE;

    Uint16 adcIL[4], adcVLinkP, adcVLinkN, adcVo;
	Uint16 BIASv1_65;

	adcIL[0]	= AdcbResultRegs.ADCRESULT0;
	adcIL[1]	= AdcbResultRegs.ADCRESULT1;
	adcIL[2]	= AdcbResultRegs.ADCRESULT2;

	adcVo 		= AdcdResultRegs.ADCRESULT4;
	adcVLinkP	= AdcaResultRegs.ADCRESULT6;
	adcVLinkN	= AdcaResultRegs.ADCRESULT7;

	BIASv1_65	= (float)(AdcdResultRegs.ADCRESULT0);


	rIL[0]	= scaleADC(adcIL[0], 234.0, scaleIL);
	rIL[1]	= scaleADC(adcIL[1], 234.0, scaleIL);
	rIL[2]	= scaleADC(adcIL[2], 240.0, scaleIL);

	rVo 	= scaleADC(adcVo, 0., scaleVo);
	rVdcP 	= scaleADC(adcVLinkP, 0., scaleVlink);
	rVdcN 	= scaleADC(adcVLinkN, 0., scaleVlink);


	IL_sen[0]	= firstLPF(rIL[0], &zIL[0], coef1stLPF);
	IL_sen[1]	= firstLPF(rIL[1], &zIL[1], coef1stLPF);
	IL_sen[2]	= firstLPF(rIL[2], &zIL[2], coef1stLPF);

	Vo_sen 		= firstLPF(rVo, &zVo, coef1stLPF);
	Vdc_sen		= firstLPF((rVdcP+rVdcN), &zVDC, coef1stLPF);

    //Software Protection//
    if(IL_sen[0]>90.) { OperationMode_BDC = FAULT; LED6_ON; }
    if(IL_sen[1]>90.) { OperationMode_BDC = FAULT; LED6_ON; }
    if(IL_sen[2]>90.) { OperationMode_BDC = FAULT; LED6_ON; }
    //-------------------//

#if 1

	graphData1[k] = IL_sen[0];

	k++;
	if(k > sizeGraph)	k = 0;

#endif


	switch(OperationMode_BDC){
	case SYSTEM_STANDBY:

		DCDC_BUFF_OFF;
		goRUN_BDC = 0;
		LED5_OFF;
		LED6_OFF;

		forceOFFPWM(1, 1);
		forceOFFPWM(2, 1);
		forceOFFPWM(3, 1);
		forceOFFPWM(4, 1);

		Duser[0] = 0.;
		Duser[1] = 0.;
		Duser[2] = 0.;

		break;

	case SYSTEM_READY:

		resetError(&Evo);
		resetError(&EiL[0]);
		resetError(&EiL[1]);
		resetError(&EiL[2]);

		DCDC_BUFF_ON;
		Dff = 0.0;
		zDff.in = 0.0;		zDff.out = 0.0;
		Voref = Vo_sen;
		ILref= 0.333333 * (IL_sen[0] + IL_sen[1] + IL_sen[2]);

		Dhat[0] = 0;
		Dhat[1] = 0;
		Dhat[2] = 0;

		if(goRUN_BDC == 1){
			forceOFFPWM(1, 0);		// Boost PWM forcing disable
			forceOFFPWM(2, 0);
			forceOFFPWM(3, 0);
			OperationMode_BDC = SYSTEM_RUN;

			//ILref = 3.0;
			//ILref = deltaV;
		}
		else{
			forceOFFPWM(1, 1);		// Boost PWM forcing disable
			forceOFFPWM(2, 1);
			forceOFFPWM(3, 1);
		}

		LED5_ON;

		break;

	case SYSTEM_RUN:

		if(!goRUN_BDC)		OperationMode_BDC = SYSTEM_READY;

		if(Voref < cmdVo-DELTA_V_REF)			Voref += DELTA_V_REF;
		else if(Voref > cmdVo+DELTA_V_REF)		Voref -= DELTA_V_REF;
		else									Voref = cmdVo;

		PIconout = PIcon(Voref+deltaV, Vo_sen, &Evo, Kvo);			// Voltage controller
		ILref = Limiter(PIconout, 170, -1.0);						// Limiter
		Evo.w = (PIconout - ILref) * Kvo.a;

/*		if(ILref < cmdIo-DELTA_I_REF)			ILref += DELTA_I_REF;
		else if(ILref > cmdIo+DELTA_I_REF)		ILref -= DELTA_I_REF;
		else									ILref = cmdIo;
		//ILref = cmdIo;
*/
		PIconout = PIcon(0.33333*(ILref+ILref_step), IL_sen[0], &EiL[0], KiL);	// Voltage controller
		Dhat[0] = Limiter(PIconout, 0.1, -0.05);					// Limiter
		EiL[0].w = (PIconout - Dhat[0]) * KiL.a;

		PIconout = PIcon(0.33333*(ILref+ILref_step), IL_sen[1], &EiL[1], KiL);	// Voltage controller
		Dhat[1] = Limiter(PIconout, 0.1, -0.05);					// Limiter
		EiL[1].w = (PIconout - Dhat[1]) * KiL.a;

		PIconout = PIcon(0.33333*(ILref+ILref_step), IL_sen[2], &EiL[2], KiL);	// Voltage controller
		Dhat[2] = Limiter(PIconout, 0.1, -0.05);					// Limiter
		EiL[2].w = (PIconout - Dhat[2]) * KiL.a;

		Dff = Voref/Vdc_sen;
		Dff = Limiter(Dff, 1.0, 0.0);

/*		ePWM_SSync_SOCASEL(1, 0x06, IL1_delay);
		ePWM_SSync_SOCASEL(2, 0x06, IL2_delay);
		ePWM_SSync_SOCASEL(3, 0x06, IL3_delay);*/


		if( (LEDCnt % 5000)==0 )   LED5_TOGGLE;

		break;

	case FAULT:

		DCDC_BUFF_OFF;

		goRUN_BDC = 0;

		forceOFFPWM(1, 1);		// SRC PWM force OFF
		forceOFFPWM(2, 1);
		forceOFFPWM(3, 1);		// SRC PWM force OFF

		if(resetFault)	goto resetMode;

		break;

	case RESET:
		resetMode:
		OperationMode_BDC = RESET;
		resetFault = 0;

		LED4_OFF;
		LED5_OFF;
		LED6_OFF;

		cntReset2++;

		break;

	case OPENLOOP_MODE:

		forceOFFPWM(1, 0);
		forceOFFPWM(2, 0);
		forceOFFPWM(3, 0);

		if(openloop_duty_dc < cmd_openloop_duty_dc-DELTA_OPENLOOP_DUTY)			openloop_duty_dc += DELTA_OPENLOOP_DUTY;
		else if(openloop_duty_dc > cmd_openloop_duty_dc+DELTA_OPENLOOP_DUTY)	openloop_duty_dc -= DELTA_OPENLOOP_DUTY;
		else																	openloop_duty_dc = cmd_openloop_duty_dc;

		Dff = openloop_duty_dc;
		Dhat[0] = 0;
		Dhat[1] = 0;
		Dhat[2] = 0;

		updateDeadtime(1, testDEAD);
		updateDeadtime(2, testDEAD);
		updateDeadtime(3, testDEAD);

		ePWM_SSync_SOCASEL(1, 0x06, IL1_delay);
		ePWM_SSync_SOCASEL(2, 0x06, IL2_delay);
		ePWM_SSync_SOCASEL(3, 0x06, IL3_delay);

		break;

	default:

		break;

	}

	Duty[0] = Dff + Dhat[0] + Duser[0];
	Duty[1] = Dff + Dhat[1] + Duser[1];
	Duty[2] = Dff + Dhat[2] + Duser[2];


	Duty[0] = Limiter(Duty[0], 1.0, 0.0);
	Duty[1] = Limiter(Duty[1], 1.0, 0.0);
	Duty[2] = Limiter(Duty[2], 1.0, 0.0);

	updateDuty(1, Duty[0]);
	updateDuty(2, Duty[1]);
	updateDuty(3, Duty[2]);


	//DCDC_BUFF_OFF;
	GpioDataRegs.GPCDAT.bit.GPIO75 = 0;
	adcBIsrCnt++;

	AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}


interrupt void adcD_isr(void)
{
	adcDIsrCnt++;
	AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}




Uint32 TripBDCCnt = 0;
interrupt void tripBDC_isr(void)
{
	LED5_ON;
	LED6_ON;

	forceOFFPWM(1, 1);
	forceOFFPWM(2, 1);
	forceOFFPWM(3, 1);
	DCDC_BUFF_OFF;

	goRUN_BDC = 0;

	OperationMode_BDC = FAULT;
	checkFault2.bit.hwtrip = 1;

	EALLOW;
	EPwm1Regs.TZCLR.bit.OST = 1;
	EPwm2Regs.TZCLR.bit.OST = 1;
	EPwm3Regs.TZCLR.bit.OST = 1;

	EPwm1Regs.TZCLR.bit.DCAEVT1 = 1;
	EPwm2Regs.TZCLR.bit.DCAEVT1 = 1;
	EPwm3Regs.TZCLR.bit.DCAEVT1 = 1;

	EPwm1Regs.TZCLR.bit.INT = 1;
	EPwm2Regs.TZCLR.bit.INT = 1;
	EPwm3Regs.TZCLR.bit.INT = 1;
	EDIS;



	TripBDCCnt++;

	PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;
}






