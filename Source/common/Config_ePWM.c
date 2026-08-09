// Config_ePWM.c
// Editor: NGUYEN VAN MINH TAM (Based on the PEFCL's material)

#include "Config_ePWM.h"

volatile struct EPWM_REGS *ePWM_Regs[] = {
		0,		//  0 address dummy!!
		&EPwm1Regs,
		&EPwm2Regs,
		&EPwm3Regs,
		&EPwm4Regs,
		&EPwm5Regs,
		&EPwm6Regs,
		&EPwm7Regs,
		&EPwm8Regs,
		&EPwm9Regs,
		&EPwm10Regs,
		&EPwm11Regs,
		&EPwm12Regs
};


void conf_ePWM_Sym(Uint16 ch, float32 freq)  // count up-down
{
    // set switching frequency
    ePWM_Regs[ch]->TBPRD = (Uint16)(CPU_SPD/freq/4.0);      // for UPDOWN Counter

    // Set actions for main switch and snubber switch: EPWMxA is a main switch, EPWMxB is a snubber switch
    ePWM_Regs[ch]->AQCTLA.bit.CAD = AQ_SET;     // EPWM1A - Main switch ON, main switch is configured dependent of CMPA, active high , see page 1705 of spruhm8g.pdf (TMS320F2837xD Dual-Core Delfino)
    ePWM_Regs[ch]->AQCTLA.bit.CAU = AQ_CLEAR;   // EPWM1A - Main switch OFF, main switch is configured dependent of CMPA, active high , see page 1705 of spruhm8g.pdf (TMS320F2837xD Dual-Core Delfino)
    ePWM_Regs[ch]->AQCTLB.bit.CBD = AQ_SET;     // EPWM1B - Snubber switch ON, Counter equals CMPB on down-count (CBD) --> set
    ePWM_Regs[ch]->AQCTLB.bit.CAD = AQ_CLEAR;   // EPWM1B - Snubber switch OFF, when main switch turn ON), Counter equals CMPA on down-count (CAD)
    ePWM_Regs[ch]->AQCTLB.bit.CAU = AQ_NO_ACTION; // EPWM1B - Do Nothing, Counter equals CMPA on up-count (CAU)
    ePWM_Regs[ch]->AQCTLB.bit.CBU = AQ_NO_ACTION; // EPWM1B - Do Nothing, Counter equals CMPB on up-count (CBU)

    // set phase, channel 1 is MASTER phase
    if(ch==1){
        ePWM_Regs[ch]->TBCTL.bit.PHSEN = TB_DISABLE;        // Phase Reg. DISABLE
        ePWM_Regs[ch]->TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;    // Sync Output Select  = 01
                                                            // 00 = EPWMxSYNCI
                                                            // 01 = CTR = 0  --> counter of EPWM1 is selected as the reference phase angle
                                                            // 10 = CTR = CMPB
                                                            // 11 = disable SyncOut

    }
    else if(ch!=1){
        ePWM_Regs[ch]->TBCTL.bit.PHSEN = TB_ENABLE;         // TBPHS on EPWMxSYNCI signal
        ePWM_Regs[ch]->TBCTL.bit.SYNCOSEL = TB_SYNC_IN;     // Sync Output Select  = 00
                                                            // 00 = EPWMxSYNCI  --> phase of other EPWM channel refer to EPWM1
                                                            // 01 = CTR = 0
                                                            // 10 = CTR = CMPB
                                                            // 11 = disable SyncOut
    }

    SyncSocRegs.SYNCSELECT.bit.ECAP4SYNCIN = 0x0;   // Selects Sync Input Source for ECAP4: (in case of using two time clock sources)
                                                    // 000: EPWM1SYNCOUT selected
                                                    // 001: EPWM4SYNCOUT selected
                                                    // 010: EPPW7SYNCOUT selected
                                                    // 011: EPWM10SYNCOUT selected
                                                    // 100: ECAP1SYNCOUT selected
                                                    // 101: EXTSYNCIN1 selected
                                                    // 110: EXTSYNCIN2 selected
                                                    // 111: Reserved
                                                    // Notes:
                                                    // [1] Reserved position defaults to 000 selection
                                                    // Reset type: CPU1.SYSRSn
                                                    // ----> for more informations ==> see page 1689 of spruhm8g.pdf

    SyncSocRegs.SYNCSELECT.bit.SYNCOUT = 0x1;       // Select Syncout Source:
                                                    // 00: EPWM1SYNCOUT selected
                                                    // 01: EPWM4SYNCOUT selected
                                                    // 10: EPPW7SYNCOUT selected
                                                    // 11: EPWM10SYNCOUT selected
                                                    // Reset type: CPU1.SYSRSn
                                                    // ----> for more informations ==> see page 1689 of spruhm8g.pdf


    // Setup TBCLK
    ePWM_Regs[ch]->TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up-down
    ePWM_Regs[ch]->TBCTL.bit.PRDLD = TB_SHADOW;     // period Shadow Load:  0 --> Load on CTR, 1 --> Load immediately
    ePWM_Regs[ch]->TBCTL.bit.HSPCLKDIV = TB_DIV1;   // Clock Pre-scale:  000 = /1, 001 = /2, 010 = /4, ...

    // Set Counter Compare Control Register, see page 1776 of spruhm8g.pdf (TMS320F2837xD Dual-Core Delfino)
    ePWM_Regs[ch]->CMPCTL.bit.SHDWAMODE = CC_SHADOW;    // Load registers every ZERO ????
    ePWM_Regs[ch]->CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    ePWM_Regs[ch]->CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    ePWM_Regs[ch]->CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    updateDuty_Sym(ch, 0.0);
    updatePhase(ch, 0.0);  // use for phase shift control
}

void updateDuty_Sym(Uint16 ch, float32 duty)
{
    ePWM_Regs[ch]->CMPA.bit.CMPA = (Uint32)((float32)ePWM_Regs[ch]->TBPRD * duty);
    ePWM_Regs[ch]->CMPB.bit.CMPB = (Uint32)((float32)ePWM_Regs[ch]->TBPRD * (duty+0.14)); // 0.14 represents for 7% of one switching period, which is a duty cycle of the snubber switch
    //ePWM_Regs[ch]->CMPB.bit.CMPB = (Uint32)((float32)ePWM_Regs[ch]->TBPRD * (1-duty));   // No need in the complementary mode
}

void ePWM_MSync_SOCASEL(Uint16 ch, Uint16 SOCASEL, Uint16 delayCLK)   // sampling every 1 PWM period
{
	ePWM_Regs[ch]->ETSEL.bit.SOCAEN = 1;        // Enable SOC on A group
	ePWM_Regs[ch]->ETSEL.bit.SOCASEL = SOCASEL; // select SOC from CMPB when the timer is incrementing
	                                            // EPWMxSOCB / A Select
	                                            // 000 = reserved
                                                // 001 = CTR = 0   --> ideally used, however, because of the delay of the LC filter, need to compensate the delay (shift the SOC abit by using CMPA or CMPB)
                                                // 010 = CTR = PRD
                                                // 011 = reserved
                                                // 100 = CTRU = CMPA
                                                // 101 = CTRD = CMPA
                                                // 110 = CTRU = CMPB
                                                // 111 = CTRD = CMPB

	ePWM_Regs[ch]->ETPS.bit.SOCAPRD = 1;        // Generate pulse on 1st event*/
	ePWM_Regs[ch]->CMPB.bit.CMPB = delayCLK;    // the delay value will be experimentally measured and set (compared the sensing value and the average value of inductor current)
	                                            // The channel used as a reference for delay should not be used in other PWM scheme.
}

void ePWM_SSync_SOCASEL(Uint16 ch, Uint16 SOCASEL, Uint16 delayCLK)   // sampling every 2 PWM period
{
	ePWM_Regs[ch]->ETSEL.bit.SOCAEN = 1;        // Enable SOC on A group
	ePWM_Regs[ch]->ETSEL.bit.SOCASEL = SOCASEL; // select SOC from CMPB when the timer is incrementing (refer to page 1861 of spruhm8g)
	ePWM_Regs[ch]->ETPS.bit.SOCAPRD = 2;        // Generate pulse on 2nd event*/
	ePWM_Regs[ch]->CMPA.bit.CMPA = delayCLK;
    //ePWM_Regs[ch]->CMPB.bit.CMPB = delayCLK;
}

void updateDuty(Uint16 ch, float32 duty)
{
	ePWM_Regs[ch]->CMPA.bit.CMPA = (Uint32)((float32)ePWM_Regs[ch]->TBPRD * duty);
}
void updateDutyB(Uint16 ch, float32 duty)
{
    ePWM_Regs[ch]->CMPB.bit.CMPB = (Uint32)((float32)ePWM_Regs[ch]->TBPRD * duty);
}
void updateDutyBoost(Uint16 ch, float32 duty)
{
    ePWM_Regs[ch]->CMPA.bit.CMPA = (Uint32)((float32)ePWM_Regs[ch]->TBPRD * (1-duty));

}
void updatePhase(int16 ch, float32 phase)
{
	if(phase == 0)	ePWM_Regs[ch]->TBPHS.bit.TBPHS = 0;
	else if(phase > 0){
		ePWM_Regs[ch]->TBCTL.bit.PHSDIR = 0;
		ePWM_Regs[ch]->TBPHS.bit.TBPHS = (int16)((float32)ePWM_Regs[ch]->TBPRD * phase * 0.00555555555);
	}
	else{
		ePWM_Regs[ch]->TBCTL.bit.PHSDIR = 1;
		ePWM_Regs[ch]->TBPHS.bit.TBPHS = (int16)((float32)ePWM_Regs[ch]->TBPRD * -phase * 0.00555555555);
	}
}

void updateFreq(Uint16 ch, float32 freq)
{
    ePWM_Regs[ch]->TBPRD = (Uint16)(CPU_SPD/freq/4.0);      // for UPDOWN Counter
    //ePWM_Regs[ch]->TBPRD = (Uint16)(CPU_SPD/freq) - 1;		// for UP or DOWN Counter
}

void updateDeadtime(Uint16 ch, float32 dt_ns)
{
	Uint16 deadband;

	deadband = (Uint16)(dt_ns * 0.5*CPU_SPD * 1e-9);  // need to check, should not 0.5

	ePWM_Regs[ch]->DBFED.bit.DBFED = deadband;
	ePWM_Regs[ch]->DBRED.bit.DBRED = deadband;
}



void forceOFFPWM(Uint16 ch, Uint16 off)  // Both PWM A and B are OFF
{
		if(off == 0){
			ePWM_Regs[ch]->AQCSFRC.bit.CSFA = 0x0;
			ePWM_Regs[ch]->AQCSFRC.bit.CSFB = 0x0;
		}
		else if(off == 1){
			ePWM_Regs[ch]->AQCSFRC.bit.CSFA = 0x1;
			ePWM_Regs[ch]->AQCSFRC.bit.CSFB = 0x2;  //  force continuous low on output
		}
}

void forceOFFPWMxB(Uint16 ch, Uint16 off)  // PWMxB is always OFF
{
        if(off == 0){
            ePWM_Regs[ch]->AQCSFRC.bit.CSFA = 0x0;
            ePWM_Regs[ch]->AQCSFRC.bit.CSFB = 0x1;
        }
        else if(off == 1){
            ePWM_Regs[ch]->AQCSFRC.bit.CSFA = 0x1;
            ePWM_Regs[ch]->AQCSFRC.bit.CSFB = 0x1;  //  force continuous low on output
        }
}

void conf_ePWM_Trip(Uint16 ch, Uint16 tripch, Uint16 interruptEnable)
{
	EALLOW;
// my comment
	ePWM_Regs[ch]->TZSEL.all = tripch << 8; // 6 of 8 - high bit : One-Short Trip Zone
	                                        //6 of 8 - low bit : cycle by cycle tripzone
	ePWM_Regs[ch]->TZCTL.bit.TZA = TZ_FORCE_LO; // force low epwm[ch]
	ePWM_Regs[ch]->TZCTL.bit.TZB = TZ_FORCE_LO; // force low epwm[ch]
	/** TZCTL.bit.TZx (x=A or B)
                00 = high impedance
                01 = force high
                10 = force low
                11 = do nothing (disable)
	 * */


    EPwm1Regs.TZSEL.bit.DCAEVT1 = 1;

    EPwm1Regs.DCTRIPSEL.bit.DCAHCOMPSEL = 3;
    EPwm1Regs.TZDCSEL.bit.DCAEVT1 = 2;
    EPwm1Regs.TZEINT.bit.DCAEVT1 = 1;

	if(interruptEnable)	ePWM_Regs[ch]->TZEINT.bit.OST = 1;

	EDIS;
}


void ePWM_SOCSEL(Uint16 ch,  Uint16 SOCASEL, Uint16 Number_envent)
{
    ePWM_Regs[ch]->ETSEL.bit.SOCAEN = 1;        // Enable SOC on A group
    ePWM_Regs[ch]->ETSEL.bit.SOCASEL = SOCASEL; // select SOC from CMPB when the timer is incrementing (refer to page 1861 of spruhm8g)
    ePWM_Regs[ch]->ETPS.bit.SOCAPRD = Number_envent;        // Generate pulse on 2nd event*/
}

void INIT_EPWM(Uint16 ch, float32 freq)  // count up
{

    ePWM_Regs[ch]->TBPRD = (Uint16)(CPU_SPD/freq/2.0);     // for UP Counter

    if(ch==1){
        ePWM_Regs[ch]->TBCTL.bit.PHSEN = TB_DISABLE;
        ePWM_Regs[ch]->TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;
        ePWM_Regs[ch]->TBPHS.bit.TBPHS = 0x0000;        // Phase is 0
        //ePWM_Regs[ch]->TBCTR = 0x0000;                  // Clear counter
   }
    else if(ch!=1){
        ePWM_Regs[ch]->TBCTL.bit.PHSEN = TB_ENABLE;
        ePWM_Regs[ch]->TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
        //ePWM_Regs[ch]->TBCTR = 0x0000;
    }

   // SyncSocRegs.SYNCSELECT.bit.ECAP4SYNCIN = 0x0;
   // SyncSocRegs.SYNCSELECT.bit.SYNCOUT = 0x1;

        ePWM_Regs[ch]->AQCTLA.bit.ZRO = AQ_SET;            // Clear PWM2A on Period
        ePWM_Regs[ch]->AQCTLA.bit.CAU = AQ_CLEAR;              // Set PWM2A on event A,
                                                                            // up count

        ePWM_Regs[ch]->AQCTLB.bit.PRD = AQ_SET;            // Clear PWM2B on Period
        ePWM_Regs[ch]->AQCTLB.bit.CBU = AQ_CLEAR;              // Set PWM2B on event B,
                                                                           // up count

    // Setup TBCLK
    ePWM_Regs[ch]->TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up-down
    ePWM_Regs[ch]->TBCTL.bit.PRDLD = TB_SHADOW;     // period Shadow Load:  0 --> Load on CTR, 1 --> Load immediately
    ePWM_Regs[ch]->TBCTL.bit.HSPCLKDIV = TB_DIV1;   // Clock Pre-scale:  000 = /1, 001 = /2, 010 = /4, ...

    // Set Counter Compare Control Register, see page 1776 of spruhm8g.pdf (TMS320F2837xD Dual-Core Delfino)
    ePWM_Regs[ch]->CMPCTL.bit.SHDWAMODE = CC_SHADOW;    // Load registers every ZERO ????
    ePWM_Regs[ch]->CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    ePWM_Regs[ch]->CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    ePWM_Regs[ch]->CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    //update_Duty(ch, 0.0);
    //updatePhase(ch, 0.0);
    updateFreq(ch, freq);
    //updateDeadtime(ch, 0.0);
}

void A3P2L_ePWM(Uint16 ch, float32 freq)  // count up-down
{
    // Set switching frequency
    ePWM_Regs[ch]->TBPRD = (float)(CPU_SPD/freq/4.0);      // for UPDOWN Counter

    // Set actions
    ePWM_Regs[ch]->AQCTLA.bit.CAU = AQ_CLEAR;
    ePWM_Regs[ch]->AQCTLA.bit.CAD = AQ_SET;
    ePWM_Regs[ch]->AQCTLB.bit.CBU = AQ_CLEAR;
    ePWM_Regs[ch]->AQCTLB.bit.CBD = AQ_SET;

    switch (ch)
    {
        case 1: case 4: case 7:
            ePWM_Regs[ch]->TBCTL.bit.PHSEN = TB_DISABLE;        // Phase Reg. DISABLE
            ePWM_Regs[ch]->TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;    // Sync Output Select  = 01
                                                                // 00 = EPWMxSYNCI
                                                                // 01 = CTR = 0
                                                                // 10 = CTR = CMPB
                                                                // 11 = disable SyncOut
            ePWM_Regs[ch]->TBCTR = 0x0000;                      // Clear counter
            break;
        case 2: case 3: case 5: case 6: case 8: case 9: case 11: case 12:
            ePWM_Regs[ch]->TBCTL.bit.PHSEN = TB_ENABLE;         // TBPHS on EPWMxSYNCI signal
            ePWM_Regs[ch]->TBCTL.bit.SYNCOSEL = TB_SYNC_IN;     // Sync Output Select  = 00
                                                                // 00 = EPWMxSYNCI
                                                                // 01 = CTR = 0
                                                                // 10 = CTR = CMPB
                                                                // 11 = disable SyncOut
            ePWM_Regs[ch]->TBCTR = 0x0000;                      // Clear counter
            break;
        case 10:
            EPwm10Regs.TBCTL.bit.PHSEN = TB_ENABLE;             // TBPHS on EPWMxSYNCI signal
            EPwm10Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;         // Sync Output Select  = 00
                                                                // 00 = EPWMxSYNCI  --> phase of other EPWM channel refer to EPWM1
                                                                // 01 = CTR = 0
                                                                // 10 = CTR = CMPB
                                                                // 11 = disable SyncOut
            EALLOW;
            SyncSocRegs.SYNCSELECT.bit.EPWM10SYNCIN = 2;        // Selects Sync Input Source for EPWM10: EPWM7SYNCOUT selected
            EDIS;
            EPwm10Regs.TBCTR = 0x0000;                          // Clear counter
            break;

        default:
            // Do nothing
            break;
    }

    // Setup TBCLK, counter mode
    ePWM_Regs[ch]->TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;     // Count up-down
    ePWM_Regs[ch]->TBCTL.bit.PRDLD = TB_SHADOW;             // period Shadow Load:  0 --> Load on CTR, 1 --> Load immediately
    ePWM_Regs[ch]->TBCTL.bit.HSPCLKDIV = TB_DIV1;           // Clock Pre-scale:  000 = /1, 001 = /2, 010 = /4, ...
    ePWM_Regs[ch]->TBCTL.bit.CLKDIV = TB_DIV1;

    // Set Counter Compare Control Register, see page 1776 of spruhm8g.pdf (TMS320F2837xD Dual-Core Delfino)
    ePWM_Regs[ch]->CMPCTL.bit.SHDWAMODE = CC_SHADOW;        // Load registers every ZERO ????
    ePWM_Regs[ch]->CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    ePWM_Regs[ch]->CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    ePWM_Regs[ch]->CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
}

void A3P2L_ePWM_180(Uint16 ch, float32 freq)  // count up-down
{
    // set switching frequency
    ePWM_Regs[ch]->TBPRD = (Uint16)(CPU_SPD/freq/4.0);      // for UPDOWN Counter

    // Set actions for main switch and snubber switch: EPWMxA is a main switch, EPWMxB is a snubber switch
    ePWM_Regs[ch]->AQCTLA.bit.CAD = AQ_SET;     // EPWM1A - Main switch ON, main switch is configured dependent of CMPA, active high , see page 1705 of spruhm8g.pdf (TMS320F2837xD Dual-Core Delfino)
    ePWM_Regs[ch]->AQCTLA.bit.CAU = AQ_CLEAR;   // EPWM1A - Main switch OFF, main switch is configured dependent of CMPA, active high , see page 1705 of spruhm8g.pdf (TMS320F2837xD Dual-Core Delfino)
    ePWM_Regs[ch]->AQCTLB.bit.CBD = AQ_SET;     // EPWM1B - Snubber switch ON, Counter equals CMPB on down-count (CBD) --> set
    ePWM_Regs[ch]->AQCTLB.bit.CBU = AQ_CLEAR;   // EPWM1B - Snubber switch OFF, when main switch turn ON), Counter equals CMPA on down-count (CAD)
    //ePWM_Regs[ch]->AQCTLB.bit.CAU = AQ_NO_ACTION; // EPWM1B - Do Nothing, Counter equals CMPA on up-count (CAU)
    //ePWM_Regs[ch]->AQCTLB.bit.CBU = AQ_NO_ACTION; // EPWM1B - Do Nothing, Counter equals CMPB on up-count (CBU)

    // set phase, channel 1 is MASTER phase
    if(ch==1){
        ePWM_Regs[ch]->TBCTL.bit.PHSEN = TB_DISABLE;        // Phase Reg. DISABLE
        ePWM_Regs[ch]->TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;    // Sync Output Select  = 01
                                                            // 00 = EPWMxSYNCI
                                                            // 01 = CTR = 0  --> counter of EPWM1 is selected as the reference phase angle
                                                            // 10 = CTR = CMPB
                                                            // 11 = disable SyncOut
        ePWM_Regs[ch]->TBCTR = 0x0000;                        // Clear counter

    }
    else if(ch!=1){
        ePWM_Regs[ch]->TBCTL.bit.PHSEN = TB_ENABLE;         // TBPHS on EPWMxSYNCI signal
        ePWM_Regs[ch]->TBCTL.bit.SYNCOSEL = TB_SYNC_IN;     // Sync Output Select  = 00
                                                            // 00 = EPWMxSYNCI  --> phase of other EPWM channel refer to EPWM1
                                                            // 01 = CTR = 0
                                                            // 10 = CTR = CMPB
                                                            // 11 = disable SyncOut
        ePWM_Regs[ch]->TBCTR = 0x0000;                  // Clear counter
    }

    // Setup TBCLK, counter mode
    ePWM_Regs[ch]->TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up-down
    ePWM_Regs[ch]->TBCTL.bit.PRDLD = TB_SHADOW;     // period Shadow Load:  0 --> Load on CTR, 1 --> Load immediately
    ePWM_Regs[ch]->TBCTL.bit.HSPCLKDIV = TB_DIV1;   // Clock Pre-scale:  000 = /1, 001 = /2, 010 = /4, ...
    ePWM_Regs[ch]->TBCTL.bit.CLKDIV = TB_DIV1;

    // Set Counter Compare Control Register, see page 1776 of spruhm8g.pdf (TMS320F2837xD Dual-Core Delfino)
    ePWM_Regs[ch]->CMPCTL.bit.SHDWAMODE = CC_SHADOW;    // Load registers every ZERO ????
    ePWM_Regs[ch]->CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    ePWM_Regs[ch]->CMPCTL.bit.LOADAMODE = CC_CTR_PRD;
    ePWM_Regs[ch]->CMPCTL.bit.LOADBMODE = CC_CTR_PRD;

}




void configure_deadband(Uint16 ch, Uint16 type)
{
    // Deadband Config   // Note that Deadband-> CMP_B depend on CMP_A

     ePWM_Regs[ch]->DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;  // DB_FULL_ENABLE EPwm2A -> RED // copy old code
     ePWM_Regs[ch]->DBCTL.bit.IN_MODE = DBA_ALL;  // B=duty  DBA_ALL        EPwm2A = source for RED & FED
     ePWM_Regs[ch]->DBCTL.bit.POLSEL = type;     // ref epwm_define.h DB_ACTV_LOC
 //    ePWM_Regs[ch]->DBCTL.bit.OUTSWAP=0x3; // swap duty boost

}
void configure_deadband_boost(Uint16 ch, Uint16 type)
{
    // Deadband Config   // Note that Deadband-> CMP_B depend on CMP_A

     ePWM_Regs[ch]->DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;  // DB_FULL_ENABLE EPwm2A -> RED // copy old code
     ePWM_Regs[ch]->DBCTL.bit.IN_MODE = DBA_ALL;  // B=duty  DBA_ALL        EPwm2A = source for RED & FED
     ePWM_Regs[ch]->DBCTL.bit.POLSEL = type;     // ref epwm_define.h DB_ACTV_LOC
     ePWM_Regs[ch]->DBCTL.bit.OUTSWAP = 0x3; // swap duty boost

}
void configure_deadband_rec(Uint16 ch, Uint16 type)
{
    // Deadband Config   // Note that Deadband-> CMP_B depend on CMP_A

     ePWM_Regs[ch]->DBCTL.bit.OUT_MODE = DB_DISABLE;  // DB_FULL_ENABLE EPwm2A -> RED // copy old code
     ePWM_Regs[ch]->DBCTL.bit.IN_MODE = DBB_RED_DBA_FED;  // B=duty  DBA_ALL        EPwm2A = source for RED & FED
     ePWM_Regs[ch]->DBCTL.bit.POLSEL = type;     // ref epwm_define.h DB_ACTV_LOC
 //    ePWM_Regs[ch]->DBCTL.bit.OUTSWAP=0x3; // swap duty boost

}


void ForceOFFPWM(Uint16 ch, Uint16 off){
    if (off ==1){
        ePWM_Regs[ch]->AQCSFRC.bit.CSFA = 0x1;//0x1 means forces a continuous low on output X
        ePWM_Regs[ch]->AQCSFRC.bit.CSFB = 0x1;//0x2 means forces a continuous high on output X
    }
    if (off == 0){
         ePWM_Regs[ch]->AQCSFRC.bit.CSFA = 0x0;
         ePWM_Regs[ch]->AQCSFRC.bit.CSFB = 0x0;
     }
}
void update_Sinduty(Uint16 ch, float32 m, float32 Sinduty )
{
    ePWM_Regs[ch]->CMPA.bit.CMPA = (Uint16)(( 0.5*ePWM_Regs[ch]->TBPRD) * (1.0 + m*Sinduty));

}
void TRIPZONE_CONFIG (Uint16 ch){
   //Trip Zone Config
   EALLOW; // IMPORTANT
       ePWM_Regs[ch]->TZSEL.bit.OSHT1 = 1;      //enables TZ1 as a one-shot event source for ePWM1
       ePWM_Regs[ch]->TZCTL.bit.TZA = TZ_FORCE_HI;        //EPWM1A will be forced low on a trip event
       ePWM_Regs[ch]->TZCTL.bit.TZB = TZ_FORCE_HI;        //DUng voi thuc te bo mach
       ePWM_Regs[ch]->TZEINT.bit.OST = 1;                 //EN interrupt
   EDIS;// IMPORTANT
}
