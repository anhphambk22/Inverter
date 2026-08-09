/*
 * Config_ePWM.h
 *
 *  Created on: 2015. 1. 29.
 *      Author: Minho Kwon
 */
#include "F28x_Project.h"     // Device Headerfile and Examples Include File

#ifndef CONFIG_EPWM_H_
#define CONFIG_EPWM_H_

#if CPU_FRQ_200MHZ
	#define CPU_SPD              200E6
#endif

#if CPU_FRQ_150MHZ
	#define CPU_SPD              150E6
#endif

#if CPU_FRQ_120MHZ
	#define CPU_SPD              120E6
#endif

// Config_ePWM.c

extern void ePWM_MSync_SOCASEL(Uint16 ch, Uint16 SOCASEL, Uint16 delayCLK);
extern void ePWM_SSync_SOCASEL(Uint16 ch, Uint16 SOCASEL, Uint16 delayCLK);
extern void updateDuty(Uint16 ch, float32 duty);
extern void updateDutyBoost(Uint16 ch, float32 duty);
extern void updateDutyB(Uint16 ch, float32 duty);
extern void updatePhase(int16 ch, float32 phase);
extern void updateFreq(Uint16 ch, float32 freq);
extern void updateDeadtime(Uint16 ch, float32 dt_ns);
extern void forceOFFPWM(Uint16 ch, Uint16 off);
extern void forceOFFPWMxB(Uint16 ch, Uint16 off);
extern void conf_ePWM_Trip(Uint16 ch, Uint16 tripch, Uint16 interruptEnable);

extern void conf_ePWM_Sym(Uint16 ch, float32 freq);
extern void updateDuty_Sym(Uint16 ch, float32 duty);
//minhtam
void ePWM_SOCSEL(Uint16 ch,  Uint16 SOCASEL, Uint16 Number_envent);
void configure_deadband(Uint16 ch, Uint16 type);
void configure_deadband_boost(Uint16 ch, Uint16 type);
void configure_deadband_rec(Uint16 ch, Uint16 type);
void A3P2L_ePWM(Uint16 ch, float32 freq);  // count up-down
void A3P2L_ePWM_180(Uint16 ch, float32 freq);
void INIT_EPWM(Uint16 ch, float32 freq);
void A3P2L_ePWM_phaseship180(Uint16 ch, float32 freq);  // count up-down
void A3P2L_ePWM_rec(Uint16 ch, float32 freq);  // count up-down
void ForceOFFPWM(Uint16 ch, Uint16 off);
void update_Sinduty(Uint16 ch, float32 m, float32 Sinduty );
void TRIPZONE_CONFIG (Uint16 ch);
void updateDeadtimeR(Uint16 ch, float32 dt_ns);
void updateDeadtimeF(Uint16 ch, float32 dt_ns);
#endif /* CONFIG_EPWM_H_ */
