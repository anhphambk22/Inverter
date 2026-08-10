/*
 * Config_Tripzone.c
 *
 *  Created on: 2015. 11. 13.
 *      Author: Administrator
 */

#include "Config_Tripzone.h"
#include "Constant.h"

void conf_Tz_Qualification(Uint16 Qsel, float32 samplingWindow)
{
/*  Qsel :  0x0 = Synchronize to SYSCLKOUT
 *          0x1 = Qualification using 3 samples
 *          0x2 = Qualification using 6 samples
 *          0x3 = Asynchronous.
 */

    Uint16 qualPeriod;

    if(Qsel == 0x0)             qualPeriod = (Uint16)(samplingWindow * CPU_CLK / 2.0);
    else if(Qsel == 0x1)        qualPeriod = (Uint16)(samplingWindow * CPU_CLK / 4.0);
    else if(Qsel == 0x2)        qualPeriod = (Uint16)(samplingWindow * CPU_CLK / 10.0);
    else                        qualPeriod = 0x0;

    EALLOW;

    GpioCtrlRegs.GPBQSEL2.bit.GPIO61 = Qsel;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO62 = Qsel;
    GpioCtrlRegs.GPCQSEL1.bit.GPIO71 = Qsel;
    GpioCtrlRegs.GPCQSEL1.bit.GPIO73 = Qsel;

    GpioCtrlRegs.GPBCTRL.bit.QUALPRD3 = qualPeriod;     // GPIO56 - GPIO63
    GpioCtrlRegs.GPCCTRL.bit.QUALPRD0 = qualPeriod;     // GPIO64 - GPIO71
    GpioCtrlRegs.GPCCTRL.bit.QUALPRD1 = qualPeriod;     // GPIO72 - GPIO79

    EDIS;
}

/*
void resetTripCheck(void)
{
    Uint16 i;
    for(i=0;i<SIZE_CHECK_TRIP;i++)      checkTrip[i].all = 0x3F;

}*/


void InitTzGpio(void)
{
    // GPIO61 : VH, VB, Vlink, VDC
    // GPIO62 : IB
    // GPIO71 : IB
    // GPIO73 : TEMP

    EALLOW;

    GpioCtrlRegs.GPAPUD.bit.GPIO30 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO31 = 0;
    GpioCtrlRegs.GPBPUD.bit.GPIO32 = 0;
    GpioCtrlRegs.GPBPUD.bit.GPIO33 = 0;

    GpioCtrlRegs.GPAMUX2.bit.GPIO30 = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 0;
    GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 0;
    GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 0;

    GpioCtrlRegs.GPADIR.bit.GPIO30 = 0;  //
    GpioCtrlRegs.GPADIR.bit.GPIO31 = 0;  //
    GpioCtrlRegs.GPBDIR.bit.GPIO32 = 0;  //
    GpioCtrlRegs.GPBDIR.bit.GPIO33 = 0;  //

    InputXbarRegs.INPUT1SELECT = 30;    //
    InputXbarRegs.INPUT2SELECT = 31;    //
    InputXbarRegs.INPUT3SELECT = 32;    //
    InputXbarRegs.INPUT4SELECT = 33;    //


//    GpioCtrlRegs.GPBPUD.bit.GPIO61 = 0;  // Enable pull-up on GPIO31 (TZ1) FOR VH, VB, Vlink, VDC
//    GpioCtrlRegs.GPBPUD.bit.GPIO62 = 0;  // Enable pull-up on GPIO32 (TZ2) FOR TEMP
//    GpioCtrlRegs.GPCPUD.bit.GPIO71 = 0;  // Enable pull-up on GPIO33 (TZ3) FOR IB negative
//    GpioCtrlRegs.GPCPUD.bit.GPIO73 = 0;  // Enable pull-up on GPIO29 (TRIP4) FOR IB positive


//    GpioCtrlRegs.GPBMUX2.bit.GPIO61 = 0; // Configure GPIO32 as TZ1 FOR VH, VB, Vlink, VDC
//    GpioCtrlRegs.GPBMUX2.bit.GPIO62 = 0; // Configure GPIO33 as TZ2 FOR TEMP
//    GpioCtrlRegs.GPCMUX1.bit.GPIO71 = 0; // Configure GPIO35 as TZ3 FOR IB negative
//    GpioCtrlRegs.GPCMUX1.bit.GPIO73 = 0; // Configure GPIO29 as TRIP4 FOR IB positive


//    GpioCtrlRegs.GPBDIR.bit.GPIO61 = 0;  //
//    GpioCtrlRegs.GPBDIR.bit.GPIO62 = 0;  //
//    GpioCtrlRegs.GPCDIR.bit.GPIO71 = 0;  //
//    GpioCtrlRegs.GPCDIR.bit.GPIO73 = 0;  //


//    InputXbarRegs.INPUT1SELECT = 61;    // VH, VB, Vlink, VDC
//    InputXbarRegs.INPUT2SELECT = 62;    // TEMP
//    InputXbarRegs.INPUT3SELECT = 71;    // IB negative
//    InputXbarRegs.INPUT4SELECT = 73;    // IB positive

    EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX7 = 1;
    EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX7 = 1;
    EPwmXbarRegs.TRIPOUTINV.bit.TRIP4 = 1;

    EPwmXbarRegs.TRIPLOCK.bit.LOCK = 0;


    EDIS;
}






