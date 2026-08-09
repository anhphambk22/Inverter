/*
 * xEVtech_28377D_v2.c
 *
 *  Created on: Aug 11, 2022
 *      Author: Nguyen Van Minh Tam
 */
#include "F28x_Project.h"     // Device Headerfile and Examples Include File
#include "xEVtech_28377D_v2.h"

void setxEVtech_F28377D_v2(void)
{
    Config_GPIOLED();
    Config_GPIOrelay();
    Config_ePWM_Buffer();
}
void Config_GPIOLED(void)
{
  EALLOW;
     GpioCtrlRegs.GPBMUX1.bit.GPIO41 = 0;
     GpioCtrlRegs.GPBMUX2.bit.GPIO49 = 0;
     GpioCtrlRegs.GPBMUX2.bit.GPIO50 = 0;
     GpioCtrlRegs.GPBMUX2.bit.GPIO51 = 0;
     GpioCtrlRegs.GPBMUX2.bit.GPIO52 = 0;
     GpioCtrlRegs.GPBMUX2.bit.GPIO53 = 0;
             //LED 41 - 53//
     //----END OF MUX-----//
     //
     GpioCtrlRegs.GPBDIR.bit.GPIO41 = 1;
     GpioCtrlRegs.GPBDIR.bit.GPIO49 = 1;
     GpioCtrlRegs.GPBDIR.bit.GPIO50 = 1;
     GpioCtrlRegs.GPBDIR.bit.GPIO51 = 1;
     GpioCtrlRegs.GPBDIR.bit.GPIO52 = 1;
     GpioCtrlRegs.GPBDIR.bit.GPIO53 = 1;
             //LED 41 - 53//

     //----END OF DIR-----//
   EDIS;
}
void Config_GPIOrelay(void)
{
    EALLOW;
       GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 0;
       GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 0;
       GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 0;
       GpioCtrlRegs.GPAMUX2.bit.GPIO27 = 0;
       GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 0;
       GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 0;
       GpioCtrlRegs.GPAMUX2.bit.GPIO30 = 0;


       //----END OF MUX-----//
       //
       GpioCtrlRegs.GPADIR.bit.GPIO24 = 1;
       GpioCtrlRegs.GPADIR.bit.GPIO25 = 1;
       GpioCtrlRegs.GPADIR.bit.GPIO26 = 1;
       GpioCtrlRegs.GPADIR.bit.GPIO27 = 1;
       GpioCtrlRegs.GPADIR.bit.GPIO28 = 1;
       GpioCtrlRegs.GPADIR.bit.GPIO29 = 1;
       GpioCtrlRegs.GPADIR.bit.GPIO30 = 1;
               //LED RELAY 24 - 30/

       //----END OF DIR-----//
     EDIS;
    }
void Config_ePWM_Buffer(void)
{
    EALLOW;
       GpioCtrlRegs.GPCMUX2.bit.GPIO91 = 0;
       GpioCtrlRegs.GPCMUX2.bit.GPIO92 = 0;
       GpioCtrlRegs.GPCMUX2.bit.GPIO93 = 0;
       GpioCtrlRegs.GPCMUX2.bit.GPIO94 = 0;
       //----END OF MUX-----//
       GpioCtrlRegs.GPCDIR.bit.GPIO91 = 1;
       GpioCtrlRegs.GPCDIR.bit.GPIO92 = 1;
       GpioCtrlRegs.GPCDIR.bit.GPIO93 = 1;
       GpioCtrlRegs.GPCDIR.bit.GPIO94 = 1;
               //LED RELAY 24 - 30/

       //----END OF DIR-----//
     EDIS;
}

