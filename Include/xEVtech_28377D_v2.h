/*
 * xEVtech_28377D_v2.h
 *
 *  Created on: Aug 11, 2022
 *      Author: Nguyen Van Minh Tam
 */

#ifndef INCLUDE_XEVTECH_28377D_V2_H_
#define INCLUDE_XEVTECH_28377D_V2_H_

#include "F28x_Project.h"

void Config_GPIOLED(void);
void Config_GPIOrelay(void);
void Config_ePWM_Buffer(void);
void setxEVtech_F28377D_v2(void);

#define LED_D10_ON          (GpioDataRegs.GPBSET.bit.GPIO41   = 1)
#define LED_D10_OFF         (GpioDataRegs.GPBCLEAR.bit.GPIO41   = 1)
#define LED_D10_TOGGLE      (GpioDataRegs.GPBTOGGLE.bit.GPIO41 = 1)

#define LED_D5_ON           (GpioDataRegs.GPBSET.bit.GPIO49   = 1)
#define LED_D5_OFF          (GpioDataRegs.GPBCLEAR.bit.GPIO49   = 1)
#define LED_D5_TOGGLE       (GpioDataRegs.GPBTOGGLE.bit.GPIO49 = 1)

#define LED_D9_ON           (GpioDataRegs.GPBSET.bit.GPIO50   = 1)
#define LED_D9_OFF          (GpioDataRegs.GPBCLEAR.bit.GPIO50   = 1)
#define LED_D9_TOGGLE       (GpioDataRegs.GPBTOGGLE.bit.GPIO50 = 1)

#define LED_D8_ON           (GpioDataRegs.GPBSET.bit.GPIO51   = 1)
#define LED_D8_OFF          (GpioDataRegs.GPBCLEAR.bit.GPIO51   = 1)
#define LED_D8_TOGGLE       (GpioDataRegs.GPBTOGGLE.bit.GPIO51 = 1)

#define LED_D4_ON           (GpioDataRegs.GPBSET.bit.GPIO52   = 1)
#define LED_D4_OFF          (GpioDataRegs.GPBCLEAR.bit.GPIO52   = 1)
#define LED_D4_TOGGLE       (GpioDataRegs.GPBTOGGLE.bit.GPIO52 = 1)

#define LED_D3_ON           (GpioDataRegs.GPBSET.bit.GPIO53   = 1)
#define LED_D3_OFF          (GpioDataRegs.GPBCLEAR.bit.GPIO53   = 1)
#define LED_D3_TOGGLE       (GpioDataRegs.GPBTOGGLE.bit.GPIO53 = 1)

//----------------------------------------
// Định nghĩa chân GPIO & Macro điều khiển Rơ-le (Kênh 24V)
// 1. Rơ-le K1: "Relay Ngõ Ra Biến Tần" -> GPIO 28
#define GPIO_RELAY_K1_INV    28
#define RELAY_K1_INV_ON()    (GpioDataRegs.GPASET.bit.GPIO28 = 1)
#define RELAY_K1_INV_OFF()   (GpioDataRegs.GPACLEAR.bit.GPIO28 = 1)

// 2. Rơ-le K6: "Relay PV / Ngõ Vào DC" -> GPIO 27
#define GPIO_RELAY_K6_PV     27
#define RELAY_K6_PV_ON()     (GpioDataRegs.GPASET.bit.GPIO27 = 1)
#define RELAY_K6_PV_OFF()    (GpioDataRegs.GPACLEAR.bit.GPIO27 = 1)

// 3. Rơ-le K3 và K4: "Relay Nối Lưới" (Grid Relay) -> GPIO 30
#define GPIO_RELAY_K3K4_GRID 30
#define RELAY_K3K4_GRID_ON()  (GpioDataRegs.GPASET.bit.GPIO30 = 1)
#define RELAY_K3K4_GRID_OFF() (GpioDataRegs.GPACLEAR.bit.GPIO30 = 1)

// Macro ngắn gọn để tương thích với mã nguồn
#define RELAY1_ON()          RELAY_K1_INV_ON()
#define RELAY1_OFF()         RELAY_K1_INV_OFF()
#define RELAY2_ON()          RELAY_K6_PV_ON()
#define RELAY2_OFF()         RELAY_K6_PV_OFF()
#define RELAY3_ON()          RELAY_K3K4_GRID_ON()
#define RELAY3_OFF()         RELAY_K3K4_GRID_OFF()

// Tương thích tên cũ phần cứng 24V
#define RELAY24V_J4_ON       RELAY_K1_INV_ON()
#define RELAY24V_J4_OFF      RELAY_K1_INV_OFF()
#define RELAY24V_J22_ON      RELAY_K3K4_GRID_ON()
#define RELAY24V_J22_OFF     RELAY_K3K4_GRID_OFF()
#define RELAY24V_J8_ON       RELAY_K6_PV_ON()
#define RELAY24V_J8_OFF      RELAY_K6_PV_OFF()

#define ePWM1_3_BUFF_ON        (GpioDataRegs.GPCSET.bit.GPIO91 = 1)
#define ePWM1_3_BUFF_OFF       (GpioDataRegs.GPCCLEAR.bit.GPIO91 = 1)

#define ePWM4_6_BUFF_ON        (GpioDataRegs.GPCSET.bit.GPIO92 = 1)
#define ePWM4_6_BUFF_OFF       (GpioDataRegs.GPCCLEAR.bit.GPIO92 = 1)

#define ePWM7_9_BUFF_ON        (GpioDataRegs.GPCSET.bit.GPIO93 = 1)
#define ePWM7_9_BUFF_OFF       (GpioDataRegs.GPCCLEAR.bit.GPIO93 = 1)

#define ePWM10_12_BUFF_ON        (GpioDataRegs.GPCSET.bit.GPIO94 = 1)
#define ePWM10_12_BUFF_OFF       (GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1)

#endif /* INCLUDE_XEVTECH_28377D_V2_H_ */
