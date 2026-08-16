#include "Config_ADC.h"
#include "Config_ePWM.h"
#include "DLOG_1CH_F.h" // os
#include "F28x_Project.h"
#include "SINEANALYZER_DIFF_F.h"
#include "SPLL_3PH_SRF_F.h"
#include "def.h"
#include "func.h"
#include "math.h"
#include "xEVtech_28377D_v2.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
enum MODE OPERATION_MODE = CALIB_ADC;
#define Fs                                                                     \
  25000 // ⚠️ KHÔNG DÙNG trong PR_init! Tần số lấy mẫu thực = FREQ_SAMPLING =
        // 10kHz (Constant.h)
#define GPIO_TEST1 63

// =========================================================================
// BIẾN ĐIỀU KHIỂN RELAY - Gán trực tiếp trên Watch Window CCS
// K1: Relay Ngõ Ra Biến Tần (GPIO28) | K6: Relay Ngõ Vào DC (GPIO27) | K3&K4:
// Relay Nối Lưới (GPIO30)
// =========================================================================
Uint16 relay_k1_inv =
    0; // 1 = BẬT K1 (Relay Ngõ Ra Biến Tần - GPIO28), Mặc định Mức 1
Uint16 relay_k6_pv =
    0; // 1 = BẬT K6 (Relay Ngõ Vào DC - GPIO27),      Mặc định Mức 1
Uint16 relay_k3k4_grid =
    1; // 1 = BẬT K3&K4 (Relay Nối Lưới - GPIO30),     Mặc định Mức 1
Uint16 auto_test_relay = 0; // 1 = Tự động xoay vòng | 0 = Thủ công
Uint32 cnt_relay_test = 0;

// Soft-start
float32 ramp_Vref_inv = 0.0f;
float32 ramp_ma_open = 0.0f; // Soft-start hệ số ma trong Vòng Hở
float32 ramp_ma_inv = 0.0f;  // Soft-start hệ số ma trong Vòng Kín Áp
float32 ramp_Iref = 0.0f;    // Soft-start dòng điện trong Vòng Kín Dòng

// Setpoint biên độ ĐỈNH Vòng Áp & Vòng Dòng
// ⚠️ Lưu ý: Vpeak_ref là biên độ ĐỈNH (Peak), KHÔNG phải RMS!
// Công thức: Vpeak_ref = Vrms_mong_muon × 1.4142
// Ví dụ: Muốn 36Vrms trên VOM → Đặt Vpeak_ref = 50.9V
float32 Vpeak_ref = 50.0f; // Setpoint biên độ ĐỈNH VAC ngõ ra (V peak)
float32 Vpeak_ref_rms =
    35.4f; // CHỈ ĐỂ QUAN SÁT: Vrms tương đương = Vpeak_ref / 1.4142
           // Cập nhật tay khi đổi Vpeak_ref (Vpeak_ref / 1.4142)
float32 Ipeak_ref = 2.0f; // Setpoint biên độ đỉnh Dòng điện ngõ ra (A peak) - Mặc định 0.0A để đồng bộ nối lưới an toàn

float32 deadrise = 500.0, deadfall = 100.0;
float32 Fs_LLC_OpenLoop = 105000;
Uint16 dac_sw = 15, startpwm, CTR1;
Uint16 adcVlink, adcIL, adcVgrid, adcIL2, adcIpd;
float32 offsetstart = 0.0, VgridA, Vlink, IL1, IL2, iL, iL2, iLpd, VlinkADC;
Uint16 adcVi, adcVbatt, adcIlink;
float32 VbattADC;
float32 Ilink, Vbatt; // LLC Part
float32 Vlinkref = 150, Vlinkref_max = 150, Vbatt_ref = 140, MaxVLink = 350.0f,
        Vlinkref_in, Vbatt_ref_offset = 0, ramp_Vlink, MaxVbatt = 500,
        Vbattt = 0;
float32 MaxVac = 350.0f; // Ngưỡng bảo vệ quá áp AC ngõ ra (350V đỉnh để không ngắt giả khi thử lưới)
float32 ma_open = 0.5; // Hệ số điều chế Vòng Hở (Thay đổi từ 0.0 đến 0.95)
float32 Vgm = 50, VgridADC, Vgpeak;
Uint16 adcVgrid2 = 0;
float32 Vgrid2ADC = 0.0f, Vgrid2 = 0.0f, offset_Comp_vGrid2 = 0.0f;
struct Z_1ST zVGA2;
// declare virtual oscilloscope variable:
Uint16 datagraph = 3;
DLOG_1CH_F dlog_1ch1;
float32 DBUFF_1CH1[100];
float32 dval1, fcutVlink = 5000.0, stopBand_2nd = 10;
// end
SPLL_3ph_SRF_F srf_pll;
struct ADAPTIVE_SOGI_PLL my_adaptive_pll;
Uint16 dac_Vgrid = 0;
float32 duty_recA = 0;
float32 duty_recB = 0;
float32 cal_duty = 0.0;
Uint16 ResetFault = 0;
float32 dutyA = 0, dutyB = 0;
float32 phase = 0, max = 6.26, min = 0.02;
float32 cnt_Calib = 0, Picon_theta = 0.0, theta = 0.0, ENCLOSEDLOOP_LLC = 0,
        f_cl = 0.5, f_th = 4, step_f = 10;
float32 Fs_LLC;
float32 dutypwmtest = 0, freqpwmtest = 0, coef_check = 0, offset_test = 0;
int ENPFC = 0, ENPD = 0;
interrupt void adcA_isr(void);
//====
struct COEF_1ST coef1stLPF, coef1stLPF_REF, coef1stLPF_LC, coef1stLPF_VLink, coef1stLPF_iL;
struct COEF_1ST coef_APF;
struct COEF_2ND coef2ndBSF_Vlink, coef2ndBSF_Vbatt, coef2nd_BPF_W5th,
    coef2nd_BPF_W7th;
struct COEF_2ND coef2nd_BPF_W11th, coef2nd_BPF_W13th;
struct Z_2ND z2_Ig5_BPF_A, z2_Ig5_BPF_B, z2_Ig5_BPF_C, z2_Ig7_BPfsF_A,
    z2_Ig7_BPF_B, z2_Ig7_BPF_C;
struct Z_2ND z2_Ig11_BPF_A, z2_Ig11_BPF_B, z2_Ig11_BPF_C, z2_Ig13_BPF_A,
    z2_Ig13_BPF_B, z2_Ig13_BPF_C;
struct Z_2ND z2_Vlink_BSF, z2_Vbatt_BSF;
struct Z_1ST zVDCp, zVDCn, zVGA, zVGB, zVGC, zIGA, zIGB, zIGC, zVGd, zVGq, zIGd,
    zIGq, zMaff, zREF;
struct Z_1ST zVbatt;
struct COEF_1ST coef_LF;
//----
struct THREE_PHASE_VALUE rVGabc, rIGabc, ma_abc, VGabc, IGabc;
struct PID_GAINS GPIPLL;
struct PID_ERRORS EPIPLL;
struct COEF_2ND coef_Vlink_LLC;
struct COEF_1ST coef_Io_LLC;

// Phase-Locked Loop variables and define:
struct THREE_PHASE_VALUE singen;
struct STATIONARY Vg_albe, Vg_albe1, er_VLink_albe;
struct SYNCHRONOUS VgDQ, er_VLink_dq2, er_VLink_dq4, er_VLink_dq6;
struct COEF_1ST coeff_LF;
struct COEF_2ND ddsogi1, ddsogi2;
struct Z_2ND zsogi1, zsogi2;
struct SRF srf_pll1;

union CHECK_FUALT { //
  Uint16 all;
  struct CHECK_FUALT_BIT {
    Uint16 hwtrip : 1;

    Uint16 overTEMP : 1;

    Uint16 swtrip : 1;
    Uint16 overVLink : 1;
    Uint16 overVbatt : 1;
    Uint16 overIL1 : 1;
    Uint16 overIL2 : 1;
    Uint16 overIlink : 1;
    Uint16 overILpd : 1;
    Uint16 overFsLLC : 1;
    Uint16 rsed : 2;
  } bit;
};

union CHECK_FUALT checkFault;

#define LF_KP 0.444
#define LF_KI 25.3331
float32 refsin, refsin4, refsin2, refsin6;
float32 refcos, refcos4, refcos2, refcos6;
Uint16 PLL_Flag = 1;
float32 theta_gen, theta_out;
Uint16 INPUT = 1;
float sign = 1;
float32 PIconout1, Duty_pd, Duty_pd_sum, PIconout2;
float32 Vlinksen, er_VLink;
float32 phase1 = 0, phase2 = 180;
struct PID_GAINS kVi, kIL, kVpd, kVlink, kVbatt, kLLC_ss;
struct PID_ERRORS eVbatt, eVlink, eIL1, eIL2, eLLC_ss;
struct Z_1ST Vlink_LLC1, Vlink_LLC2, io_LLC;
struct COEF_2ND coeff_Vlink_LLC;

float32 IFLref_LIMIT = 15.0f; // Ngưỡng bảo vệ quá dòng ngõ ra AC (15A đỉnh để không ngắt giả)
float32 Ilinkref_LIMIT = 30;
float32 Ilinkss_LIMIT = 20;
uint32_t index = 0;

float32 plot1[512], plot2[512];

float32 Piconout = 0.0, Piconout2 = 0.0, PI_V_out = 0.0, Fs_ref = 0,
        PiconoutLLC = 0.0;
float32 err;

Uint16 enable_FF = 0;

PR_Controller pr, pr_3rd, pr_5th, pr_I;
float32 pr_3rd_out1 = 0.0, pr_5th_out1 = 0.0;
// === DEBUG: Biến riêng để xem trên Watch Window (CCS không đọc được struct
// member) ===
volatile float32 debug_pr_b0 = 0.0f; // Phải ≈ 4.005 nếu PR_init đã chạy
volatile float32 debug_pr_a1 = 0.0f; // Phải ≈ 0.2
volatile float32 debug_Piconout =
    0.0f; // Output PR (dao động ±50V nếu hoạt động đúng)
volatile float32 debug_err_inv =
    0.0f; // Sai số (dao động quanh 0 nếu bám setpoint)
// === Biến trạng thái cho PR INLINE (thay thế struct PR_Controller) ===
volatile float32 pr_e0 = 0, pr_e1 = 0, pr_e2 = 0; // error history (vòng áp)
volatile float32 pr_y1 = 0, pr_y2 = 0; // resonant output history (vòng áp)
volatile float32 pr_I_e0 = 0, pr_I_e1 = 0,
                 pr_I_e2 = 0; // error history (vòng dòng)
volatile float32 pr_I_y1 = 0,
                 pr_I_y2 = 0; // resonant output history (vòng dòng)
float32 Kp = 0.1f; // Hệ số Kp cho Vòng Áp Inverter (Tăng lên 0.1f cho thử
                   // nghiệm bám setpoint chính xác)
float32 Kr = 50;
float32 Kp_I = 0.05f; // Hệ số Kp cho Vòng Kín Dòng Điện (Băng thông vòng kín ~1kHz)
float32 Kr_I = 100.0f; // Hệ số Kr cho Vòng Kín Dòng Điện (Độ lợi gain >60dB tại 50Hz)
float32 omegac_I = 5.0f;
float32 sign_iL = -1.0f; // 1.0 = Thuận cực tính (Phản hồi âm) | -1.0 = Đảo cực
                         // tính cảm biến dòng iL
float32 sign_Vgrid =
    1.0f; // 1.0 = Thuận cực tính | -1.0 = Đảo cực tính cảm biến áp VgridA
volatile float32 iL_offset =
    110.0f; // Offset ADC count khi iL=0A (trừ ra trước khi quy đổi)
volatile Uint16 is_grid_tied = 0;  // 0 = Độc lập | 1 = Nối lưới (SOGI-PLL, theta_out)
volatile Uint16 is_pll_locked = 0; // 1 = PLL đã khóa pha thành công (49-51Hz)
volatile Uint16 is_grid_sync_ready = 0; // 1 = ĐỦ 3 ĐIỀU KIỆN ĐỒNG BỘ HÒA LƯỚI (ΔV < 5V, Δf < 0.5Hz, Δθ < 5°)
volatile float32 delta_V = 0.0f;       // Chênh lệch điện áp hiệu dụng giữa Inverter và Lưới (V)
volatile float32 delta_f = 0.0f;       // Chênh lệch tần số so với 50Hz (Hz)
volatile float32 delta_theta_deg = 0.0f; // Chênh lệch góc pha tức thời (Độ)
float32 compensator_3rd_flag = 0, compensator_5th_flag = 0;
float32 error1 = 0.0, omegac = 10.0f, Kr_3rd = 20, omegac_3rd = 5.0f,
        Kr_5th = 10, omegac_5th = 5.0f;
// omegac=10 rad/s → băng thông ~1.6Hz quanh 50Hz, đủ rộng để bám setpoint
volatile float32 Vlink_offset = 560.0f; // Zero offset ADC khi Vdc = 0V
volatile float32 Vlink_gain =
    0.0890f; // Hệ số V/count (Đã Calib chuẩn đét tại 120.0VDC thực tế)
volatile float32 Vgrid_gain_calib =
    0.91f; // Hệ số Calib chuẩn cho cảm biến áp AC Inverter 1
volatile float32 Vgrid2_gain_calib =
    0.776f; // Hệ số Calib chuẩn cho cảm biến áp AC Lưới 2 (Calib tuyến tính 6 điểm: 17V -> 90V)
volatile float32 total_delay_sec =
    0.000493f; // Tổng thời gian trễ phần cứng + phần mềm (493us tương ứng 8.875 độ ở 50Hz) cố định cho SOGI-PLL
volatile float32 delay_I_sec =
    0.0f; // Biến bù pha DÒNG ĐIỆN ĐỘC LẬP (Nhập âm trên Watch Window khi dòng bị sớm pha, KHÔNG ảnh hưởng PLL)
volatile float32 phase_shift_deg =
    8.875f; // Góc bù trễ pha tự động hiển thị trên Watch Window (độ)
volatile float32 Vgrid_sync_gain =
    1.00f; // Hệ số bù điện áp Feedforward hòa lưới (Chuẩn 1.00 để không vọt dòng khi Iref=0)
volatile float32
    iL_gain_calib =
        5.00f; // Hệ số Calib cảm biến dòng AC Inverter (Tăng từ 3.5 lên 5.0 để iL_rms đọc chuẩn trùng VOM)
volatile float32 R_load =
    50.0f; // Điện trở tải thử nghiệm (Ohm) cho bù Feedforward độc lập
volatile float32 PR_AW_limit =
    95.0f; // Giới hạn Anti-windup PR (V). Chỉnh trên Watch Window.
volatile float32 Vgrid_rms =
    0.0f; // Giá trị hiệu dụng RMS điện áp AC ngõ ra 1 (Vrms)
volatile float32 Vgrid2_rms =
    0.0f; // Giá trị hiệu dụng RMS điện áp AC lưới 2 (Vrms)
volatile float32 iL_rms =
    0.0f; // Giá trị hiệu dụng RMS dòng điện AC ngõ ra (Irms)
volatile float32 Vgrid_max = 0.0f;   // Điện áp đỉnh MAX đo được của Inverter 1 (V peak)
volatile float32 Vgrid2_max = 0.0f;  // Điện áp đỉnh MAX đo được của Lưới 2 (V peak)
volatile float32 iL_max = 0.0f;      // Dòng điện đỉnh MAX đo được qua cuộn lọc (A peak)
volatile Uint16 reset_max_peaks = 0; // Gán = 1 trên Watch Window để Reset các giá trị MAX về 0
static float32 Vgrid_sq_sum = 0.0f;  // Bộ đệm tính True-RMS áp 1
static float32 Vgrid2_sq_sum = 0.0f; // Bộ đệm tính True-RMS áp 2 (Lưới)
static float32 iL_sq_sum = 0.0f;     // Bộ đệm tính True-RMS dòng
// end

//============
// end of PLL's variable declaration

// Tự định nghĩa hàm cấu hình chân UART (SCI-B)
void InitScibGpio(void) {
  EALLOW;
  // Cấu hình SCI-B trên chân 54 (TX) và 55 (RX)
  // Trên F2837xD, MUX = 6 tương ứng với SCITXDB (GPIO54) và SCIRXDB (GPIO55)
  GPIO_SetupPinMux(55, GPIO_MUX_CPU1, 6); // SCIRXDB
  GPIO_SetupPinOptions(55, GPIO_INPUT, GPIO_ASYNC);

  GPIO_SetupPinMux(54, GPIO_MUX_CPU1, 6); // SCITXDB
  GPIO_SetupPinOptions(54, GPIO_OUTPUT, GPIO_PUSHPULL);
  EDIS;
}

// Các hàm phụ trợ gửi dữ liệu qua UART nhẹ nhàng, tránh dùng sprintf gây tràn
// bộ nhớ
void scib_xmit(char a) {
  while (ScibRegs.SCICTL2.bit.TXRDY == 0) {
  }
  ScibRegs.SCITXBUF.all = a;
}
void scib_msg(char *msg) {
  int k = 0;
  while (msg[k] != '\0') {
    scib_xmit(msg[k]);
    k++;
  }
}
void scib_send_int(int val) {
  char buf[10];
  int k = 0;
  if (val == 0) {
    scib_xmit('0');
    return;
  }
  if (val < 0) {
    scib_xmit('-');
    val = -val;
  }
  while (val > 0) {
    buf[k++] = (val % 10) + '0';
    val /= 10;
  }
  while (k > 0) {
    k--;
    scib_xmit(buf[k]);
  }
}
void scib_send_float_x10(float val) {
  int val_int = (int)(val * 10.0f);
  if (val_int < 0) {
    scib_xmit('-');
    val_int = -val_int;
  }
  int whole = val_int / 10;
  int frac = val_int % 10;
  scib_send_int(whole);
  scib_xmit('.');
  scib_send_int(frac);
}

 void main(void) {
  //

  DINT;
  // Step 1. Initialize System Control:
  // PLL, WatchDog, enable Peripheral Clocks
  // This example function is found in the F2837xD_SysCtrl.c file.
  //
  InitSysCtrl();

  EALLOW;
  // Bật Clock cho các ngoại vi
  CpuSysRegs.PCLKCR2.bit.EPWM1 = 1;
  CpuSysRegs.PCLKCR7.bit.SCI_B = 1; // <--- Cấp Clock cho SCI-B
  EDIS;
  DINT;
  //
  // Initialize the PIE control registers to their default state.
  // The default state is all PIE interrupts disabled and flags
  // are cleared.
  // This function is found in the F2837xD_PieCtrl.c file.
  //
  InitPieCtrl();
  IER = 0x0000;
  IFR = 0x0000;

  InitPieVectTable();
  // Vector Remapping
  EALLOW;
  PieVectTable.ADCA1_INT = &adcA_isr;
  EDIS;
  //============================================================================================

  // enable PIE interrupt
  PieCtrlRegs.PIEIER1.bit.INTx1 = 1; // ADCA  // PIE    ͷ Ʈ(ADCA1INT) Ȱ  ȭ
  IER |= M_INT1;                     // Enable group 1 interrupts
  // Step 2. Initialize GPIO:
  InitGpio();
  InitScibGpio(); // Khởi tạo chân GPIO cho SCI-B (UART) để giao tiếp C#
  setxEVtech_F28377D_v2();
  InitEPwmGpio(); // 12EPWM - GPIO
  GPIO_SetupPinMux(GPIO_TEST1, GPIO_MUX_CPU1, 0);
  GPIO_SetupPinOptions(GPIO_TEST1, GPIO_OUTPUT, GPIO_PUSHPULL);
  ePWM_SOCSEL(1, ET_CTR_ZERO,
              2); // Trigger ADC mỗi 2 lần counter=0 (FREQ_SW=20kHz → ISR=10kHz)

  // Chỉ sử dụng ePWM1 và ePWM2 cho Inverter (Full-Bridge)
  // Cấu hình cùng pha sóng mang (sóng mang tam giác chung) để chạy điều chế đơn
  // cực (Unipolar SPWM)
  A3P2L_ePWM(1, FREQ_SW); // count up-down (Leg A)
  A3P2L_ePWM(2, FREQ_SW); // count up-down (Leg B) - Cùng pha với ePWM1

  updateDuty(1, 0.0);
  updateDuty(2, 0.0);

  //===deadtime (Bảo vệ phần cứng: Ép 2 xung A và B phải nghịch đảo nhau)
  configure_deadband(1, DB_ACTV_HIC); // Nhánh A (ePWM 1A và 1B ngược nhau)
  configure_deadband(2, DB_ACTV_HIC); // Nhánh B (ePWM 2A và 2B ngược nhau)

  updateDeadtime(1, 500.0);
  updateDeadtime(2, 500.0);
  //===
  EALLOW;
  CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;
  EDIS;

  conf_ADC_A();
  conf_ADC_C();
  Config_DAC_PINB1();

  // --- GHI ĐÈ KÊNH ADC VẬT LÝ (Map lại chân SOC) ---
  EALLOW;
  // 1. Vlink (DC-Link) -> ADC A1 (SOC0 của ADC-A)
  AdcaRegs.ADCSOC0CTL.bit.CHSEL = 1;
  AdcaRegs.ADCSOC0CTL.bit.ACQPS = 14;
  AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5;

  // 2. Vgrid (Đo lưới 1) -> ADC A5 (SOC1 của ADC-A)
  AdcaRegs.ADCSOC1CTL.bit.CHSEL = 5;
  AdcaRegs.ADCSOC1CTL.bit.ACQPS = 14;
  AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 5;

  // 3. iL (Dòng điện cảm) -> ADC C3 (SOC0 của ADC-C)
  AdccRegs.ADCSOC0CTL.bit.CHSEL = 3;
  AdccRegs.ADCSOC0CTL.bit.ACQPS = 14;
  AdccRegs.ADCSOC0CTL.bit.TRIGSEL = 5;

  // 4. Vgrid2 (Đo lưới 2) -> ADC C2 (SOC1 của ADC-C)
  AdccRegs.ADCSOC1CTL.bit.CHSEL = 2;
  AdccRegs.ADCSOC1CTL.bit.ACQPS = 14;
  AdccRegs.ADCSOC1CTL.bit.TRIGSEL = 5;

  // 5. SỬA NGẮT: conf_ADC_A() đặt INT1SEL=2 (chờ SOC2 xong mới ngắt).
  //    SOC2 không dùng nữa -> đặt lại INT1SEL=1 để ngắt ngay sau SOC1 (Vgrid1).
  //    Tránh trễ ~1.5µs mỗi chu kỳ và giữ đồng bộ lấy mẫu với PWM đúng thời
  //    điểm.
  AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 1; // Kích ngắt sau EOC của SOC1
  AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   // Bật ADCINT1
  AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; // Xóa flag cũ
  EDIS;
  DELAY_US(1000);
  // ----------------------------------------------

  EALLOW;
  CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
  EDIS;

  // OFF pwm first
  ePWM1_3_BUFF_OFF;

  ForceOFFPWM(1, 1);
  ForceOFFPWM(2, 1);
  // Totempole
  // Voltage loop
  kVlink.P = 0.03;
  kVlink.I = 0.5;
  kVlink.a = 1.0 / kVlink.P; // antiwindup

  // Current loop
  kIL.P = 0.03;        // 0.003 loopcurrent
  kIL.I = 50;          // 60 loop current
  kIL.a = 1.0 / kIL.P; // antiwindup

  // LLC converter
  // Soft start loop
  kLLC_ss.P = 0.0006;
  kLLC_ss.I = 0.0001;
  kLLC_ss.a = 1 / kLLC_ss.P;

  // Voltage loop
  kVbatt.P = 1.2;
  kVbatt.I = 0.000000001;
  kVbatt.a = 1.0 / kVbatt.P;

  // kVpd.a = 1.0/kVpd.P;

  // === KHỞI TẠO BỘ ĐIỀU KHIỂN PR & SOGI-PLL (BẮT BUỘC) ===
  Init_Adaptive_SOGI_PLL(&my_adaptive_pll, SAMPLING_TIME);
  // PR cơ bản 50Hz cho vòng áp
  PR_init(&pr, Kp, Kr, SAMPLING_TIME, 2.0 * PI * 50.0, omegac);
  // PR bù hài bậc 3 (150Hz)
  PR_init(&pr_3rd, 0.0f, Kr_3rd, SAMPLING_TIME, 2.0 * PI * 150.0, omegac_3rd);
  // PR bù hài bậc 5 (250Hz)
  PR_init(&pr_5th, 0.0f, Kr_5th, SAMPLING_TIME, 2.0 * PI * 250.0, omegac_5th);
  // PR cho vòng dòng điện 50Hz
  PR_init(&pr_I, Kp_I, Kr_I, SAMPLING_TIME, 2.0 * PI * 50.0, omegac_I);
  // Gán debug để xác nhận PR_init đã chạy
  debug_pr_b0 = pr.b0;
  debug_pr_a1 = pr.a1;
  // =================================================================

  EINT; // Enable Global interrupt INTM
  ERTM; // Enable Global realtime interrupt DBGM

  // initializing virtual oscilloscope on CCS's graph variables:
  DLOG_1CH_F_init(&dlog_1ch1);
  dlog_1ch1.input_ptr = &dval1; // data value
  dlog_1ch1.output_ptr = &DBUFF_1CH1[0];
  dlog_1ch1.size = 100;
  dlog_1ch1.pre_scalar = 5;
  dlog_1ch1.trig_value = 0.01;
  dlog_1ch1.status = 2;
  //===== end of initialization

  //===== Cấu hình giao tiếp UART (SCI-B) cho phần mềm C# =====
  ScibRegs.SCICCR.all = 0x0007;  // 1 stop bit, No parity, 8 char bits
  ScibRegs.SCICTL1.all = 0x0003; // Enable TX, RX
  // Tốc độ 115200 Baud (Tính theo LSPCLK mặc định 50MHz của C2000)
  ScibRegs.SCIHBAUD.all = 0x0000;
  ScibRegs.SCILBAUD.all = 53;
  ScibRegs.SCICTL1.all = 0x0023; // Khởi động SCI
  //=============================================================

  reset_PIDError(&EPIPLL);
  // calculate PLL Loop filter parameters based on PI controller
  coeff_LF = getcoeff_LF(LF_KP, LF_KI, SAMPLING_TIME); // coeff of PLL
  coef1stLPF = get1stLPFcoef(5000.0, FREQ_SAMPLING);   // coeff for filter Vgrid
  coef1stLPF_iL = get1stLPFcoef(1000.0, FREQ_SAMPLING); // Coeff cho LPF Dòng iL (Băng thông 1000Hz lọc sạch nhiễu xung PWM 20kHz)
  coef1stLPF_VLink =
      get1stLPFcoef(fcutVlink, FREQ_SAMPLING); // coeff for filter VLink
  coef2ndBSF_Vlink =
      get2ndBSFcoef(GRID_FREQ * 2, 1, stopBand_2nd, FREQ_SAMPLING);
  coef2ndBSF_Vbatt =
      get2ndBSFcoef(GRID_FREQ * 2, 1, stopBand_2nd, FREQ_SAMPLING);
  // Low pass filter
  zVGA.in = 0.0;
  zVGA.out = 0.0;
  zVGB.in = 0.0;
  zVGB.out = 0.0;
  zVGC.in = 0.0;
  zVGC.out = 0.0;
  zVbatt.in = 0.0;
  zVbatt.out = 0.0;
  zIGA.in = 0.0;
  zIGA.out = 0.0;
  // Band stop filter
  z2_Vlink_BSF.in = 0.0;
  z2_Vlink_BSF.out = 0.0;
  z2_Vlink_BSF.Zn1.in = 0.0;
  z2_Vlink_BSF.Zn1.out = 0.0;
  //         zsogi1
  //         zsogi1
  Init_Adaptive_SOGI_PLL(&my_adaptive_pll, SAMPLING_TIME);

  // Initialize PR controllers once
  PR_init(&pr, Kp, Kr, SAMPLING_TIME, GRID_FREQ, omegac);
  PR_init(&pr_3rd, 0, Kr_3rd, SAMPLING_TIME, 3 * GRID_FREQ, omegac_3rd);
  PR_init(&pr_5th, 0, Kr_5th, SAMPLING_TIME, 5 * GRID_FREQ, omegac_5th);
  PR_init(&pr_I, Kp_I, Kr_I, SAMPLING_TIME, GRID_FREQ, omegac_I);

  long sci_loop_cnt = 0;

  while (1) {
    // Nhận lệnh từ giao diện C# (Không chặn chương trình)
    if (ScibRegs.SCIRXST.bit.RXRDY == 1) {
      Uint16 command = ScibRegs.SCIRXBUF.all; // Đọc ký tự C# gửi xuống
      if (command == '2') {
        OPERATION_MODE = SYSTEM_STANDBY;
      } else if (command == '7') {
        OPERATION_MODE = INVERTER_RUN;
      } else if (command == '8') {
        OPERATION_MODE = PWM_TEST;
      } else if (command == '9') {
        OPERATION_MODE = OPEN_LOOP;
      }
    }

    // Định kỳ gửi giá trị cảm biến lên C# (Khoảng 10 lần / giây)
    sci_loop_cnt++;
    if (sci_loop_cnt >= 200000) {
      sci_loop_cnt = 0;

      // Đóng gói và gửi thủ công để né hàm sprintf khổng lồ
      scib_msg("Vdc:");
      scib_send_float_x10((float)Vlink);
      scib_msg(",Vac:");
      scib_send_float_x10((float)my_adaptive_pll.V_mag);
      scib_msg("\n");
    }
    //===============

    switch (dac_sw) {
    case 0:
      DaccRegs.DACVALS.all = adcVgrid;
      break;
    case 1:
      DaccRegs.DACVALS.all = dac_Vgrid;
      break;
    case 2:
      DaccRegs.DACVALS.all = (Uint16)4095.0 * theta_out / 6.28;
      break;
    case 3:
      DaccRegs.DACVALS.all = 4095.0 * (sin(theta_out) + 1.0) / 2.0;
      break;
    case 4:
      DaccRegs.DACVALS.all = (singen.A + 1) * 4095.0 / 2.0;
      break;
    case 5:
      DaccRegs.DACVALS.all =
          (Uint16)(my_adaptive_pll.omega_est / 314.159 * 2048.0);
      break;
    case 6:
      DaccRegs.DACVALS.all = 4095.0 * (cos(theta_out) + 1.0) / 2.0;
      break;
    case 7:
      DaccRegs.DACVALS.all = (Uint16)4095.0 * cal_duty;
      break;
    case 8:
      DaccRegs.DACVALS.all = (Uint16)(4095.0 * (dutyA + 1.0f) / 2.0f);
      break;
    case 9:
      DaccRegs.DACVALS.all = (Uint16)4095.0 * Piconout + 2048;
      break;
    case 10:
      DaccRegs.DACVALS.all = (Uint16)4095.0 * Vgpeak;
      break;
    case 11:
      DaccRegs.DACVALS.all = (Uint16)4095.0 * Piconout2 + 2048;
      break;
    case 12:
      DaccRegs.DACVALS.all = (Uint16)4095.0 * ENPFC;
      break;
    case 13:
      DaccRegs.DACVALS.all = (Uint16)4095.0 * adcIlink;
      break;
    case 14:
      DaccRegs.DACVALS.all = (Uint16)adcVlink;
      break;
    case 15:
      DaccRegs.DACVALS.all =
          (Uint16)4095.0 * phase2 / 180; // watch start up mode
      break;
    case 16:
      DaccRegs.DACVALS.all = (Uint16)4095.0 * Fs_LLC / 280000.0; // watch fs
      break;
    case 17:
      DaccRegs.DACVALS.all = (Uint16)(my_adaptive_pll.k_adaptive * 1000.0);
      break;
    case 18:
      DaccRegs.DACVALS.all = 2048; // Test DAC (Xuat muc giua 50% = 1.65V)
      break;
    case 19:
      DaccRegs.DACVALS.all = 4095; // Test DAC (Xuat muc cao nhat 100% = 3.3V)
      break;
    case 20:
      DaccRegs.DACVALS.all = 0; // Test DAC (Xuat muc thap nhat 0% = 0V)
      break;
    case 21:
      // Xuất VgridA (điện áp lưới đọc được) ra DAC để quan sát Oscilloscope
      // Scale: VgridA ±200V → DAC 0~4095, trung tâm 2048 = 1.65V = 0V
      DaccRegs.DACVALS.all =
          (Uint16)(4095.0f * (VgridA / 200.0f + 1.0f) / 2.0f);
      break;
    case 22: {
      // Xuất iL đã lọc LPF mượt ra DAC (Thang ±5A)
      static float32 iL_dac_f22 = 0.0f;
      iL_dac_f22 = 0.85f * iL_dac_f22 + 0.15f * iL;
      float32 val_dac = (4095.0f * (iL_dac_f22 / 5.0f + 1.0f) / 2.0f);
      if (val_dac > 4095.0f) val_dac = 4095.0f;
      if (val_dac < 0.0f) val_dac = 0.0f;
      DaccRegs.DACVALS.all = (Uint16)val_dac;
      break;
    }
    case 23: {
      // Xuất iL đã lọc LPF mượt ra DAC (Thang PHÓNG ĐẠI ±2.5A cho dòng nhỏ)
      static float32 iL_dac_f23 = 0.0f;
      iL_dac_f23 = 0.85f * iL_dac_f23 + 0.15f * iL;
      float32 val_dac = (4095.0f * (iL_dac_f23 / 2.5f + 1.0f) / 2.0f);
      if (val_dac > 4095.0f) val_dac = 4095.0f;
      if (val_dac < 0.0f) val_dac = 0.0f;
      DaccRegs.DACVALS.all = (Uint16)val_dac;
      break;
    }
    }
  }
}
//=======================
float32 duty, duty2, T;
float32 dutyff;
float32 duty_LLC;
Uint16 led_cnt = 0;
float32 oset = 2.49, oset2 = 2.48, osetVg = 1.648;
float32 offset_Comp_vGrid, offset_Comp_vDCLink, offset_Comp_iL,
    offset_Comp_vbatt, offset_Comp_ilink;
float32 ILref;
float32 ILrefAC = 0;
float32 pre_error_VgPLL = 0;
float32 error_VgPLL = 0;
Uint16 dectection_zero = 0;
//==============================
interrupt void adcA_isr(void) {
  GPIO_WritePin(GPIO_TEST1, 1);
  theta_gen = ramp_cycle_gen(theta_gen, 50.0, FREQ_SAMPLING);
  //=======
  singen.A = sinf(2 * PI * theta_gen);
  singen.B = sinf(2 * PI * theta_gen - sign * 2 * PI / 3);
  singen.C = sinf(2 * PI * theta_gen + sign * 2 * PI / 3);
  // theta_gen is variable which is created by the ram_cycle_gen function:
  // Input: theta_gen is the theta_gen variable at the time ADCA Interrupt
  // Service Routine 50.0 is the frequency of theta_gen. 25000.0 is the sampling
  // time/ interrupt time ADC - INVERTER ĐÃ ĐƯỢC MAP LẠI CHÂN
  adcVlink = AdcaResultRegs.ADCRESULT0; // Đọc Vlink từ SOC0 của ADC-A (ADC_A1)
  adcVgrid = AdcaResultRegs.ADCRESULT1; // Đọc Vgrid1 từ SOC1 của ADC-A (ADC_A5)
  adcIL = AdccResultRegs.ADCRESULT0;    // Đọc iL từ SOC0 của ADC-C (ADC_C3)
  adcVgrid2 =
      AdccResultRegs.ADCRESULT1; // Đọc Vgrid2 từ SOC1 của ADC-C (ADC_C2)

  float32 iL_raw = scaled_sensor_CURR(adcIL, 0.0, 1.65, offset_Comp_iL, 3.27, 1) *
                   iL_gain_calib;
  iL = firstLPF(iL_raw, &zIGA, coef1stLPF_iL);

  // Vgrid scale (ADC_A5)
  VgridADC = scaled_sensor_VOLT(adcVgrid, 0, 1.9, offset_Comp_vGrid, 3.9, 3.3,
                                7 * 173, 1);
  VgridA = firstLPF(VgridADC, &zVGA, coef1stLPF) * Vgrid_gain_calib;

  // Vgrid2 scale (ADC_C2 - Cảm biến đo lưới 2)
  Vgrid2ADC = scaled_sensor_VOLT(adcVgrid2, 0.0, 1.57, offset_Comp_vGrid2, 3.77,
                                 3.0, 7 * 437, 1.2);
  Vgrid2 = firstLPF(Vgrid2ADC, &zVGA2, coef1stLPF) * Vgrid2_gain_calib;

  // Vlink scale: Trừ zero offset (560) và nhân hệ số V/count (0.0866)
  float adcVlink_diff = (float)adcVlink - Vlink_offset;
  if (adcVlink_diff < 0.0f)
    adcVlink_diff = 0.0f;
  VlinkADC = adcVlink_diff * Vlink_gain;

  Vlink = firstLPF(VlinkADC, &zVGB, coef1stLPF_VLink);

  // --- TÍNH GIÁ TRỊ HIỆU DỤNG TRUE-RMS TRỰC TIẾP CHO ÁP VÀ DÒNG AC ---
  Vgrid_sq_sum = 0.998f * Vgrid_sq_sum + 0.002f * (VgridA * VgridA);
  Vgrid_rms = sqrtf(Vgrid_sq_sum);

  Vgrid2_sq_sum = 0.998f * Vgrid2_sq_sum + 0.002f * (Vgrid2 * Vgrid2);
  Vgrid2_rms = sqrtf(Vgrid2_sq_sum);

  iL_sq_sum = 0.998f * iL_sq_sum + 0.002f * (iL * iL);
  iL_rms = sqrtf(iL_sq_sum);

  // --- THEO DÕI GIÁ TRỊ ĐỈNH LỚN NHẤT (MAX PEAK TRACKING) ---
  if (reset_max_peaks || OPERATION_MODE == CALIB_ADC) {
    Vgrid_max = 0.0f;
    Vgrid2_max = 0.0f;
    iL_max = 0.0f;
    reset_max_peaks = 0;
  } else if (OPERATION_MODE != SYSTEM_STANDBY) {
    float32 abs_v1 = fabsf(VgridA);
    float32 abs_v2 = fabsf(Vgrid2);
    float32 abs_il = fabsf(iL);
    if (abs_v1 > Vgrid_max)  Vgrid_max  = abs_v1;
    if (abs_v2 > Vgrid2_max) Vgrid2_max = abs_v2;
    if (abs_il > iL_max)     iL_max     = abs_il;
  }

  //===================================================================================================================
  //                        ADC - LLC RESONANT PART
  adcIlink = AdcaResultRegs.ADCRESULT4; // Uni
  adcVbatt = AdcaResultRegs.ADCRESULT6; // Bi
  /// Ilink scale
  Ilink = scaled_sensor_CURR(adcIlink, 2.5, 0, offset_Comp_ilink, 3.3, 2.2);
  // Vbatt scale
  // VbattADC = scaled_sensor_VOLT(adcVbatt, 0.0, 1.65,
  // offset_Comp_vbatt, 3.3, 2.7, 7*510, 1.2);
  VbattADC = scaled_sensor_VOLT(adcVbatt, 0.0, 0.0, offset_Comp_vbatt, 3.3, 6.8,
                                7 * 510, 1.2);
  if (coef_check == 2)
    Vbatt = secondBSF(VbattADC, &z2_Vbatt_BSF, coef2ndBSF_Vbatt);
  else if (Vbatt >= 50)
    Vbatt = firstLPF(VbattADC, &zVbatt, coef1stLPF_VLink) * 1.0676 + 0.5127;
  else
    Vbatt = firstLPF(VbattADC, &zVbatt, coef1stLPF_VLink);
  //===================================================================================================================
  //                              ADAPTIVE SOGI-PLL (Khóa pha theo Vgrid2 -
  //                              ADC_C2)
  my_adaptive_pll.v_s = (fabsf(Vgrid2) > 10.0f) ? Vgrid2 : VgridA;
  Adaptive_SOGI_PLL_Func(&my_adaptive_pll);

  // TỰ ĐỘNG BÙ TRỄ PHA THEO TẦN SỐ LƯỚI REAL-TIME (Giải pháp A: Δθ_auto = ω_est * total_delay_sec)
  float32 auto_phase_comp = my_adaptive_pll.omega_est * total_delay_sec;
  phase_shift_deg = auto_phase_comp * (180.0f / 3.1415926535f); // Cập nhật độ lệch pha hiển thị (độ)
  theta_out = my_adaptive_pll.theta_out + auto_phase_comp;
  if (theta_out >= 6.283185307f) theta_out -= 6.283185307f;
  else if (theta_out < 0.0f) theta_out += 6.283185307f;

  refsin = sin(theta_out);
  refcos = cos(theta_out);

  Vgpeak = my_adaptive_pll.V_mag;
  float32 pll_f0 = my_adaptive_pll.omega_est / (2.0f * PI);
  is_pll_locked = (pll_f0 >= 49.0f && pll_f0 <= 51.0f && Vgpeak > 20.0f) ? 1 : 0;

  // --- TÍNH TOÁN 3 ĐIỀU KIỆN ĐỒNG BỘ HÒA LƯỚI (ΔV, Δf, Δθ) ---
  // 1. Chênh lệch Điện áp RMS (V)
  delta_V = fabsf(Vgrid_rms - Vgrid2_rms);

  // 2. Chênh lệch Tần số (Hz)
  delta_f = fabsf(pll_f0 - 50.0f);

  // 3. Chênh lệch Góc pha tức thời (Độ)
  float32 v_q = my_adaptive_pll.v_alpha * cosf(my_adaptive_pll.theta) + my_adaptive_pll.v_beta * sinf(my_adaptive_pll.theta);
  float32 v_q_norm = (Vgpeak > 5.0f) ? fabsf(v_q / Vgpeak) : 1.0f;
  if (v_q_norm > 1.0f) v_q_norm = 1.0f;
  delta_theta_deg = asinf(v_q_norm) * (180.0f / 3.1415926535f);

  // 4. Cờ tự động xác nhận ĐỦ ĐIỀU KIỆN ĐỒNG BỘ HÒA LƯỚI (CÓ HYSTERESIS & DEBOUNCE 200ms CHỐNG RUNG RELAY)
  static Uint16 sync_debounce_cnt = 0;
  Uint16 cond_sync_now = (delta_V < 8.0f && delta_f < 0.8f && delta_theta_deg < 10.0f && Vgrid2_rms > 10.0f);
  if (cond_sync_now) {
    if (sync_debounce_cnt < 2000) { // Yêu cầu bám liên tục 200ms (2000 mẫu ngắt) mới bật cờ
      sync_debounce_cnt++;
    } else {
      is_grid_sync_ready = 1;
    }
  } else {
    if (sync_debounce_cnt > 0) {
      sync_debounce_cnt--;
    } else {
      is_grid_sync_ready = 0;
    }
  }
  // detect zero
  duty_recA = (theta_out >= 0.03) && (theta_out <= 3.11);
  duty_recB = (theta_out >= 3.18) && (theta_out <= 6.25);

  if ((theta_out >= 0.0) && (theta_out <= 3.14159)) {
    cal_duty = (Vlinkref - Vgpeak) / Vlinkref;
  } else {
    cal_duty = (-Vgpeak) / (Vlinkref);
  }
  // --- BẢO VỆ BẰNG CẢM BIẾN (SOFTWARE PROTECTION) ---
  // Chỉ bật bảo vệ sau khi đã Calib xong offset cảm biến (khác CALIB_ADC)
  if (OPERATION_MODE != CALIB_ADC) {
    // 1. Bảo vệ Quá Áp DC Link (> MaxVLink = 350V)
    if (fabs(Vlink) > MaxVLink) {
      OPERATION_MODE = FAULT;
      checkFault.bit.swtrip = 1;
      checkFault.bit.overVLink = 1;
    }

    // 2. Bảo vệ Quá Áp AC ngõ ra (> MaxVac = 70V Đỉnh ~ 49.5V rms)
    if (fabs(VgridA) > MaxVac) {
      OPERATION_MODE = FAULT;
      checkFault.bit.swtrip = 1;
    }

    // 3. Bảo vệ Quá Dòng AC ngõ ra (> IFLref_LIMIT = 5A)
    if (fabs(iL) > IFLref_LIMIT) {
      OPERATION_MODE = FAULT;
      checkFault.bit.swtrip = 1;
      checkFault.bit.overIL1 = 1;
    }
  }
  // --------------------------------------------------

  // =========================================================================
  // VÒNG LẶP ĐIỀU KHIỂN INVERTER 1 PHA (FULL-BRIDGE UNIPOLAR SPWM)
  // =========================================================================
  switch (OPERATION_MODE) {
  case CALIB_ADC:
    cnt_Calib++;
    offset_Comp_iL = get_offset(offset_Comp_iL, iL);
    offset_Comp_vGrid = get_offset(offset_Comp_vGrid, VgridA);
    offset_Comp_vGrid2 = get_offset(offset_Comp_vGrid2, Vgrid2);
    offset_Comp_vDCLink = get_offset(offset_Comp_vDCLink, Vlink);
    if (cnt_Calib >= PRD_7SEC) {
      cnt_Calib = 0;
      OPERATION_MODE = SYSTEM_STANDBY;
    }
    break;

  case SYSTEM_STANDBY:
    // Reset soft-start ramp cho tất cả các chế độ
    ramp_Vref_inv = 0.0f;
    ramp_ma_open = 0.0f;
    ramp_ma_inv = 0.0f;
    ramp_Iref = 0.0f;
    // Tắt toàn bộ xung PWM
    updateDuty(1, 0.0);
    updateDuty(2, 0.0);
    ForceOFFPWM(1, 1);
    ForceOFFPWM(2, 1);
    ePWM1_3_BUFF_OFF;

    // Reset bộ đệm PR inline (áp và dòng)
    pr_e0 = 0;
    pr_e1 = 0;
    pr_e2 = 0;
    pr_y1 = 0;
    pr_y2 = 0;
    pr_I_e0 = 0;
    pr_I_e1 = 0;
    pr_I_e2 = 0;
    pr_I_y1 = 0;
    pr_I_y2 = 0;
    ramp_Vref_inv = 0.0f;
    ramp_Iref = 0.0f;

    // Ghi chú: Gán OPERATION_MODE = INVERTER_RUN (7) trên Watch Window
    break;

  case INVERTER_RUN: {
    // =======================================================
    // BỘ ĐIỀU KHIỂN PR ĐƠN GIẢN NHẤT — INLINE, KHÔNG GỌI HÀM
    // =======================================================
    ePWM1_3_BUFF_ON;
    ForceOFFPWM(1, 0);
    ForceOFFPWM(2, 0);

    // 1. Soft-start ramp (10V/s — tăng và giảm linh hoạt theo Vpeak_ref)
    if (ramp_Vref_inv < Vpeak_ref) {
      ramp_Vref_inv += 10.0f * SAMPLING_TIME;
      if (ramp_Vref_inv > Vpeak_ref)
        ramp_Vref_inv = Vpeak_ref;
    } else if (ramp_Vref_inv > Vpeak_ref) {
      ramp_Vref_inv -= 10.0f * SAMPLING_TIME;
      if (ramp_Vref_inv < Vpeak_ref)
        ramp_Vref_inv = Vpeak_ref;
    }

    // 2. Tạo tham chiếu sin 50Hz
    float Vref_inv = ramp_Vref_inv * sinf(2.0f * PI * theta_gen);

    // 3. Lấy Vdc (tránh chia 0)
    float Vdc_base = (Vlink > 20.0f) ? Vlink : 100.0f;

    // 4. Feedforward
    float duty_ff = Vref_inv / Vdc_base;

    // 5. Tính sai số (VgridA đọc ÂM khi Vout DƯƠNG → cộng để tạo phản hồi âm
    // đúng)
    float err = Vref_inv + VgridA;

    // 6. PR INLINE — Hệ số Tustin (T=0.0001, f0=50Hz, ωc=10, Kr=50)
    //    b0=4.005, b1=-7.998, b2=3.997, a1=a2=0.2
    pr_e2 = pr_e1;
    pr_e1 = pr_e0;
    pr_e0 = err;

    float resonant =
        (0.2f * pr_e0 - 0.2f * pr_e2 + 7.998f * pr_y1 - 3.997f * pr_y2) /
        4.005f;

    // Anti-windup
    if (resonant > PR_AW_limit)
      resonant = PR_AW_limit;
    if (resonant < -PR_AW_limit)
      resonant = -PR_AW_limit;

    pr_y2 = pr_y1;
    pr_y1 = resonant;

    // Kp * err + resonant
    Piconout = Kp * err + resonant;

    // 7. Tổng duty
    float duty_pr = Piconout / Vdc_base;
    float control_signal = duty_ff + duty_pr;

    // 8. Giới hạn duty [-0.95, 0.95] cho phép đẩy áp AC ngõ ra tối đa
    dutyA = Limiter(control_signal, 0.95f, -0.95f);

    // Debug
    debug_Piconout = Piconout;
    debug_err_inv = err;
    debug_pr_b0 = 4.005f; // Xác nhận inline PR đang chạy

    // 9. Unipolar SPWM (Đã đảo cực tính để đồng pha 100% với điện áp lưới)
    float duty_LegA = (-dutyA + 1.0f) / 2.0f;
    float duty_LegB = (dutyA + 1.0f) / 2.0f;

    updateDuty(2, duty_LegA);
    updateDuty(1, duty_LegB);
    break;
  }

  case CURRENT_RUN: {
    // =========================================================================
    // MODE 14: VÒNG LẶP ĐIỀU KHIỂN DÒNG ĐIỆN ĐỘC LẬP TRÊN TẢI TRỞ (OFF-GRID CURRENT CONTROL)
    // Sử dụng dao động 50Hz nội sinh (theta_gen) và Feedforward bù theo điện trở R_load
    // =========================================================================
    is_grid_tied = 0;
    ePWM1_3_BUFF_ON;
    ForceOFFPWM(1, 0);
    ForceOFFPWM(2, 0);

    // 1. Soft-start dòng điện Iref (Tăng/giảm 0.1A mỗi chu kỳ ngắt)
    if (ramp_Iref < Ipeak_ref) {
      ramp_Iref += 0.1f * SAMPLING_TIME;
      if (ramp_Iref > Ipeak_ref)
        ramp_Iref = Ipeak_ref;
    } else if (ramp_Iref > Ipeak_ref) {
      ramp_Iref -= 0.1f * SAMPLING_TIME;
      if (ramp_Iref < Ipeak_ref)
        ramp_Iref = Ipeak_ref;
    }

    // 2. Tham chiếu sin 50Hz nội sinh (theta_gen)
    float sin_theta = sinf(2.0f * PI * theta_gen);
    float Iref_inv = ramp_Iref * sin_theta;

    // 3. Điện áp DC Link làm điện áp cơ sở
    float Vdc_base = (Vlink > 20.0f) ? Vlink : 100.0f;

    // 4. Feedforward bù tải trở (Iref * R_load / Vdc)
    float duty_ff = (Iref_inv * R_load) / Vdc_base;

    // 5. Sai số dòng điện cảm biến
    float err_I = Iref_inv - (sign_iL * iL);

    // 6. PR INLINE DÒNG ĐIỆN — Hệ số Tustin (T=0.0001, f0=50Hz, ωc=5, Kr=30)
    pr_I_e2 = pr_I_e1;
    pr_I_e1 = pr_I_e0;
    pr_I_e0 = err_I;

    float resonant_I = (0.060f * pr_I_e0 - 0.060f * pr_I_e2 + 7.998f * pr_I_y1 -
                        3.998f * pr_I_y2) /
                       4.002f;

    if (resonant_I > 50.0f)
      resonant_I = 50.0f;
    if (resonant_I < -50.0f)
      resonant_I = -50.0f;

    pr_I_y2 = pr_I_y1;
    pr_I_y1 = resonant_I;

    float Piconout_I = Kp_I * err_I + resonant_I;

    // 7. Tổng tín hiệu điều khiển = Feedforward + PR Feedback
    float duty_pr = Piconout_I / Vdc_base;
    float control_signal = duty_ff + duty_pr;

    // 8. Giới hạn duty cycle an toàn [-0.8, 0.8]
    dutyA = Limiter(control_signal, 0.8f, -0.8f);

    // 9. Unipolar SPWM (Đã đảo cực tính để đồng pha 100% với điện áp lưới)
    float duty_LegA = (-dutyA + 1.0f) / 2.0f;
    float duty_LegB = (dutyA + 1.0f) / 2.0f;

    updateDuty(2, duty_LegA);
    updateDuty(1, duty_LegB);
    break;
  }

  case GRID_CONNECTED_RUN: {
    // =========================================================================
    // MODE 15: VÒNG LẶP ĐIỀU KHIỂN DÒNG NỐI LƯỚI CHUYÊN DỤNG (GRID-TIED INVERTER MODE)
    // Khóa pha theo SOGI-PLL (theta_out) và Feedforward bù theo điện áp lưới real-time (VgridA / Vdc)
    // =========================================================================
    is_grid_tied = 1;
    ePWM1_3_BUFF_ON;
    ForceOFFPWM(1, 0);
    ForceOFFPWM(2, 0);

    // 1. Soft-start dòng điện Iref (Tăng/giảm mượt mờ 0.5A / giây)
    if (ramp_Iref < Ipeak_ref) {
      ramp_Iref += 5000.0f * SAMPLING_TIME;
      if (ramp_Iref > Ipeak_ref)
        ramp_Iref = Ipeak_ref;
    } else if (ramp_Iref > Ipeak_ref) {
      ramp_Iref -= 5000.0f * SAMPLING_TIME;
      if (ramp_Iref < Ipeak_ref)
        ramp_Iref = Ipeak_ref;
    }

    // 2. Tham chiếu sin lấy theo pha SOGI-PLL đã khóa lưới, có bù góc pha DÒNG ĐIỆN ĐỘC LẬP (delay_I_sec)
    float sin_theta = sinf(theta_out);
    float theta_I = theta_out + (my_adaptive_pll.omega_est * delay_I_sec);
    float sin_theta_I = sinf(theta_I);
    float Iref_inv = ramp_Iref * sin_theta_I;

    // 3. Điện áp DC Link cơ sở
    float Vdc_base = (Vlink > 20.0f) ? Vlink : 100.0f;

    // 4. Feedforward bù điện áp Lưới từ SOGI-PLL đã bù góc trễ total_delay_sec (Vgpeak * sin_theta)
    float duty_ff = (Vgpeak * sin_theta * Vgrid_sync_gain) / Vdc_base;

    // 5. Sai số dòng điện cảm biến
    float err_I = Iref_inv - (sign_iL * iL);

    // 6. PR INLINE DÒNG ĐIỆN — Hệ số Tustin chuẩn hóa (Kr_I = 100.0, ωc = 5.0 rad/s)
    pr_I_e2 = pr_I_e1;
    pr_I_e1 = pr_I_e0;
    pr_I_e0 = err_I;

    float resonant_I = (0.200f * pr_I_e0 - 0.200f * pr_I_e2 + 7.9980f * pr_I_y1 -
                        3.9990f * pr_I_y2) /
                       4.0030f;

    if (resonant_I > 50.0f)
      resonant_I = 50.0f;
    if (resonant_I < -50.0f)
      resonant_I = -50.0f;

    pr_I_y2 = pr_I_y1;
    pr_I_y1 = resonant_I;

    float Piconout_I = Kp_I * err_I + resonant_I;

    // 7. Tổng tín hiệu điều khiển = Feedforward Lưới + PR Feedback
    float duty_pr = Piconout_I / Vdc_base;
    float control_signal = duty_ff + duty_pr;

    // 8. Giới hạn duty cycle an toàn [-0.95, 0.95] cho phép bơm dòng tối đa
    dutyA = Limiter(control_signal, 0.95f, -0.95f);

    // 9. Unipolar SPWM (Đã đảo cực tính để đồng pha 100% với điện áp lưới)
    float duty_LegA = (-dutyA + 1.0f) / 2.0f;
    float duty_LegB = (dutyA + 1.0f) / 2.0f;

    updateDuty(2, duty_LegA);
    updateDuty(1, duty_LegB);
    break;
  }

  case OPEN_LOOP: {
    ePWM1_3_BUFF_ON;
    ForceOFFPWM(1, 0);
    ForceOFFPWM(2, 0);

    if (ma_open < 0.0f || ma_open > 0.95f) {
      ma_open = 0.5f;
    }

    // Khởi động mềm (Soft-start) hệ số ma_open với tốc độ 0.1 / giây (5 giây từ
    // 0 -> 0.5)
    if (ramp_ma_open < ma_open) {
      ramp_ma_open += 0.1f * SAMPLING_TIME;
      if (ramp_ma_open > ma_open)
        ramp_ma_open = ma_open;
    } else if (ramp_ma_open > ma_open) {
      ramp_ma_open -= 0.1f * SAMPLING_TIME;
      if (ramp_ma_open < ma_open)
        ramp_ma_open = ma_open;
    }

    float control_signal = ramp_ma_open * sinf(2.0f * PI * theta_gen);
    dutyA = control_signal;

    float duty_LegA = (-dutyA + 1.0f) / 2.0f;
    float duty_LegB = (dutyA + 1.0f) / 2.0f;

    updateDuty(2, duty_LegA);
    updateDuty(1, duty_LegB);
    break;
  }

  case PWM_TEST:
    ePWM1_3_BUFF_ON;
    ForceOFFPWM(1, 0);
    ForceOFFPWM(2, 0);
    updateDuty(1, 0.5f);
    updateDuty(2, 0.5f);
    break;

  case FAULT:
    ForceOFFPWM(1, 1);
    ForceOFFPWM(2, 1);
    ePWM1_3_BUFF_OFF;
    // Đưa toàn bộ Relay về lại giá trị cài đặt ban đầu khi bị FAULT
    relay_k1_inv = 0;
    relay_k6_pv = 0;
    relay_k3k4_grid = 1; // 1 = Ngắt K3&K4 (Relay Nối Lưới) về giá trị ban đầu
    if (ResetFault)
      goto ResetMode;
    break;

  case RESET:
  ResetMode:
    ResetFault = 0;
    checkFault.all = 0;
    error1 = 0;
    pr.prev_error[0][0] = 0;
    pr.prev_error[0][1] = 0;
    pr.prev_output[0][0] = 0;
    pr.prev_output[0][1] = 0;
    OPERATION_MODE = SYSTEM_STANDBY;
    break;

  default:
    ForceOFFPWM(1, 1);
    ForceOFFPWM(2, 1);
    ePWM1_3_BUFF_OFF;
    break;
  }

  // =========================================================================
  // ĐIỀU KHIỂN RELAY TOÀN CỤC - Hoạt động ở MỌI chế độ (mode)
  // Gán relay_k1_inv / relay_k6_pv / relay_k3k4_grid = 1 (BẬT) hoặc 0 (TẮT)
  // =========================================================================
  if (relay_k1_inv)
    RELAY_K1_INV_ON();
  else
    RELAY_K1_INV_OFF(); // K1: Relay Ngõ Ra Biến Tần  (GPIO28)
  if (relay_k6_pv)
    RELAY_K6_PV_ON();
  else
    RELAY_K6_PV_OFF(); // K6: Relay Ngõ Vào DC       (GPIO27)
  if (relay_k3k4_grid)
    RELAY_K3K4_GRID_ON();
  else
    RELAY_K3K4_GRID_OFF(); // K3&K4: Relay Nối Lưới     (GPIO30)

  GPIO_WritePin(GPIO_TEST1, 0);
  // ADC routine
  AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; // clear INT1 flag
  PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
