/*
 * func.h
 *
 *  Created on: 2015. 1. 29.
 *      Author: Minho Kwon
 *      edited by NGUYEN VAN MINH TAM /// 22.08.22
 */

#ifndef FUNC_H_
#define FUNC_H_

#include "F28x_Project.h"
#include "Constant.h"
#include "Type.h"
//===============

//====================
extern float32 PIcon(float32 ref, float32 in, struct PID_ERRORS *error, struct PID_GAINS K);
extern float32 PIcon_transient(float32 ref, float32 in, struct PID_ERRORS *error, struct PID_GAINS K);
extern float32 PIDcon(float32 ref, float32 in, struct PID_ERRORS *error, struct PID_GAINS K);
extern void antiwindup(float32 PIout, float32 Limitout, struct PID_ERRORS *error, struct PID_GAINS K);
extern float32 Limiter(float32 in, float32 upper, float32 lower);
extern float32 scaleADC(Uint16 in, float32 offset, float32 scale);
extern struct COEF_1ST get1stLPFcoef(float32 cutoff, float32 freqSampling);
//extern float32 firstLPF(float32 in, float32 *past_in, float32 past_out, struct LPF_COEF coef);
extern float32 firstLPF(float32 in, struct Z_1ST *Zn1, struct COEF_1ST coef);
extern float32 getScaleGain(float32 sensorGain, float32 ampGain);
extern float32 zTF_2nd(float32 in, struct Z_2ND *Zn2, struct COEF_2ND coef);
extern void resetError(struct PID_ERRORS *error);

extern struct STATIONARY ClarkTrans(struct THREE_PHASE_VALUE in);
extern struct THREE_PHASE_VALUE invClarkTrans(struct STATIONARY in);
extern struct SYNCHRONOUS ParkTrans(struct STATIONARY in, float32 sin, float32 cos);
extern struct STATIONARY invParkTrans(struct SYNCHRONOUS in, float32 sin, float32 cos);

extern struct SYNCHRONOUS abc2dq(struct THREE_PHASE_VALUE in, struct STATIONARY *stat, float32 refsin, float32 refcos);
extern struct THREE_PHASE_VALUE dq2abc(struct SYNCHRONOUS in, struct STATIONARY *stat, float32 refsin, float32 refcos);
//minhtam
float32 duty_ramp(float32 duty, float32 duty_cmd, float32 ISR);
float32 ramp_cycle_gen(float32 in, float32 Freq, float32 ISR );
float32 scaled_sensor(Uint16 ADC_Result, float32 offset);
float32 scaled_sensor_dc(Uint16 ADC_Result, float32 offset);
float scaled_sensor_CURR(unsigned int ADC_Result, float offset_IC, float offset_ref, float offset_comp,float R1, float R2);
float scaled_sensor_VOLT(unsigned int ADC_Result, float offset_IC, float offset_ref, float offset_comp, float R1, float R2, float Rsen1, float Rsen2);
float get_offset(float in, float signal);
float32 V_adc(float32 adc_value);
float32 out_sensor(float32 vadc, float32 offset);
struct STATIONARY abc2albe(struct THREE_PHASE_VALUE in);
void reset_PIDError(struct PID_ERRORS *error);
struct SYNCHRONOUS albe2dq(struct STATIONARY in, float32 refsin, float32 refcos);
struct COEF_1ST getcoeff_LF(float32 Kp, float32 Ki, float32 Ts);
void PLL_INIT(float32 grid_freq, float32 sampling_time,struct COEF_1ST lf_para,struct SRF *PLL);  //initialization
void SRF_PLL_FUNC(struct SRF *pll);
void INIT_COEF_2ND(struct COEF_2ND *in);
void update_coef_2nd( float32 sampling_time, float32 res_freq,struct COEF_2ND *in);
extern struct COEF_2ND get2ndBSFcoef(float32 freqCenter, float32 gain, float32 stopBand, float32 freqSampling);
void INIT_DDSOGI(struct PD in);
float32 scaled_sensor_current(Uint16 ADC_Result, float32 offset);

struct STATIONARY Vgrid1phase_to_albe(float32 pre_error_VgPLL,float32 error_VgPLL,float32 Vgrid1phase, float32 sampl_time, float32 K,struct STATIONARY albe2);
Uint16 scaled_sensor_rever(float32 sensor, float32 offset);
float updateRef(float ref, float cmd, float delta);
extern float PR_calculate(PR_Controller *pr, float error, int index);
extern void PR_init(PR_Controller *pr, float Kp, float Kr, float T, float omega0, float omegac);
extern float32 secondBSF(float32 in, struct Z_2ND*Zn2, struct COEF_2ND coef3);

// --- ADAPTIVE OBSERVER SOGI-PLL ---
struct ADAPTIVE_SOGI_PLL {
    float32 v_s;
    float32 Kp;
    float32 Ki;
    float32 k_dc;
    float32 gamma;
    float32 lambda;
    float32 Ts;

    float32 v_alpha;
    float32 v_beta;
    float32 v_dc;
    float32 pi_integral;
    float32 theta;
    float32 k_adaptive;

    float32 omega_est;
    float32 theta_out;
    float32 V_mag;
};

void Init_Adaptive_SOGI_PLL(struct ADAPTIVE_SOGI_PLL *pll, float32 Ts);
void Adaptive_SOGI_PLL_Func(struct ADAPTIVE_SOGI_PLL *pll);

#endif /* FUNC_H_ */
