#include <math.h>
/*   Func.c  */


#include "func.h"


float32 PIcon(float32 ref, float32 in, struct PID_ERRORS *error, struct PID_GAINS K)
{
	float32 out;

	error->e = ref - in;

	error->sum += error->e ;
	out = (K.P * error->e) + error->sum* K.I * SAMPLING_TIME;

	return out;
}

float32 PIcon_Bili(float32 ref, float32 in, struct Z_1ST *Zn1, struct COEF_1ST coef)
{
    float32 error, out;

    error = ref - in;
    out = coef.b0 * error + coef.b1 * Zn1->in + (-coef.a1 * Zn1->out);

    Zn1->in = error;
    Zn1->out = out;

    return out;
}

struct COEF_1ST getcoeff_PI(struct PID_GAINS K, float32 Ts)
{
     struct COEF_1ST coef;

     coef.b0= (2.0 * K.P+ K.I * Ts) / 2.0;
     coef.b1= -(2.0 * K.P-K.I * Ts) / 2.0;
     coef.a1= -1;
     coef.a0= 1;

     return coef;
}

float32 PIDcon(float32 ref, float32 in, struct PID_ERRORS *error, struct PID_GAINS K)
{
	float32 out, errorDelta;

	error->e = ref - in;

	error->sum += (error->e - error->w) * K.I * SAMPLING_TIME;
	errorDelta = (error->e - error->past) / SAMPLING_TIME;
	error->past = error->e;

	out = (K.P * error->e) + error->sum + (K.D * errorDelta);

	return out;
}


void antiwindup(float32 PIout, float32 Limiterout, struct PID_ERRORS *error, struct PID_GAINS K)
{
	float32 windupError;

	windupError = PIout - Limiterout;
	error->w = K.a * windupError;
}

float32 Limiter(float32 in, float32 upper, float32 lower)
{
	if (in > upper)				return upper;
	else if (in < lower)		return lower;
	else						return in;
}


float32 scaleADC(Uint16 in, float32 offset, float32 scale)
{
	return ((float32)in-offset) * scale;
}

float32 getScaleGain(float32 sensorGain, float32 ampGain)
{
	float32 analGain, digiGain;   // ampGain = R2/R1 (Opamp gain)

	analGain = sensorGain * ampGain;
	digiGain = 3.3 / ( 4096.0 * analGain);		// 3.0/(4096.0*analGain)

	return	digiGain;

}

struct COEF_1ST get1stLPFcoef(float32 cutoff, float32 freqSampling)
{
	float32 w_cut;
	struct COEF_1ST coef;
	w_cut = TWOPI * cutoff;

	coef.a1 = ( w_cut - 2.0f * freqSampling)/( 2.0f * freqSampling + w_cut);
	coef.b0 = w_cut / ( 2.0f * freqSampling + w_cut);
	coef.b1 = coef.b0;

	coef.a0 = 0;

	return coef;
}

float32 firstLPF(float32 in, struct Z_1ST *Zn1, struct COEF_1ST coef)
{
	float32 out;

	out = (-coef.a1 * Zn1->out) + (coef.b0 * (in + Zn1->in));
	Zn1->in = in;
	Zn1->out = out;

	return out;				
}

float32 zTF_2nd(float32 in, struct Z_2ND *Zn2, struct COEF_2ND coef){

    float32 out;

	out = -coef.a1 * Zn2->Zn1.out
		+ -coef.a2 * Zn2->out
		+  coef.b0 * in
		+  coef.b1 * Zn2->Zn1.in
		+  coef.b2 * Zn2->in;
	
	Zn2->in = Zn2->Zn1.in;
	Zn2->Zn1.in = in;

	Zn2->out = Zn2->Zn1.out;
	Zn2->Zn1.out = out;

	return out;				
}

float32 secondBSF(float32 in, struct Z_2ND*Zn2, struct COEF_2ND coef3)
{
    float32 out;
    out =   coef3.b0*in
          + coef3.b1*Zn2->Zn1.in
          + coef3.b2*Zn2->in
          - coef3.a1*Zn2->Zn1.out
          - coef3.a2*Zn2->out;

    Zn2->in = Zn2->Zn1.in;
    Zn2->Zn1.in = in;
    Zn2->out = Zn2->Zn1.out;
    Zn2->Zn1.out = out;

    return out;
}

// Band stop coef
struct COEF_2ND get2ndBSFcoef(float32 freqCenter, float32 gain, float32 stopBand, float32 freqSampling)
{
    float32 B, w_0, Ts, denum;
    struct COEF_2ND coef3;

    w_0 = TWOPI * freqCenter;
    B = TWOPI * stopBand;
    Ts = 1 / freqSampling;

    denum = 1 + B * Ts / 2 + w_0 * w_0 * Ts * Ts / 4;

    coef3.b0 = (gain + gain * (w_0 * w_0 * Ts * Ts / 4)) / denum;
    coef3.b1 = (-2 * gain + gain * (w_0 * w_0 * Ts * Ts / 2)) / denum;
    coef3.b2 = (gain + gain * (w_0 * w_0 * Ts * Ts / 4)) / denum;

    coef3.a0 = 1;
    coef3.a1 = (-2 + w_0 * w_0 * Ts * Ts) / denum;
    coef3.a2 = (1 - 2 * B * Ts + (w_0 * w_0 * Ts * Ts / 4)) / denum;
    return coef3;
}

void resetError(struct PID_ERRORS *error)
{
	error->e = 0.0;
	error->sum = 0.0;
	error->w = 0.0;
	error->past = 0.0;
}


struct STATIONARY abc2albe(struct THREE_PHASE_VALUE in)
{
    struct STATIONARY out;

    out.alpha   = (in.A -0.5* in.B -0.5* in.C);
    out.beta    = ( 0.5 * SQRT3*in.B - 0.5*SQRT3*in.C);
//magnitude invariant transformation
    return out;
}

struct SYNCHRONOUS albe2dq(struct STATIONARY in, float32 refsin, float32 refcos)
{
	struct SYNCHRONOUS out;

	out.D = refcos*in.alpha + refsin*in.beta;
	out.Q = -refsin*in.alpha + refcos*in.beta;

	return out;
}


struct THREE_PHASE_VALUE albe2abc(struct STATIONARY in)
{
	struct THREE_PHASE_VALUE out;
	out.A = in.alpha;
	out.B = (in.alpha - (SQRT3*in.beta)) * -0.5;
	out.C = (in.alpha + (SQRT3*in.beta)) * -0.5;

	return out;
}

struct STATIONARY dq2albe(struct SYNCHRONOUS in, float32 refsin, float32 refcos)
{
	struct STATIONARY out;

	out.alpha 	= refcos*in.D - refsin*in.Q;
	out.beta 	= refsin*in.D + refcos*in.Q;

	return out;
}

struct SYNCHRONOUS abc2dq(struct THREE_PHASE_VALUE in, struct STATIONARY *stat, float32 refsin, float32 refcos)
{
	struct SYNCHRONOUS out;

	stat->alpha = in.A;
	stat->beta  = INV_SQRT3*(in.B - in.C);
	out.D = refcos*stat->alpha + refsin*stat->beta;
	out.Q = -refsin*stat->alpha + refcos*stat->beta;

//	*stat = abc2albe(in);
//	out = albe2dq(*stat, refsin, refcos);

	return out;
}

struct THREE_PHASE_VALUE dq2abc(struct SYNCHRONOUS in, struct STATIONARY *stat, float32 refsin, float32 refcos)
{
	struct THREE_PHASE_VALUE out;

	stat->alpha = refcos*in.D - refsin*in.Q;
	stat->beta  = refsin*in.D + refcos*in.Q;

	out.A = stat->alpha;
	out.B = (stat->alpha - (SQRT3*stat->beta)) * -0.5;
	out.C = (stat->alpha + (SQRT3*stat->beta)) * -0.5;

//	*stat = dq2albe(in, refsin, refcos);
//	out = albe2abc(*stat);

	return out;
}


//==== Minh Tam

float32 scale_testADC(float32 in_dsp, float32 offset, float32 gain)
{
    return (in_dsp - offset)*gain;
}

float32 duty_ramp(float32 duty, float32 duty_cmd, float32 ISR)
{
    float32 out;
//    float32 delta = 1.0/ISR;
    float32 delta = 1.0/ISR;
    if(duty < duty_cmd)
        duty = duty + delta;
    if(duty > duty_cmd)
        duty = duty - delta;
//    if(abs(duty - duty_cmd) <= 0.01) duty = duty_cmd;
      out = duty;
    return out; // cmt
}

float32 ramp_cycle_gen(float32 in, float32 Freq, float32 ISR )
{
    in += (1.0/ISR * Freq);
    if (in>(1.0))
        in -= (1.0);
    return in; // 0->1
}
//+++++++++++++++++++
float32 scaled_sensor(Uint16 ADC_Result, float32 offset){

    float32 out;
    float R1 = 4.7;
    float R2 = 5.6;
    float Rsen1 = 7.0*330.0;
    float Rsen2 = 1.0;
    float32 Vadc = ADC_Result *  3.3/4096.0;
    float32 Out_Sensor = (Vadc - offset) * R1 / R2;
    float32 In_Sensor = Out_Sensor/8.2;
    out = In_Sensor * (Rsen1 + Rsen2)/Rsen2;
   return out;
 }

float32 scaled_sensor_dc(Uint16 ADC_Resultdc, float32 offsetdc){

    float32 outdc;
    float R1dc = 3.3;
    float R2dc = 2.2;
    float Rsen1dc = 7.0*330.0;
    float Rsen2dc = 0.51;
    float32 Vadcdc = ADC_Resultdc *  3.3/4096.0;
    float32 Out_Sensordc = (Vadcdc - offsetdc) * R1dc / R2dc;
    float32 In_Sensordc = Out_Sensordc/8.2;
    outdc = In_Sensordc * (Rsen1dc + Rsen2dc)/Rsen2dc;
   return outdc;
 }

float scaled_sensor_CURR(unsigned int ADC_Result, float offset_IC, float offset_ref, float offset_comp,float R1, float R2)
{
    return (((ADC_Result * 3.3/4095.0) - offset_ref - offset_comp) * R1/R2 - offset_IC)/ 0.03075; // 30.75 ACS724
}

float scaled_sensor_VOLT(unsigned int ADC_Result, float offset_IC, float offset_ref, float offset_comp, float R1, float R2, float Rsen1, float Rsen2)
{
    return ((((ADC_Result *  3.3/4095.0) - offset_ref - offset_comp) * R1 / R2 - offset_IC)/8.2) * (Rsen1 + Rsen2)/Rsen2; // Gain IC  8.2
}

float get_offset(float in, float signal)
{
    if(signal > 0) return in + 0.000004;
    else if(signal < 0) return in - 0.000004;
    return in;
}

float32 V_adc(float32 adc_value){
    return adc_value*3.3/4096;
}
float32 out_sensor(float32 vadc, float32 offset){
    return  (vadc - offset) * 3.3 / 2.7;
}
void reset_PIDError(struct PID_ERRORS *error)
{
    error->e = 0.0;
    error->sum = 0.0;
    error->w = 0.0;
    error->past = 0.0;
}

struct COEF_1ST getcoeff_LF(float32 Kp, float32 Ki, float32 Ts){
    struct COEF_1ST out;
    out.b0 =   (2.0*Kp + Ki * Ts)/2.0;
    out.b1 = - (2.0*Kp - Ki * Ts)/2.0;
    out.a1 = -1;
    out.a0 = 0;
    return out;
}
void PLL_INIT(float32 grid_freq, float32 sampling_time,struct COEF_1ST lf_para,struct SRF *PLL){
    //initialization
    PLL->fn = grid_freq;
    PLL->fo = 0.0;
   //
    PLL->theta[0] = 0.0;
    PLL->theta[1] = 0.0;
    //
    PLL->v_q[0] = 0.0;
    PLL->v_q[1] = 0.0;
    //
    PLL->ylf[0] = 0.0;
    PLL->ylf[1] = 0.0;
    //
    PLL->delta_T = sampling_time;
    PLL->coeff_LF = lf_para;
}

void SRF_PLL_FUNC(struct SRF *pll){
     pll->ylf[0] = pll->ylf[1] + (pll->coeff_LF.b0*pll->v_q[0]) + \
                  (pll->coeff_LF.b1 * pll->v_q[1]);
     pll->ylf[1]=pll->ylf[0];
     pll->v_q[1]=pll->v_q[0];
     pll->fo=pll->fn + pll->ylf[0];
     pll->theta[0]=pll->theta[1] + ((pll->fo*pll->delta_T)*(float32)(2*3.1415926));

     if(pll->theta[0] > (float32)(2*3.1415926))
     pll->theta[0]=pll->theta[0] - (float32)(2*3.1415926);

     pll->theta[1]=pll->theta[0];
}
/////////////////////
void INIT_COEF_2ND(struct COEF_2ND *in){
    in->a0 = 0.0;     in->a0 = 0.0;
    in->a1 = 0.0;     in->b1 = 0.0;
    in->a2 = 0.0;     in->b2 = 0.0;
}

// pd based on sogi+ psd.
void INIT_DDSOGI(struct PD in){
    in.k =0.0;
    in.sogi_coef.a0 =0;     in.sogi_coef.b0 =0;
    in.sogi_coef.a1 =0;     in.sogi_coef.b1 =0;
    in.sogi_coef.a2 =0;     in.sogi_coef.b2 =0;
    in.psd1.alpha =0.0;     in.psd2.alpha =0.0;
    in.psd1.beta =0.0;      in.psd2.beta =0.0;
    in.qsogi_coef  =  in.sogi_coef;

}
void COEF_DDSOGI_UPDATE(float32 sampl_time, float32 res_freq, struct PD in){
    float32 x,y,temp;
    in.k =0.5;
    x = 2.0* in.k * res_freq *sampl_time;
    y = res_freq * sampl_time * res_freq * sampl_time;
    temp = 1.0 / (x + y + 4.0);
   in.sogi_coef.b0 = (x * temp);
   in.sogi_coef.b2 = (-1.0) *in.sogi_coef.b0;
   in.sogi_coef.a1 = ((2.0 * (4.0 - y)) * temp);
   in.sogi_coef.a2 = ((x - y - 4) * temp);
   in.qsogi_coef.b0 = ((0.5 * y) * temp);
   in.qsogi_coef.b1 = ( in.qsogi_coef.b0 * (2.0));
   in.qsogi_coef.b2 =in.qsogi_coef.b0;
}
/*
void QSG_FUNC (struct PD in){

      in.qsg_in[0] = (pll->qsg_coeff.osg_b0 * (pll->al.u[0] - pll->al.u[2])) + \
          (pll->qsg_coeff.osg_a1 * pll->al.osg_u[1]) + \
          (pll->qsg_coeff.osg_a2 * pll->al.osg_u[2]);
      pll->al.osg_u[2] = pll->al.osg_u[1];
      pll->al.osg_u[1] = pll->al.osg_u[0];
      pll->al.osg_qu[0] = (pll->qsg_coeff.osg_qb0 * pll->al.u[0]) + \
          (pll->qsg_coeff.osg_qb1 * pll->al.u[1]) + \
          (pll->qsg_coeff.osg_qb2 * pll->al.u[2]) + \
          (pll->qsg_coeff.osg_a1 * pll->al.osg_qu[1]) + \
          (pll->qsg_coeff.osg_a2 * pll->al.osg_qu[2]);
      pll->al.osg_qu[2] = pll->al.osg_qu[1];
      pll->al.osg_qu[1] = pll->al.osg_qu[0];
      pll->al.u[2] = pll->al.u[1];
      pll->al.u[1] = pll->al.u[0];

} */

struct STATIONARY Vgrid1phase_to_albe(float32 pre_error_VgPLL,float32 error_VgPLL,float32 Vgrid1phase, float32 sampl_time, float32 K,struct STATIONARY albe2)
{
    struct STATIONARY out;
    pre_error_VgPLL = Vgrid1phase - albe2.alpha;
    error_VgPLL =pre_error_VgPLL *K - albe2.beta ;
    out.alpha =albe2.alpha+ error_VgPLL  * sampl_time *314.159;  /*314.159 */;

    out.beta = albe2.beta+ out.alpha* sampl_time*314.159;
    return out;
}


Uint16 scaled_sensor_rever(float32 sensor, float32 offset){

    Uint16 out;
    float R1 = 4.7;
    float R2 = 5.6;
    float Rsen1 = 7.0*390.0;
    float Rsen2 = 1.2;

    float32 In_Sensor = sensor* Rsen2/(Rsen1 + Rsen2);

    float32  Out_Sensor =In_Sensor  *8.2;

    float32 Vadc =offset + Out_Sensor  * R2 / R1;

    out = Vadc *  4096.0/3.3;

   return out;
 }

float updateRef(float ref, float cmd, float delta)
{
    if (ref < cmd - delta)
    {
        ref += delta;
    }
    else if (ref > cmd + delta)
    {
        ref -= delta;
    }
    else
    {
        ref = cmd;
    }
    return ref;
}

void PR_init(PR_Controller *pr, float Kp, float Kr, float T, float omega0, float omegac) {
    pr->Kp = Kp;
    pr->Kr = Kr;
    pr->T = T;
    pr->omega0 = omega0;
    pr->omegac = omegac;

    // Calculate coefficient in z-domain
    pr->b0 = pr->T*pr->T*pr->omega0*pr->omega0 + 4*pr->T*pr->omegac + 4;
    pr->b1 = 2 * pr->T * pr->T * pr->omega0 * pr->omega0 - 8;
    pr->b2 = pr->T*pr->T*pr->omega0*pr->omega0 - 4*pr->T*pr->omegac + 4;
    pr->a1 = 4*pr->Kr*pr->T*pr->omegac;
    pr->a2 = pr->a1;
    // Initialize input and output value
    /*for ( i = 0; i < 2; ++i) {
        pr->prev_error[i][0] = pr->prev_error[i][1] = 0.0;
        pr->prev_output[i][0] = pr->prev_output[i][1] = 0.0;
    }*/
}

float PR_calculate(PR_Controller *pr, float error, int index) {
    // Kp
    float proportional_part = pr->Kp * error;

    // Kr
    float resonant_part = (pr->a1 * error - pr->a2 * pr->prev_error[index][1] - pr->b1 * pr->prev_output[index][0]
    - pr->b2 * pr->prev_output[index][1])/pr->b0;

    // Anti-windup for resonant accumulator (dùng biến PR_AW_limit để chỉnh trên Watch Window)
    extern volatile float PR_AW_limit;
    if (resonant_part > PR_AW_limit) resonant_part = PR_AW_limit;
    else if (resonant_part < -PR_AW_limit) resonant_part = -PR_AW_limit;

    // Output
    float output = proportional_part + resonant_part;

    // Update values to previous register
    pr->prev_error[index][1] = pr->prev_error[index][0];
    pr->prev_error[index][0] = error;
    pr->prev_output[index][1] = pr->prev_output[index][0];
    pr->prev_output[index][0] = resonant_part;

    return output;
}

void Init_Adaptive_SOGI_PLL(struct ADAPTIVE_SOGI_PLL *pll, float32 Ts) {
    pll->v_alpha = 0.0;
    pll->v_beta = 0.0;
    pll->v_dc = 0.0;
    pll->pi_integral = 0.0;
    pll->theta = 0.0;
    pll->k_adaptive = 0.707;
    
    pll->Kp = 60.0;
    pll->Ki = 1400.0;
    pll->k_dc = 0.1;
    pll->gamma = 10000.0;
    pll->lambda = 50.0;
    pll->Ts = Ts;
}

void Adaptive_SOGI_PLL_Func(struct ADAPTIVE_SOGI_PLL *pll) {
    float32 v_q, v_q_norm, error_w;
    float32 error_sogi_1, norm_err_1, d_k_1, d_v_alpha_1, d_v_beta_1, d_v_dc_1;
    float32 v_alpha_pred, v_beta_pred, v_dc_pred, k_pred;
    float32 error_sogi_2, norm_err_2, d_k_2, d_v_alpha_2, d_v_beta_2, d_v_dc_2;
    float32 omega_0 = 314.159265; // 2*PI*50

    // 1. Tinh bien do de chuan hoa
    pll->V_mag = sqrt(pll->v_alpha * pll->v_alpha + pll->v_beta * pll->v_beta);
    if (pll->V_mag < 1.0) pll->V_mag = 1.0;

    // 2. Vong khoa pha (PLL)
    v_q = pll->v_alpha * cos(pll->theta) + pll->v_beta * sin(pll->theta);
    v_q_norm = v_q / pll->V_mag;

    pll->pi_integral += v_q_norm * pll->Ts;
    if (pll->pi_integral > 100.0) pll->pi_integral = 100.0;
    if (pll->pi_integral < -100.0) pll->pi_integral = -100.0;

    error_w = pll->Kp * v_q_norm + pll->Ki * pll->pi_integral;
    if (error_w > 62.8) error_w = 62.8;
    if (error_w < -62.8) error_w = -62.8;

    pll->omega_est = omega_0 + error_w;

    pll->theta += pll->omega_est * pll->Ts;
    if (pll->theta >= 6.2831853) pll->theta -= 6.2831853;
    else if (pll->theta < 0.0) pll->theta += 6.2831853;
    
    pll->theta_out = pll->theta;

    // 3. Adaptive Observer - SOGI (Heun's method)
    error_sogi_1 = pll->v_s - pll->v_alpha - pll->v_dc;
    float32 v_mag_norm = (pll->V_mag > 10.0f) ? pll->V_mag : 311.0f;
    norm_err_1 = error_sogi_1 / v_mag_norm;
    d_k_1 = pll->gamma * (norm_err_1 * norm_err_1) - pll->lambda * (pll->k_adaptive - 0.707);
    d_v_alpha_1 = pll->omega_est * pll->k_adaptive * error_sogi_1 - pll->omega_est * pll->v_beta;
    d_v_beta_1 = pll->omega_est * pll->v_alpha;
    d_v_dc_1 = pll->omega_est * pll->k_dc * error_sogi_1;

    v_alpha_pred = pll->v_alpha + d_v_alpha_1 * pll->Ts;
    v_beta_pred = pll->v_beta + d_v_beta_1 * pll->Ts;
    v_dc_pred = pll->v_dc + d_v_dc_1 * pll->Ts;
    k_pred = pll->k_adaptive + d_k_1 * pll->Ts;

    if (k_pred < 0.1) k_pred = 0.1;
    if (k_pred > 4.0) k_pred = 4.0;

    error_sogi_2 = pll->v_s - v_alpha_pred - v_dc_pred;
    norm_err_2 = error_sogi_2 / v_mag_norm;
    d_k_2 = pll->gamma * (norm_err_2 * norm_err_2) - pll->lambda * (k_pred - 0.707);
    d_v_alpha_2 = pll->omega_est * k_pred * error_sogi_2 - pll->omega_est * v_beta_pred;
    d_v_beta_2 = pll->omega_est * v_alpha_pred;
    d_v_dc_2 = pll->omega_est * pll->k_dc * error_sogi_2;

    pll->v_alpha += (d_v_alpha_1 + d_v_alpha_2) / 2.0 * pll->Ts;
    pll->v_beta += (d_v_beta_1 + d_v_beta_2) / 2.0 * pll->Ts;
    pll->v_dc += (d_v_dc_1 + d_v_dc_2) / 2.0 * pll->Ts;
    pll->k_adaptive += (d_k_1 + d_k_2) / 2.0 * pll->Ts;

    if (pll->k_adaptive < 0.1) pll->k_adaptive = 0.1;
    if (pll->k_adaptive > 4.0) pll->k_adaptive = 4.0;
}
