// Type.h //

#include "F28x_Project.h"

struct PID_GAINS{

	float32 P;
	float32 I;
	float32 D;
	float32 a;	// anti windup gain
};

struct PID_ERRORS{

	float32 e;
	float32 sum;
	float32 past;
	float32 w;
};


struct LPF_COEF{

	float32 a0;
	float32 a1;
	float32 b0;
	float32 b1;
};

struct Z_1ST{

	float32 in;
	float32 out;
};

struct TypeII_GAINS{

    float32 P;
    float32 I;
    float32 wp;
    float32 a;  // anti windup gain
};

struct COEF_1ST{

	float32 a0;
	float32 a1;
	float32 b0;
	float32 b1;
};

struct Z_2ND{

	float32 in;
	float32 out;
	struct Z_1ST Zn1;
};

struct COEF_2ND{

	float32 a0;
	float32 a1;
	float32 a2;
	float32 b0;
	float32 b1;
	float32 b2;
};

struct STATIONARY{

	float32 alpha;
	float32 beta;

};

struct SYNCHRONOUS{

	float32 D;
	float32 Q;
};

struct THREE_PHASE_VALUE{

	float32 A;
	float32 B;
	float32 C;
};
// NVMT
//===
struct SRF{
    struct COEF_1ST         coeff_LF; // Loop filter based on PI controller.
    float32                 v_q[2]; // input Q
    float32                 ylf[2]; // output LF
    float32                 fo; // output frequency of PLL
    float32                 fn; //nominal frequency
    float32                 theta[2];   // output of PLL
    float32                 delta_T;  // Sampling time
};
struct PD{
    float32             k;
    struct COEF_2ND     sogi_coef, qsogi_coef;
    struct STATIONARY   psd1, psd2;
    float32             qsg_in[2], qsg_inq[2];
    float32             qsg_out[2], qsg_outq[2];
};
// Phase detector based on DDSOGI + PSD
typedef struct {
    float Kp;        // Proportional coefficient
    float Kr;        // Resonant coefficient
    float T;         // Sampling time
    float omega0;    // Resonant frequency
    float omegac;        // Cut-off frequency
    float a1, a2;        // a1 coefficient
    float b0, b1, b2; // b0, b1, b2 coefficient
    float prev_error[3][2]; // Memory for previous input values
    float prev_output[3][2]; // Memory for previous output values
} PR_Controller;
