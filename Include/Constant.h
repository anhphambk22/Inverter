/* Cotant.h */

#define CPU_CLK             (200e6)
#define FREQ_SAMPLING       (10e3)
#define SAMPLING_TIME       (1.0/FREQ_SAMPLING)      //  = 1/FREQ_SAMPLING

#define FREQ_SW             (20e3)
#define GRID_FREQ           (50.0)
#define PI                  (3.1415926535897932)
#define TWOPI               (2.0*PI)
#define MAXDUTY             (1.0f)
#define TWODIV3              (0.666667)
#define ZERO                (0.0f)
#define SQRT3               (1.7320508075689)
#define INV_SQRT3           (0.5773502691896)
#define W60                 (TWOPI * 60.0)
#define W60SAMP             (W60 * SAMPLING_TIME)
#define W50                 (TWOPI * 50.0)
#define W50SAMP             (W50 * SAMPLING_TIME)

#define DEADTIME            (300.0)//(150.0)     // nano sec

#define PHASE_SHIFT            (180.0)     // degree


#define PRD_1SEC            (FREQ_SAMPLING)
#define PRD_2SEC            (FREQ_SAMPLING * 2.0)
#define PRD_3SEC            (FREQ_SAMPLING * 3.0)
#define PRD_4SEC            (FREQ_SAMPLING * 4.0)
#define PRD_5SEC            (FREQ_SAMPLING * 5.0)
#define PRD_6SEC            (FREQ_SAMPLING * 6.0)
#define PRD_7SEC            (FREQ_SAMPLING * 7.0)

/*
#define OVERIB              (70.0)              // max 60A              //hardware protection : 70A
#define OVERVH              (400.0 * 1.1)       // nominal 400VDC       //hardware protection : 400VDC
#define OVERVDC             (350.0 * 1.1)       // nominal 100~350VDC   //hardware protection : 400VDC
#define OVERVB              (30.0)              // 1s : 2 ~ 4.2V
                                                // 2s : 4 ~ 8.4V
                                                // 3s : 6 ~ 12.6V
                                                // 4s : 8 ~ 16.8V
                                                // 5s : 10 ~ 21.0V
                                                // 6s : 12 ~ 25.2V
                                                // 7s : 14 ~ 29.4V
*/
#define DELTA_V_REF         30* SAMPLING_TIME//(100.0 * SAMPLING_TIME)     // 100V per sec
#define DELTA_I_REF         10* SAMPLING_TIME // (60e3 * SAMPLING_TIME)      // 60000A per sec  60A per ms
#define DELTA_OPENLOOP_DUTY (0.05 * SAMPLING_TIME)


