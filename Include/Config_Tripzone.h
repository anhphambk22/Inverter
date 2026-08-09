/*
 * Config_Tripzone.h
 *
 *  Created on: 2015. 11. 13.
 *      Author: Administrator
 */

#ifndef HYBRIDESS_28377D_MERGE_INCLUDE_CONFIG_TRIPZONE_H_
#define HYBRIDESS_28377D_MERGE_INCLUDE_CONFIG_TRIPZONE_H_

#include "F28x_Project.h"     // Device Headerfile and Examples Include File

union CHECK_TRIP{
	Uint16 all;
	struct CHECK_TRIP_BITS{
		Uint16 TZ1:1;
		Uint16 TZ2:1;
		Uint16 TZ3:1;
		Uint16 TZ4:1;
		Uint16 TZ5:1;
		Uint16 TZ6:1;
		Uint16 rsvd:10;
	}bit;
};

#define	SIZE_CHECK_TRIP		3
extern union CHECK_TRIP checkTrip[SIZE_CHECK_TRIP];

extern void conf_Tz_Qualification(Uint16 Qsel, float32 samplingWindow);
extern void InitTzGpio(void);


#endif /* HYBRIDESS_28377D_MERGE_INCLUDE_CONFIG_TRIPZONE_H_ */
