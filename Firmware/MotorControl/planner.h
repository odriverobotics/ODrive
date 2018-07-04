/**********************************************************************
 *      Author: tstern
 *
 *	Misfit Tech invests time and resources providing this open source code,
 *	please support Misfit Tech and open-source hardware by purchasing
 *	products from Misfit Tech, www.misifittech.net!
 *
 *	Written by Trampas Stern  for Misfit Tech.
 *	BSD license, check license.txt for more information
 *	All text above, must be included in any redistribution
 *********************************************************************/

/*
 *  This file implements a trajectory planner for use with serial
 *  interface. It allows the smart stepper to move at constant velocity.
 *  Additionally you can move to some location at constant velocity or
 *  with a trajectory curve
 */

#ifndef PLANNER_H_
#define PLANNER_H_

#include "odrive_main.h"

#define UINT32_T_MAX 65535
#define PLANNER_UPDATE_RATE_HZ (3000UL) //how often planner updates PID
#define EXCEEDANCE_MAX 50

class Planner {
private:
    //todo we should not use floating point, rather use "Angle"
    volatile float endAngle;
    volatile float startAngle;
    volatile float currentSetAngle;
    volatile float tickIncrement;
    volatile int trpz_stage;
    volatile float trpz_time;
    volatile uint32_t trpz_time_intv;
    volatile float trpz_time_intv1;
    volatile uint32_t trpz_time_intv2;
    volatile float dt;
    volatile float dt_count_down;
    volatile float vi;
    volatile float transition_angle_diff;
    volatile int effort_exceedance_count = 0;
    int torque_limit;

public:

    long tick(void); //this is called on regular tick interval
    bool moveTrapezoid(float finalAngle, float v_max, float a_max, float old_angle);
};

// extern Planner SmartPlanner;


#endif /* PLANNER_H_ */
