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
#include "planner.h"
#include "motionPlanning.h"
#include "odrive_main.h"

#define DPOS  (float)0.225//0.225

//define the planner class as being global


extern volatile TrpzInfo trpz_info;


float Xpeak[3 + 1] = {0}; //Target angle(must be positive), Vpeak, Apeak, Jerk
float T[3 + 1] = {0}; //order is 3 for S curve
float temp_memo[25] = {0};
double tfinal;
volatile long counter = 0;
int direction_scurve = 1;
int move_direction = 1;
unsigned long scurve_start_time_in_micros;
unsigned long scurve_end_time_in_micros;


long Planner::tick(void) {
        if (fabs(currentSetAngle - endAngle) >= DPOS) {
            volatile float current_time=(HAL_GetTick() - trpz_info.start_time) / 1000.0;
            // volatile float current_time = (micros() - trpz_info.start_time) / 1000000.0;
            // fixed time version
            currentSetAngle = startAngle + move_direction * getSetPointTrapezoid(
                    current_time + trpz_time_intv);
//            move motor
//            ptrStepperCtrl->moveToAbsAngle(ANGLE_FROM_DEGREES(currentSetAngle));
              // Serial1.println("p 1 "+String(currentSetAngle));

            trpz_time_intv1 = current_time + trpz_time_intv;
        } else { // dead zone

            trpz_stage = 3;
//            ptrStepperCtrl->moveToAbsAngle(ANGLE_FROM_DEGREES(endAngle));
//            currentMode = PLANNER_NONE;
        }
        return currentSetAngle;
}

bool Planner::moveTrapezoid(float finalAngle, float v_max, float acc, float old_angle) {
    memset(Xpeak, 0, 4 * sizeof(float));
    memset(T, 0, 4 * sizeof(float));

//    if (v_max > 300) {
//        v_max = 300;
////        serial_println(SerialUSB, "WARNING: speed too high. Changing speed to 300 rpm.");
//    }

    //get current position
//    startAngle = ANGLE_T0_DEGREES(ptrStepperCtrl->getCurrentAngle());
    startAngle = old_angle;

    if (finalAngle - startAngle > 0) move_direction = 1;
    else move_direction = -1;
    Xpeak[0] = fabs(finalAngle - startAngle);
    Xpeak[1] = fabs(v_max) / 60.0 * 360.0;   // convert rpm to degree per second
    Xpeak[2] = fabs(acc);

    computePeriodsTrapezoid(Xpeak, T);
    trpz_info.acc = Xpeak[2];
    trpz_info.v_max = Xpeak[1];
    trpz_info.t1 = T[2];
    trpz_info.t2 = T[1] + T[2];
    trpz_info.t3 = T[1] + T[2] * 2;
    trpz_info.start_time = HAL_GetTick();
    trpz_info.d1 = pow(trpz_info.t1, 2) * trpz_info.acc / 2;
    trpz_info.d2 = trpz_info.d1 + (trpz_info.t2 - trpz_info.t1) * trpz_info.v_max;
    trpz_info.d3 = fabs(finalAngle - startAngle);

    tfinal = T[0] * 2 + T[1];

    endAngle = finalAngle;

    //set the current angle
    currentSetAngle = startAngle;

    return true;
}
