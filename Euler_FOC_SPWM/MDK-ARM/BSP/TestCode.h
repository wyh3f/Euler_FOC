//TestCode
#ifndef __TestCode_H
#define __TestCode_H



#include "stdio.h"
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include <math.h>
#include "stdbool.h"
#include <stdint.h>


void TestCode_INIT(void);
void TestCode_Main(void);

#include "ALGORITHM_Clarke_Park.h"
#include "PID.h"


extern ThreePhase ABC;
extern ThreePhase PWM;
extern Clarke Alpha_Beta;
extern Park QP_thet;

extern ThreePhase Detection_ABC;
extern Clarke Detection_Alpha_Beta;
extern Park Detection_QP_thet;

extern PIDController pid;
extern PIDController pid_speed;

#endif
