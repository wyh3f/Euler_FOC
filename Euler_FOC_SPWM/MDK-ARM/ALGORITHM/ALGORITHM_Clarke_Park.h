//ALGORITHM_Clarke_Park
#ifndef __ALGORITHM_Clarke_Park_H
#define __ALGORITHM_Clarke_Park_H

//,__CC_ARM,__TARGET_FPU_VFP, __FPU_PRESENT=1U, ARM_MATH_CM4
#include "arm_math.h"

#include "stdio.h"
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include <math.h>
#include "stdbool.h"
#include <stdint.h>

typedef struct ThreePhase
{
	float A;
	float B;
	float C;
}ThreePhase;

typedef struct Clarke
{
	float Alpha;
	float Beta;
}Clarke;

typedef struct Park
{
	float Q;
	float D;
	float thet;
}Park;


#endif


