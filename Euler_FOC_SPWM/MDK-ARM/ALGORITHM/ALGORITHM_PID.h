//ALGORITHM_PID
#ifndef __ALGORITHM_PID_H
#define __ALGORITHM_PID_H


#include "stdio.h"
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include <math.h>
#include "stdbool.h"
#include <stdint.h>


typedef struct pid
{
	float p;
	float i;
	float d;
	float err1;
	float err2;
	float err3;
}pid;





#endif

