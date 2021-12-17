#ifndef _MAIN_H_
#define _MAIN_H_

#include <stdbool.h>
#include <iostream>
#include "mujoco210/include/mujoco.h"
#include "mujoco210/include/glfw3.h"
#include "eigen-3.4.0/Eigen/Dense"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

typedef struct Motor
{
	float ks;
	float kd;
	float target_pos;
	float target_vel;
	float t_ff;

}motor;


#endif