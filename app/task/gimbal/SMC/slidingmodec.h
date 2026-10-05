#ifndef _SMC_H_
#define _SMC_H_

#include "stm32h7xx_hal.h"
#include <cmath>

class smc{
public:
	float C;
	float K;
	float error_eps;
	float q;
	float p;
	float u_max;
	float J;
	float angle;
	float ang_vel;
	float epsilon;

	float output;

	smc(float C, float K, float error_eps, float q, float p, float u_max, float J, float epsilon):
	C(C), K(K), error_eps(error_eps), q(q), p(p), u_max(u_max), J(J), epsilon(epsilon) {
	};

	float smc_update(float angle_target, float angle_now, float angle_vel);

private:
	float target;
	float error;
	float error_last;
	float target_dot;
	float target_ddot;
	float target_last;

	float s;
	float ds;
	float Sat(float y) {
		if (fabs(y) <= 1)
			return y;
		else
			return Signal(y);
	}

	int8_t Signal(float y) {
		if (y > 0)
			return 1;
		else if (y == 0)
			return 0;
		else
			return -1;
	}
};
#endif
