#include"slidingmodec.h"

float smc::smc_update(float angle_target, float angle_now, float angle_vel)
{
	angle = angle_now;
	target = angle_target;
	ang_vel = angle_vel;
	float e_qp;
	float qp = q/p;

	error = angle - target;
	float error_dot = ang_vel - target_dot;
	target_ddot = (target - target_last) - target_dot;
	target_dot = (target - target_last);

	if (fabs(error) < error_eps)
	{
		return 0;
	}

	if (error < 0)
		e_qp = -pow(abs(error), qp);
	else
		e_qp = pow(abs(error), qp);

	s = error_dot + C * e_qp; //smc surface
	ds = -epsilon * Sat(s) - K * s;
	output = J * (target_ddot + ds - C * qp * error_dot * e_qp / (error) );

	if (abs(error) < 1)
	{
		error = angle  - target;

		s = error_dot + C * error; //smc surface
		output = J * (target_ddot - C * error_dot - epsilon * Sat(s) - K * s);
	}

	if (output > u_max)
		output = u_max;
	if (output < -u_max)
		output = -u_max;

	target_last = target;

	return output;
}