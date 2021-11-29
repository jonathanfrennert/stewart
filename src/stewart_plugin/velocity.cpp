#include <cmath>
#include "eigen3/Eigen/Core"
#include "velocity.h"

Eigen::Matrix<float, 3, 1> init_velocity(Eigen::Matrix<float, 3, 1> p0, Eigen::Matrix<float, 3, 1> pf, float t)
{
	// g
	float g = 9.81;

	// Initial position
	float x0 = p0[0];
	float y0 = p0[1];
	float z0 = p0[2];

	// Final position
	float xf = pf[0];
	float yf = pf[1];
	float zf = pf[2];

	float vx0 = (xf-x0)/t;
	float vy0 = (yf-y0)/t;
	float vz0 = (zf-z0 + (0.5 * g * t*t))/t;

	// Storing initial velocity
	Eigen::Matrix<float, 3, 1> v0;
	v0 << vx0, vy0, vz0;
	return v0;
}

//int main()
//{
//	Eigen::Matrix<float, 3, 1> x0;
//	x0 << 1.1, 1.1, 2.1;
//
//	Eigen::Matrix<float, 3, 1> xf;
//	xf << 0.0, 0.0, 2.0;
//
//	float t = 1;
//
//	Eigen::Matrix<float, 3, 1> v;
//	v = init_velocity(x0, xf, t);
//	for(int i = 0; i < 3; i++)
//	{
//		printf("velocity component %d, %f\n", i, v[i]);
//	}
//	return 0;
//}
