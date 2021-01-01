#ifndef __HEADER_ROS__
#define __HEADER_ROS__
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

double rosenbrock(
	double x1, double x2
);

double drosenbrock(
	double x1, double x2, int n
);

double ddrosenbrock(
	double x1, double x2, int n1, int n2
);

#endif