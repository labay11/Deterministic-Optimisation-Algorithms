#include "ros.h"

double rosenbrock(
	double x1, double x2
) {
	return 100.0 * pow(x2 - x1*x1, 2.0) + pow(1 - x1, 2.0);
}

double drosenbrock(
	double x1, double x2, int n
) {
	if (n == 0)
		return 2.0 * (200 * (x1*(x1*x1 - x2)) + x1 - 1);
	else if (n == 1)
		return 200 * (x2 - x1*x1);

	printf("Undefined variable %d", n);
	return 0;
}

double ddrosenbrock(
	double x1, double x2, int n1, int n2
) {
	if (n1 == 0) {
		if (n2 == 0)
			return -400.0 * (x2 - x1*x1) + 800.0 * x1*x1 + 2;
		else if (n2 == 1)
			return -400.0 * x1;
	} else if (n1 == 1) {
		if (n2 == 0)
			return -400.0 * x1;
		else if (n2 == 1)
			return 200.0;
	}

	printf("Undefined variables %d %d", n1, n2);

	return 0;
}