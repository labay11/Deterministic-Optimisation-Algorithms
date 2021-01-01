#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include "ros.h"

double f(double * x) {
	return rosenbrock(x[0], x[1]);
}

void grad_f(
	int n_vars,
	double * x,
	double * out
) {
	for (int i = 0; i < n_vars; i++) out[i] = drosenbrock(x[0], x[1], i);
}

double * cgd(
	int n_vars, 
	double * x0,
	double sigma, double rho,
	double tolerance
) {
	double * x, * x_guess, * grad, * d;
	x = (double *) malloc(n_vars * sizeof(double)); 			// point vector
	x_guess = (double *) malloc(n_vars * sizeof(double)); 		// guess next point vector
	grad = (double * ) malloc(n_vars * sizeof(double));			// gradient vector
	d = (double *) malloc(n_vars * sizeof(double));				// direction vector

	double fx, fx_tmp, alpha, beta, tmp;

	int i, iters = 0;
	for (i = 0; i < n_vars; i++) {
		x[i] = x0[i];
		d[i] = 0.0;
	}
	fx_tmp = f(x);
	do {
		++iters;
		fx = fx_tmp;
		grad_f(n_vars, x, grad);

		beta = 0.0; tmp = 0.0;
		for (i = 0; i < n_vars; i++) {
			beta += grad[i] * d[i];
        	tmp += d[i] * d[i];
		}
		if (iters > 1) beta /= tmp; // skip first iteration

		for (i = 0; i < n_vars; i++)
			d[i] = -grad[i] + beta * d[i];

		// backtracking strategy to compute alpha
		alpha = 1.0;
		tmp = 0.0;
		for (i = 0; i < n_vars; i++) {
			tmp += grad[i] * d[i];
			x_guess[i] = x[i] + alpha * d[i];
		}
		fx_tmp = f(x_guess);
		while ((fx_tmp > fx + sigma * alpha * tmp) || alpha < sigma) {
			alpha *= rho;
			for (i = 0; i < n_vars; i++) x_guess[i] = x[i] + alpha * d[i];
			fx_tmp = f(x_guess);
		}
		
		for (i = 0; i < n_vars; i++) x[i] = x_guess[i];
	} while (fabs(fx_tmp - fx) > tolerance);

	printf("Minimisation finixed in %d iterations.\n", iters);

	free(grad);
	free(x_guess);
	free(d);

	return x;
}

int main(int argc, char ** argv) {
	double x0[] = {-1.5, -1};
	clock_t begin = clock();
	double * res = cgd(2, x0, 1e-4, 0.5, 1e-8);
	clock_t end = clock();
	printf("Result found: (%.15f, %.15f)\nTime: %.6fs\n", res[0], res[1], ((double)(end - begin))/CLOCKS_PER_SEC);
	free(res);

	return 0;
}