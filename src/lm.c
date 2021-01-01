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

void hessian_f(
	int n_vars,
	double * x,
	double * out
) {
	for (int i = 0; i < n_vars; i++) {
		out[i*n_vars + i] = ddrosenbrock(x[0], x[1], i, i);
		for (int j = i + 1; j < n_vars; j++)
			out[i*n_vars + j] = out[j*n_vars + i] = ddrosenbrock(x[0], x[1], i, j);
	}
}

void inv_mat2(
	double * in, double * out
) {
	double d = in[0] * in[3] - in[1] * in[2];
	out[0] = in[3] / d;
	out[1] = -in[1] / d;
	out[2] = -in[2] / d;
	out[3] = in[0] / d;
}

double * lm(
	int n_vars, 
	double * x0, 
	double tolerance
) {
	double * x, * grad, * H, * LM, * LM_inv;
	x = (double *) malloc(n_vars * sizeof(double)); 			// point vector
	grad = (double * ) malloc(n_vars * sizeof(double));		// gradient vector
	H = (double *) malloc(n_vars * n_vars * sizeof(double));	// hessian matrix
	LM = (double *) malloc(n_vars * n_vars * sizeof(double));	// LM matrix
	LM_inv = (double *) malloc(n_vars * n_vars * sizeof(double));	// inverse LM matrix

	double fx, fx_tmp, step, lambda = 0.005;

	int i, j, iters = 0;
	for (i = 0; i < n_vars; i++) x[i] = x0[i];

	do {
		++iters;

		fx = f(x);
		grad_f(n_vars, x, grad);
		hessian_f(n_vars, x, H);
		
		// compute LM matrix (using the fact that H is symetric)
		for (i = 0; i < n_vars; i++) {
			LM[i * n_vars + i] = H[i * n_vars + i] + lambda;
			for (j = i + 1; j < n_vars; j++) {
				LM[i * n_vars + j] = LM[j * n_vars + i] = H[i * n_vars + j];
			}
		}

		inv_mat2(LM, LM_inv);

		// step
		for (i = 0; i < n_vars; i++) {
			step = 0.0;
			for (j = 0; j < n_vars; j++) step += LM_inv[i*n_vars + j] * grad[j];
			x[i] -= step;
		}

		fx_tmp = f(x);
		if (fx_tmp < fx) lambda /= 2.0;
		else lambda *= 2.0;
	} while (fabs(fx_tmp - fx) > tolerance);

	printf("Minimisation finixed in %d iterations.\n", iters);

	free(grad);
	free(H);

	return x;
}

int main(int argc, char ** argv) {
	double x0[] = {-1.5, -1};
	clock_t begin = clock();
	double * res = lm(2, x0, 1e-8);
	clock_t end = clock();
	printf("Result found: (%.15f, %.15f)\nTime: %.6fs\n", res[0], res[1], ((double)(end - begin))/CLOCKS_PER_SEC);
	free(res);

	return 0;
}