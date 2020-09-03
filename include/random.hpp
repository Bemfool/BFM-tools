#pragma once
#include <random>
#include <time.h>

/* Function: randn
* Usage:
*		std::vector<double> seq = randn(n);
*		std::vector<double> seq = randn(n, scale);
* Parameters:
*		n - The number of random element to be generated.
* -------------------------------------------------------------
* Generate a sequence of random numbers into a vector sequence.
* Its distribution is normal distribution whose sigma = 0 ,and
* miu = 1.
*/

double *randn(int n, double scale)
{
	std::random_device rd;
	std::mt19937 gen(rd());
	double *res = new double[n];
	std::normal_distribution<double> dis(0, scale);
	for (int i = 0; i < n; i++)
		res[i] = dis(gen);
	return res;
}