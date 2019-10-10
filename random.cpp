#include "random.h"

std::vector<double> randn(int n) {
	double V1, V2, S;
	int phase = 0;
	std::vector<double> res;
	for (int i = 0; i < n; i++) {
		if (phase == 0) {
			do {
				double U1 = (double)rand() / RAND_MAX;
				double U2 = (double)rand() / RAND_MAX;
				V1 = 2 * U1 - 1;
				V2 = 2 * U2 - 1;
				S = V1 * V1 + V2 * V2;
			} while (S >= 1 || S == 0);
			res.push_back(V1 * sqrt(-2 * log(S) / S));
		}
		else
			res.push_back(V2 * sqrt(-2 * log(S) / S));
		phase = 1 - phase;
	}
	return res;
}