#include "random.h"

std::vector<double> randn(int n) {
	std::vector<double> res;
	std::default_random_engine e;
	std::normal_distribution<double> dis(0, 1.0);
	for (int i = 0; i < n; i++)
		res.push_back(dis(e));
	return res;
}
