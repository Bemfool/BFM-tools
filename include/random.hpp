#ifndef BFM_RANDOM_H
#define BFM_RANDOM_H


#include <random>
#include <time.h>
#include <cassert>


namespace bfm_utils {


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

	double *randn(int nArray, double dScale)
	{
		assert(dScale >= 0.0);

		std::random_device randomDevice;
		std::mt19937 generator(randomDevice());
		double *dResArray = new double[nArray];
		std::normal_distribution<double> dis(0, dScale);

		for (int i = 0; i < nArray; i++)
			dResArray[i] = dis(generator);
		
		return dResArray;
	}


} // NAMESPACE BFM_UTILS


#endif // BFM_RANDOM_H