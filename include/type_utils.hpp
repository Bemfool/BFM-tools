#pragma once
#include <iostream>
#include <string>
#include "constant.h"

namespace bfm_utils
{

	/* 
	* Struct: is_double
	* Usage: if(is_double<T>()) { }
	* Return:
	* 		Is `double` type or not.
	* ***********************************************************************************
	* Judge that the type is `double` or not.
	* 
	*/

	template <typename T>
	struct isDouble
	{
		operator bool() 
		{
			return false;
		}
	};
	
	template <>
	struct isDouble<double>
	{
		operator bool() 
		{
			return true;
		}
	};


	/* 
	* Function: length
	* Usage: int len = length(array);
	* Parameters:
	* 		@arr: Array.
	* Return:
	* 		Size of array.
	* ***********************************************************************************
	* Get the size of an array.
	* 
	*/

	template<class T> inline 
	int LenArr(T& arr) 
	{
		return sizeof(arr) / sizeof(arr[0]);
	}


#ifndef BFM_SHUT_UP


	/* 
	* Function: PrintArr
	* Usage: print_array(arr);
	* Parameters:
	* 		@arr:	Array to be printed;
	* 		@len: Size of array.
	* ***********************************************************************************
	* Print a list.
	* 
	*/

	template<class _Tp> inline
	void PrintArr(_Tp *arr, int len)
	{
		for(unsigned int i = 0; i < len; i++) std::cout << arr[i] << " ";
		std::cout << "\n";
	}


#else


	template<class _Tp> inline void PrintArr(_Tp *arr, int len) { }


#endif


	template<typename _Tp>
	std::string NumMat2Str(MatrixBase<_Tp> mat)
	{
		std::string res = "";
		for(unsigned int i = 0; i < mat.rows(); i++)
		{
			for(unsigned int j = 0; j < mat.cols(); j++)
			{
				res = res + std::to_string(mat(i, j)) + " ";
			}
			res = res + "\n";
		}
		return res;
	}
}