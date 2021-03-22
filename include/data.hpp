#ifndef BFM_DATA_H
#define BFM_DATA_H


#include <fstream>
#include <iostream>
#include <vector>
#include "constant.h"
#include "hdf5.h"
#include "H5Cpp.h"
#include <Eigen/Dense>


using namespace H5;
using namespace std;
using Eigen::MatrixBase;


/* Macro Function: load_hdf5_model
* Usage: load_hdf5_model(model_type, dataset_path, data_type);
* Parameters:
* 		model_type:   data name, e.g. shape_mu;
* 		dataset_path: dataset path in .h5 file, e.g. "/shape/model/mean"
* 		data_type:    data type in dataset, e.g. PredType::NATIVE_FLOAT
* ***********************************************************************
* Load data from .h5 format file into corresponding data struction.
*/

#define LOAD_H5_MODEL(model_type, dataset_path, data_type) \
		{ \
			DataSet model_type##DataSet = file.openDataSet(dataset_path); \
			model_type##DataSet.read(model_type, data_type); \
			model_type##DataSet.close(); \
			bfm_utils::Raw2Mat(m_##model_type, model_type); \
			if(model_type){ \
				delete [] model_type; \
				model_type = nullptr; \
			} \
		} 


namespace bfm_utils
{

	template<typename _Tp, typename _Ep>
	void Raw2Mat(MatrixBase<_Tp> &m, _Ep *raw) 
	{
		for (unsigned int i = 0; i < m.rows(); i++)
			for (unsigned int j = 0; j < m.cols(); j++)
				m(i, j) = raw[i * (unsigned int)m.cols() + j];
	}

	
	template<typename _Tp, typename _Ep>
	void LoadH5Model(hid_t file, const std::string& strPath, _Tp *aData, MatrixBase<_Ep>& matData, hid_t predType)
	{
		hid_t dataSet = H5Dopen(file, strPath.c_str(), H5P_DEFAULT);
		herr_t status = H5Dread(dataSet, predType, H5S_ALL, H5S_ALL, H5P_DEFAULT, aData);
		Raw2Mat(matData, aData);
	}


} // NAMESPACE BFM_UTILS


#endif // BFM_DATA_H