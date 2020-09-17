#ifndef TRANSFORM_HPP
#define TRANSFORM_HPP
#define _USE_MATH_DEFINES
#include <cmath>
#include <fstream>
#include <opencv2/opencv.hpp> 
#include <Eigen/Dense>

using Eigen::Matrix;
using Eigen::MatrixBase;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::Map;
using Eigen::Dynamic;
using Eigen::Ref;

namespace bfm_utils {


#ifndef BFM_SHUT_UP                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  


	template <size_t _Rz, size_t _Cz>
	void PrintDoubleCvMat(CvMat *src) 
	{
		for(int i = 0; i < _Rz; i++)
		{
			for(int j = 0; j < _Cz; j++)
			{
				std::cout << cvmGet(src, i, j) << " ";
			}
			
			std::cout << std::endl;
		}
	}


#else


	template <size_t _Rz, size_t _Cz> void PrintDoubleCvMat(CvMat *src) { }


#endif


	template<typename _Tp>
	void EigenMat2CvMat(const MatrixBase<_Tp> &src, CvMat *dst) 
	{
		cvZero(dst);
		for(unsigned int i = 0; i < src.rows(); i++)
		{
			for(unsigned int j = 0; j < src.cols(); j++) 
			{
				cvmSet(dst, i, j, src(i, j));
			}
		}		
	}

	template<typename _Tp>
	void CvMat2EigenMat(CvMat *src, MatrixBase<_Tp> &dst) 
	{
		for(unsigned int i = 0; i < dst.rows(); i++)
			for(unsigned int j = 0; j < dst.cols(); j++)
				dst(i, j) = cvmGet(src, i, j);
	}


	/* 
	* Function: euler2matrix
	* Usage: dlib::matrix<T> R = euler2matrix(yaw, pitch, roll, false);
	* Parameters:
	* 		@yaw: Euler angle (radian);
	* 		@pitch: Euler angle (radian);
	* 		@roll: Euler angle (radian);
	* 		@is_linearized: Choose to use linearized Euler angle transform or not. If true, be sure yaw, pitch and roll
	* 						keep small.
	* Return:
	* 		Rotation matrix R.
	* 		If linearized:
	* 			R = [[1,      -yaw, pitch],
	* 				 [yaw,    1,    -roll],
	* 				 [-pitch, roll, 1    ]]
	* 		Else
	*			R = [[c2*c1, s3*s2*c1-c3*s1, c3*s2*c1+s3*s1],
	*				 [c2*s1, s3*s2*s1+c3*c1, c3*s2*s1-s3*c1],
	*				 [-s2,   s3*c2,          c3*c2         ]]; 
	*			(c1=cos(yaw),   s1=sin(yaw))
	*			(c2=cos(pitch), s2=sin(pitch))
	*			(c3=cos(roll),  s3=sin(roll))
	* ----------------------------------------------------------------------------------------------------------------
	* Transform Euler angle into rotation matrix.
	* 
	*/

	template<typename _Tp> inline 
	Matrix<_Tp, 3, 3> Euler2Mat(const _Tp &yaw, const _Tp &pitch, const _Tp &roll, bool is_linearized = false)
	{
		// Z1Y2X3

		/* yaw - phi */	
		/* pitch - theta */		
		/* roll - psi */
		Matrix<_Tp, 3, 3> r_mat;
		if(is_linearized)
		{
			r_mat << _Tp(1.0), -yaw,     pitch,
				     yaw,      _Tp(1.0), -roll,
				     -pitch,   roll,     _Tp(1.0);
		}
		else
		{
			/* (Deprecated) Using angles. */ 
			// _Tp c1 = cos(yaw   * _Tp(M_PI) / _Tp(180.0)), s1 = sin(yaw   * _Tp(M_PI) / _Tp(180.0));
			// _Tp c2 = cos(pitch * _Tp(M_PI) / _Tp(180.0)), s2 = sin(pitch * _Tp(M_PI) / _Tp(180.0));
			// _Tp c3 = cos(roll  * _Tp(M_PI) / _Tp(180.0)), s3 = sin(roll  * _Tp(M_PI) / _Tp(180.0));
			
			/* Using radians */
			_Tp c1 = cos(yaw),   s1 = sin(yaw);
			_Tp c2 = cos(pitch), s2 = sin(pitch);
			_Tp c3 = cos(roll),  s3 = sin(roll);
			r_mat << c2 * c1, s3 * s2 * c1 - c3 * s1, c3 * s2 * c1 + s3 * s1,
				     c2 * s1, s3 * s2 * s1 + c3 * c1, c3 * s2 * s1 - s3 * c1,
				     -s2,     s3 * c2,                c3 * c2; 
		}
		return r_mat;
	}


	template<typename _Tp, typename _Ep>
	Matrix<_Tp, Dynamic, 1> TransPoints(
		const Matrix<_Tp, 3, 3> &matR, 
		const Matrix<_Tp, 3, 1> &vecT, 
		const Matrix<_Ep, Dynamic, 1> &vecPoints) 
	{
		Matrix<_Tp, 3, 4> matTrans;
		matTrans << matR, vecT;
		Matrix<_Tp, Dynamic, 1> vecPointsTypeTurned = vecPoints.template cast<_Tp>();
		Map<Matrix<_Tp, Dynamic, 3, Eigen::RowMajor>> matPoints(vecPointsTypeTurned.data(), vecPointsTypeTurned.rows() / 3, 3);
		Matrix<_Tp, 4, Dynamic> matPointsTransposed = matPoints.rowwise().homogeneous().transpose();
		// Matrix<_Tp, 3, Dynamic> matPointsTransformed = matTrans * (matPoints.rowwise().homogeneous().transpose()); // Stuck!
		Matrix<_Tp, 3, Dynamic> matPointsTransformed = matTrans * matPointsTransposed;
		return Map<Matrix<_Tp, Dynamic, 1>>(matPointsTransformed.transpose().data(), vecPoints.rows(), 1);
	}


	template<typename _Tp, typename _Ep>
	Matrix<_Tp, Dynamic, 1> TransPoints(const _Tp * const aExtParams, const Matrix<_Ep, Dynamic, 1> &vecPoints, bool bIsLinearized = false)
	{
		Matrix<_Tp, 3, 3> matR;
		Matrix<_Tp, 3, 1> vecT;
		matR = bfm_utils::Euler2Mat(aExtParams[0], aExtParams[1], aExtParams[2], bIsLinearized);
		vecT << aExtParams[3], aExtParams[4], aExtParams[5];
		return bfm_utils::TransPoints(matR, vecT, vecPoints);
	}



	template<typename T>
	void Trans(const double ext_parm[6], T &x, T &y, T &z) {
		const double &yaw   = ext_parm[0];
		const double &pitch = ext_parm[1];
		const double &roll  = ext_parm[2];
		const double &tx    = ext_parm[3];
		const double &ty    = ext_parm[4];
		const double &tz    = ext_parm[5];

		/* yaw - phi */
		double c1 = cos(yaw   * M_PI / 180.0), s1 = sin(yaw   * M_PI / 180.0);
		/* pitch - theta */
		double c2 = cos(pitch * M_PI / 180.0), s2 = sin(pitch * M_PI / 180.0);
		/* roll - psi */
		double c3 = cos(roll  * M_PI / 180.0), s3 = sin(roll  * M_PI / 180.0);

		T X = x, Y = y, Z = z; 

		x = ( c2 * c1) * X + (s3 * s2 * c1 - c3 * s1) * Y + (c3 * s2 * c1 + s3 * s1) * Z + tx;
		y = ( c2 * s1) * X + (s3 * s2 * s1 + c3 * c1) * Y + (c3 * s2 * s1 - s3 * c1) * Z + ty;
		z = (-s2     ) * X + (s3 * c2               ) * Y + (c3 * c2               ) * Z + tz; 
	}


	bool IsRMat(const Matrix3d &r_mat)
	{
		Matrix3d dif = r_mat * r_mat.transpose() - Matrix3d::Identity();
		return (dif.norm() < 1e-6);
	}


	void SatisfyExtMat(Ref<Matrix3d> matR, Ref<Vector3d> vecT)
	{
		CvMat *cvMatTrans, *cvMatR, *cvMatT;
		Matrix<double, 3, 4> matTrans;
		matTrans << matR, vecT;

		cvMatTrans = cvCreateMat(3, 4, CV_64FC1);
		bfm_utils::EigenMat2CvMat(matTrans, cvMatTrans);
		
		cvMatR = cvCreateMatHeader(3, 3, CV_64FC1);
		cvGetCols(cvMatTrans, cvMatR, 0, 3);
		cvMatT = cvCreateMatHeader(3, 1, CV_64FC1);
		cvGetCol(cvMatTrans, cvMatT, 3);

		if( cvDet(cvMatR) < 0)
			cvScale(cvMatTrans, cvMatTrans, -1);
		double sc = cvNorm(cvMatR);
		CV_Assert(fabs(sc) > DBL_EPSILON);

		double u[9], v[9], w[3];
		CvMat cvMatU = cvMat(3, 3, CV_64F, u);
		CvMat cvMatV = cvMat(3, 3, CV_64F, v);
		CvMat cvMatW = cvMat(3, 1, CV_64F, w);

		cvSVD(cvMatR, &cvMatW, &cvMatU, &cvMatV, CV_SVD_MODIFY_A + CV_SVD_U_T + CV_SVD_V_T);
		cvGEMM(&cvMatU, &cvMatV, 1, 0, 0, cvMatR, CV_GEMM_A_T);

		cvScale(cvMatT, cvMatT, cvNorm(cvMatR) / sc);

		CvMat2EigenMat(cvMatR, matR);
		CvMat2EigenMat(cvMatT, vecT);
	}
}

#endif	// TRANFORM_HPP