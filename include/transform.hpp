#pragma once
#define _USE_MATH_DEFINES
#include <cmath>
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


	/* 
	* Function: points2homogeneous
	* Usage: dlib::matrix<T> quaternion_points = points2homogeneous(ternary_points);
	* Parameters:
	* 		@ternary_points: Point matrix whose size is (n_vertice, 3).
	* Return:
	* 		Point matrix whose size is (4, n_vertice).
	* 		$Qmat = [Tmat | 1] ^ T$
	* ----------------------------------------------------------------------------------------------------------------
	* Transform a ternary point vector into quaternion vector. Detailed steps are as follows:
	* 1) Add ones matrix in last column;
	* 2) Transpose;
	* 
	*/

	// template<typename _Tp> inline
	// dlib::matrix<_Tp> points2homogeneous(const dlib::matrix<_Tp> &ternary_points)
	// {
	// 	dlib::matrix<_Tp> quaternion_points;
	// 	quaternion_points.set_size(ternary_points.nr(), 4);
	// 	dlib::set_subm(quaternion_points, dlib::range(0, ternary_points.nr()-1), dlib::range(0, 2)) = ternary_points;
	// 	dlib::set_colm(quaternion_points, 3) = (_Tp)1.0;
	// 	return dlib::trans(quaternion_points);
	// }


	template<typename _Tp, typename _Ep>
	Matrix<_Tp, Dynamic, 1> TransPoints(
		const Matrix<_Tp, 3, 3> &r_mat, 
		const Matrix<_Tp, 3, 1> &t_vec, 
		const Matrix<_Ep, Dynamic, 1> &points) 
	{
		Matrix<_Tp, 3, 4> trans_mat;
		trans_mat << r_mat, t_vec;
		
		Matrix<_Tp, Dynamic, Dynamic> before_points = points.template cast<_Tp>();
		Map<Matrix<_Tp, Dynamic, Dynamic>> after_points(before_points.data(), points.rows() / 3, 3);
		after_points = trans_mat * after_points.rowwise().homogeneous().transpose();

		return Map<Matrix<_Tp, Dynamic, 1>>(after_points.transpose().data(), points.rows(), 1);
	}


	// template<typename _Tp, typename _Ep>
	// dlib::matrix<_Tp> transform_points(const _Tp * const ext_parm, const dlib::matrix<_Ep> &points, 
	// 								bool is_linearized = false) 
	// {
	// 	dlib::matrix<_Tp, 3, 3> R;
	// 	dlib::matrix<_Tp, 3 ,1> T;

	// 	const _Tp &yaw   = ext_parm[0];
	// 	const _Tp &pitch = ext_parm[1];
	// 	const _Tp &roll  = ext_parm[2];
	// 	const _Tp &tx    = ext_parm[3];
	// 	const _Tp &ty    = ext_parm[4];
	// 	const _Tp &tz    = ext_parm[5];
		
	// 	R = euler2matrix(yaw, pitch, roll, is_linearized);
	// 	T = tx, ty, tz;
	// 	return transform_points(R, T, points);
	// }




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
		// return dlib::sum(dlib::abs(m)) / (m.nr() * m.nc());
	}


	inline void SatisfyExtMat(Matrix3d &r_mat, Vector3d &t_vec)
	{
		CvMat *cv_trans_mat, *cv_r_mat, *cv_t_vec;
		Matrix<double, 3, 4> trans_mat;
		trans_mat << r_mat, t_vec;

		cv_trans_mat = cvCreateMat(3, 4, CV_64FC1);
		bfm_utils::EigenMat2CvMat(trans_mat, cv_trans_mat);
		
		cv_r_mat = cvCreateMatHeader(3, 3, CV_64FC1);
		cvGetCols(cv_trans_mat, cv_r_mat, 0, 3);
		cv_t_vec = cvCreateMatHeader(3, 1, CV_64FC1);
		cvGetCol(cv_trans_mat, cv_t_vec, 3);

		if( cvDet(cv_r_mat) < 0)
			cvScale(cv_trans_mat, cv_trans_mat, -1);
		double sc = cvNorm(cv_r_mat);
		CV_Assert(fabs(sc) > DBL_EPSILON);

		double u[9], v[9], w[3];
		CvMat u_mat = cvMat(3, 3, CV_64F, u);
		CvMat v_mat = cvMat(3, 3, CV_64F, v);
		CvMat w_mat = cvMat(3, 1, CV_64F, w);

		cvSVD(cv_r_mat, &w_mat, &u_mat, &v_mat, CV_SVD_MODIFY_A + CV_SVD_U_T + CV_SVD_V_T);
		cvGEMM(&u_mat, &v_mat, 1, 0, 0, cv_r_mat, CV_GEMM_A_T);

		cvScale(cv_t_vec, cv_t_vec, cvNorm(cv_r_mat)/sc);

		CvMat2EigenMat(cv_r_mat, r_mat);
		CvMat2EigenMat(cv_t_vec, t_vec);
	}
}