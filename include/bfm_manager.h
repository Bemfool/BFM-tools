#pragma once
#include "data.hpp"
#include "random.hpp"
#include "transform.hpp"
#include "type_utils.hpp"
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>


using Eigen::Matrix;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::Dynamic;


class CBaselFaceModelManager {


public:

	/*
	 * Constructor
	 * ***********************************************************************************************
	 * Parameters:
	 * 		bfm_h5_path: H5 file storing Basel Face Model;
	 * 		num_vertice: Vertice number;
	 * 		num_face: Face number;
	 * 		num_id_pc: Identity (shape and texture) principle component number;
	 *		num_expr_pc: Expression princile component number;
	 *		int_params: Camera intrinsic parameters, array of four double number (fx, fy, cx, cy); 
	 *		shape_mu_h5_path: H5 database path of average shape;
	 *		shape_ev_h5_path: H5 database path of shape variance;
	 *		shape_pc_h5_path: H5 database path of shape princile component basis;
	 *	    tex_mu_h5_path: H5 database path of average texture;
	 *		tex_ev_h5_path: H5 database path of texture variance;
	 *		tex_pc_h5_path: H5 database path of texture princile component basis;
	 *	    expr_mu_h5_path: H5 database path of average expression;
	 *		expr_ev_h5_path: H5 database path of expression variance;
	 *		expr_pc_h5_path: H5 database path of expression princile component basis;
	 *		tl_h5_path: H5 database path of triangle list;
	 *		num_fp: Landmark number. Setting = 0 means that landmarks are not needed;
	 *		fp_idx_path: File path storing index of landmarks.
	 * ***********************************************************************************************
	 * Note:
	 * 		Input details see examples in README.md.
	 */
	
	CBaselFaceModelManager() {}
	CBaselFaceModelManager(
		std::string bfm_h5_path,
		unsigned int num_vertice,
		unsigned int num_face,
		unsigned int num_id_pc,
		unsigned int num_expr_pc,
		double *int_params, 
		std::string shape_mu_h5_path,
		std::string shape_ev_h5_path,
		std::string shape_pc_h5_path,
		std::string tex_mu_h5_path,
		std::string tex_ev_h5_path,
		std::string tex_pc_h5_path,
		std::string expr_mu_h5_path,
		std::string expr_ev_h5_path,
		std::string expr_pc_h5_path,
		std::string tl_h5_path,
		unsigned int num_fp = 0,
		std::string fp_idx_path = "");


	/*
	 *
	 */

	void GenRndFace(double scale = 0.0);


	void GenRndFace(double shape_scale, double tex_scale, double expr_scale);


	void GenAvgFace() { GenRndFace(0.0); }


	void GenFace();


	void GenFpFace();


	template<typename T>
	Matrix<T, Dynamic, Dynamic> GenFpFace(const T * const shape_coef,const T * const expr_coef) const 
	{
		Matrix<T, Dynamic, Dynamic> fp_current_shape = Coef2Object(shape_coef, fp_shape_mu_, fp_shape_pc_, shape_ev_, num_id_pc_);
		Matrix<T, Dynamic, Dynamic> fp_current_expr = Coef2Object(expr_coef, fp_expr_mu_, fp_expr_pc_, expr_ev_, num_expr_pc_);
		Matrix<T, Dynamic, Dynamic> fp_current_blendshape = fp_current_shape_ + fp_current_expr_;	
		return fp_current_blendshape_;
	}


	template<typename T>
	Matrix<T, Dynamic, Dynamic> GenFpFaceByShape(const T * const shape_coef) const 
	{
		Matrix<T, Dynamic, Dynamic> fp_current_shape = coef2object(shape_coef, fp_shape_mu_, fp_shape_pc_, shape_ev_, num_id_pc_);
		Matrix<T, Dynamic, Dynamic> fp_current_expr = fp_current_expr_.cast<T>();
		Matrix<T, Dynamic, Dynamic> fp_current_blendshape = fp_current_shape + fp_current_expr;	
		return fp_current_blendshape;		
	}


	template<typename T>
	Matrix<T, Dynamic, Dynamic> GenFpFaceByExpr(const T * const expr_coef) const 
	{
		Matrix<T, Dynamic, Dynamic> fp_current_shape = fp_current_shape.template cast<T>();
		Matrix<T, Dynamic, Dynamic> fp_current_expr = Coef2Object(expr_coef, fp_expr_mu_, fp_expr_pc_, expr_ev_, num_expr_pc_);
		Matrix<T, Dynamic, Dynamic> fp_current_blendshape = fp_current_shape + fp_current_expr;	
		return fp_current_blendshape;		
	}


	void GenTransMat();


	void GenRMat();


	void GenTVec();


	void GenExtParams();


	void AccExtParams(double *x);


	void WritePly(std::string fn = "face.ply", model_write_mode mode = NONE_MODE) const;


	void WriteFpPly(std::string fn = "fp_face.ply") const;

	
	void ClrExtParams();


	int get_dlib_fp_idx(int idx) const { return fp_map_[idx]; }


/*************************************************************************************************************/
/***************************************** Set & Get Function ************************************************/
/*************************************************************************************************************/


	inline const unsigned int get_num_id_pc_() const { return num_id_pc_; }
	inline const unsigned int get_num_expr_pc() const { return num_expr_pc_; }
	inline const unsigned int get_num_face_() const { return num_face_; }
	inline const unsigned int get_num_vertice_() const { return num_vertice_; }
	inline const unsigned int get_num_fp() const { return num_fp_; }
	
	inline double *get_mutable_shape_coef() { return shape_coef_; }
	inline double *get_mutable_tex_coef() { return tex_coef_; }
	inline double *get_mutable_expr_coef() { return expr_coef_; }
	inline double *get_mutable_ext_params() { return ext_params_; }
	inline double *get_mutable_int_params() { return int_params_; }
	inline const double *get_ext_params() const { return ext_params_; }
	inline const double *get_int_params() const { return int_params_; }

	inline const Matrix3d get_r_mat() const { return r_mat_; }
	inline const Vector3d get_t_vec() const { return t_vec_; }

	inline const double get_fx() const { return int_params_[0]; }
	inline const double get_fy() const { return int_params_[1]; }
	inline const double get_cx() const { return int_params_[2]; }
	inline const double get_cy() const { return int_params_[3]; }
	inline const double get_yaw() const { return ext_params_[0]; }
	inline const double get_pitch() const { return ext_params_[1]; }
	inline const double get_roll() const { return ext_params_[2]; }
	inline const double get_tx() const { return ext_params_[3]; }
	inline const double get_ty() const { return ext_params_[4]; }
	inline const double get_tz() const { return ext_params_[5]; }
	inline void set_yaw(double yaw)     { ext_params_[0] = yaw;   GenRMat();}
	inline void set_pitch(double pitch) { ext_params_[1] = pitch; GenRMat();}
	inline void set_roll(double roll)   { ext_params_[2] = roll;  GenRMat();}
	inline void set_rotation(double yaw, double pitch, double roll) 
	{
		set_yaw(yaw); 
		set_pitch(pitch); 
		set_roll(roll);
	}
	inline void set_tx(double tx) { ext_params_[3] = tx; t_vec_(0) = tx; }
	inline void set_ty(double ty) { ext_params_[4] = ty; t_vec_(1) = ty; }
	inline void set_tz(double tz) { ext_params_[5] = tz; t_vec_(2) = tz; }
	inline void set_r_mat(const Matrix3d &r_mat) { r_mat_ = r_mat; }
	inline void set_r_mat(const cv::Mat &r_mat) { cv::cv2eigen(r_mat, r_mat_); }
	inline void set_r_mat(CvMat *r_mat)
	{
		r_mat_(0, 0) = cvmGet(r_mat, 0, 0); r_mat_(0, 1) = cvmGet(r_mat, 0, 1); r_mat_(0, 2) = cvmGet(r_mat, 0, 2);
		r_mat_(1, 0) = cvmGet(r_mat, 1, 0); r_mat_(1, 1) = cvmGet(r_mat, 1, 1); r_mat_(1, 2) = cvmGet(r_mat, 1, 2);
		r_mat_(2, 0) = cvmGet(r_mat, 2, 0); r_mat_(2, 1) = cvmGet(r_mat, 2, 1); r_mat_(2, 2) = cvmGet(r_mat, 2, 2);		
	}
	inline void set_t_vec(const Vector3d &t_vec) { t_vec_ = t_vec; }
	inline void set_t_vec(const cv::Mat &t_vec) { cv::cv2eigen(t_vec, t_vec_); }
	inline void set_t_vec(CvMat *t_vec)
	{
		t_vec_(0) = cvmGet(t_vec, 0, 0);
		t_vec_(1) = cvmGet(t_vec, 1, 0);
		t_vec_(2) = cvmGet(t_vec, 2, 0);		
	}

	inline const VectorXd &get_current_shape() const { return current_shape_; }
	inline const VectorXd &get_current_tex() const { return current_tex_; }
	inline VectorXd get_std_tex() const 
	{
		VectorXd res(num_vertice_ * 3);
		for(auto i = 0; i < res.size(); i++)
			res(i) = tex_mu_(i) / 255.0;
		return res;
	} 
	inline const VectorXd &get_current_expr() const { return current_expr_; }
	inline const VectorXd &get_current_blendshape() const { return current_blendshape_; }
	inline const VectorXd &get_fp_current_blendshape() const { return fp_current_blendshape_; }
	VectorXd get_fp_current_blendshape_transformed() { return bfm_utils::TransPoints(r_mat_, t_vec_, fp_current_blendshape_); }
	VectorXd get_current_blendshape_transformed() { return bfm_utils::TransPoints(r_mat_, t_vec_, current_blendshape_); }
	inline const VectorXd &get_tl() const 	{ return tl_; }


/*************************************************************************************************************/
/************************************** Print Function (for Debug) *******************************************/
/*************************************************************************************************************/


#ifndef BFM_SHUT_UP
	inline void Check() const
	{
		BFM_DEBUG("check data\n");
		BFM_DEBUG("	(1) shape mu: \n");
		BFM_DEBUG("		Yours:   %lf\n", shape_mu_(0));
		BFM_DEBUG("		Ref: -57239 42966 80410\n\n");
		BFM_DEBUG("	(2) shape ev: \n");
		BFM_DEBUG("		Yours:   %lf, %lf\n", shape_ev_(0), shape_ev_(1));
		BFM_DEBUG("		Ref: 884340 555880\n\n");
		BFM_DEBUG("	(3) shape pc: \n");
		BFM_DEBUG("		Yours:   %lf\n", shape_pc_(0, 0));
		BFM_DEBUG("		Ref: -0.0024\n\n");
		BFM_DEBUG("	(4) texture mu: \n");
		BFM_DEBUG("		Yours:   %lf\n", tex_mu_(0));
		BFM_DEBUG("		Ref: 182.8750 135.0400 107.1400\n\n");
		BFM_DEBUG("	(5) texture ev: \n");
		BFM_DEBUG("		Yours:   %lf, %lf\n", tex_ev_(0), tex_ev_(1));
		BFM_DEBUG("		Ref: 4103.2 2024.1\n\n");
		BFM_DEBUG("	(6) texture pc: \n");
		BFM_DEBUG("		Yours:   %lf\n", tex_pc_(0, 0));
		BFM_DEBUG("		Ref: -0.0028\n\n");
		BFM_DEBUG("	(7) expression mu: \n");
		BFM_DEBUG("		Yours:   %lf\n", expr_mu_(0));
		BFM_DEBUG("		Ref: 182.8750 135.0400 107.1400\n\n");
		BFM_DEBUG("	(8) expression ev: \n");
		BFM_DEBUG("		Yours:   %lf, %lf\n", expr_ev_(0), expr_ev_(1));
		BFM_DEBUG("		Ref: 4103.2 2024.1\n\n");
		BFM_DEBUG("	(9) expression pc: \n");
		BFM_DEBUG("		Yours:   %lf\n", expr_pc_(0, 0));
		BFM_DEBUG("		Ref: -0.0028\n\n");
		BFM_DEBUG("	(10) triangle list: \n");
		BFM_DEBUG("		Yours:   %lf, %lf\n", tl_(0), tl_(1));
		BFM_DEBUG("		Ref: -0.0028\n\n");
	}

	void PrintFpShapeMu() const { BFM_DEBUG("fp - shape mu: \n%s", bfm_utils::NumMat2Str(fp_shape_mu_).c_str()); }
	void PrintFpShapePc() const { BFM_DEBUG("fp - shape pc: \n%s", bfm_utils::NumMat2Str(fp_shape_pc_).c_str()); }
	void PrintShapeEv() const { BFM_DEBUG("shape variance: \n%s", bfm_utils::NumMat2Str(shape_ev_).c_str()); }

	inline void PrintExtParams() const 
	{
		BFM_DEBUG("yaw: %lf ", ext_params_[0]);
		BFM_DEBUG("(%lf')\n", (ext_params_[0] * 180.0 / M_PI));
		BFM_DEBUG("pitch: %lf ", ext_params_[1]);
		BFM_DEBUG("(%lf')\n", (ext_params_[1] * 180.0 / M_PI));
		BFM_DEBUG("roll: %lf ", ext_params_[2]);
		BFM_DEBUG("(%lf')\n", (ext_params_[2] * 180.0 / M_PI));
		BFM_DEBUG("tx: %lf\n", ext_params_[3]);
		BFM_DEBUG("ty: %lf\n", ext_params_[4]);
		BFM_DEBUG("tz: %lf\n", ext_params_[5]);
	}

	inline void PrintIntParams() const
	{
		BFM_DEBUG("fx: %lf\n", int_params_[0]);
		BFM_DEBUG("fy: %lf\n", int_params_[1]);
		BFM_DEBUG("cx: %lf\n", int_params_[2]);
		BFM_DEBUG("cy: %lf\n", int_params_[3]);
	}
	
	void PrintShapeCoef() const 
	{ 
		BFM_DEBUG("shape coef:\n");
		bfm_utils::PrintArr(shape_coef_, num_id_pc_);
	}
	
	void PrintExprCoef() const 
	{ 
		BFM_DEBUG("expression coef:\n");
		bfm_utils::PrintArr(expr_coef_, num_expr_pc_);
	}

	inline void PrintRMat() const { BFM_DEBUG("R: \n%s", bfm_utils::NumMat2Str(r_mat_).c_str()); }
	inline void PrintTVec() const { BFM_DEBUG("T: \n%s", bfm_utils::NumMat2Str(t_vec_).c_str()); }


#else

	inline void Check() const { }
	inline void PrintFpShapeMu() const { }
	inline void PrintFpShapePc() const { }
	inline void PrintShapeEv() const { }
	inline void PrintExtParams() const { }
	inline void PrintIntParams() const { }
	inline void PrintShapeCoef() const { }
	inline void PrintExprCoef() const { }
	inline void PrintRMat() const { }
	inline void PrintTVec() const { }


#endif


private:


	void Alloc();


	bool Load();


	void ExtractFp();


	template<typename _Tp>
	Matrix<_Tp, Dynamic, Dynamic> Coef2Object(
		const _Tp *const &coef, 
		const VectorXd &mu, 
		const MatrixXd &pc, 
		const VectorXd &ev, 
		int len) const 
	{ 
		Matrix<_Tp, Dynamic, 1> tmp_coef(len);
		for(int i=0; i<len; i++)
			tmp_coef(i) = coef[i];

		Matrix<_Tp, Dynamic, 1> tmp_mu = mu.cast<_Tp>();
		Matrix<_Tp, Dynamic, 1> tmp_ev = ev.cast<_Tp>();
		Matrix<_Tp, Dynamic, Dynamic> tmp_pc = pc.cast<_Tp>();
		return tmp_mu + tmp_pc * tmp_coef.cwiseProduct(tmp_ev);
	}


	std::string bfm_h5_path_;
	std::string fp_idx_path_;
	std::string shape_mu_h5_path_;
	std::string shape_ev_h5_path_;
	std::string shape_pc_h5_path_;
	std::string tex_mu_h5_path_;
	std::string tex_ev_h5_path_;
	std::string tex_pc_h5_path_;
	std::string expr_mu_h5_path_;
	std::string expr_ev_h5_path_;
	std::string expr_pc_h5_path_;
	std::string tl_h5_path_;

	unsigned int num_vertice_;
	unsigned int num_face_;
	unsigned int num_id_pc_;
	unsigned int num_expr_pc_;

	/* ZYX - euler angle */
	/* yaw:   rotate around z axis */
	/* pitch: rotate around y axis */
    /* roll:  rotate around x axis */
	Matrix3d r_mat_;
	Vector3d t_vec_;
	double ext_params_[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };	/* yaw pitch roll tx ty tz */
	double int_params_[4] = { 0.0, 0.0, 0.0, 0.0 };	/* fx fy cx cy */

	double *shape_coef_;
	VectorXd shape_mu_;
	VectorXd shape_ev_;
	MatrixXd shape_pc_;

	double *tex_coef_;
	VectorXd tex_mu_;
	VectorXd tex_ev_;
	MatrixXd tex_pc_;

	double *expr_coef_;
	VectorXd expr_mu_;
	VectorXd expr_ev_;
	MatrixXd expr_pc_;

	VectorXd tl_;	/* triangle list */

	VectorXd current_shape_;
	VectorXd current_tex_;
	VectorXd current_expr_;
	VectorXd current_blendshape_;

	/* We use fp(feature points) as the abbreviation of faicial landmarks/fiducial points */
	bool use_fp_;
	unsigned int num_fp_;
	std::vector<int> fp_idx_; 
	std::vector<int> fp_map_;
	VectorXd fp_shape_mu_;
	MatrixXd fp_shape_pc_;
	VectorXd fp_expr_mu_;
	MatrixXd fp_expr_pc_;
	VectorXd fp_current_shape_;
	VectorXd fp_current_expr_;
	VectorXd fp_current_blendshape_;
};