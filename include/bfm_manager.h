#ifndef BFM_MANAGER_H
#define BFM_MANAGER_H

#include "data.hpp"
#include "random.hpp"
#include "transform.hpp"
#include "type_utils.hpp"
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <opencv2/core/eigen.hpp>

using Eigen::Matrix;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::Dynamic;


class BaselFaceModelManager {


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
	
	BaselFaceModelManager() {}
	BaselFaceModelManager(
		std::string strModelPath,
		unsigned int nVertices,
		unsigned int nFaces,
		unsigned int nIdPcs,
		unsigned int nExprPcs,
		double *aIntParams, 
		std::string strShapeMuH5Path,
		std::string strShapeEvH5Path,
		std::string strShapePcH5Path,
		std::string strTexMuH5Path,
		std::string strTexEvH5Path,
		std::string strTexPcH5Path,
		std::string strExprMuH5Path,
		std::string strExprEvH5Path,
		std::string strExprPcH5Path,
		std::string strTriangleListH5Path,
		unsigned int nLandmarks = 0,
		std::string strLandmarkIdxPath = "");


	/*
	 *
	 */

	void genRndFace(double scale = 0.0);


	void genRndFace(double shape_scale, double tex_scale, double expr_scale);


	void genAvgFace() { this->genRndFace(0.0); }


	void genFace();


	void genLandmarkFace();


	template<typename Derived>
	Matrix<Derived, Dynamic, 1> genLandmarkFace(const Derived* const shapeCoef,const Derived* const exprCoef) const 
	{
		return this->coef2Object(shapeCoef, m_vecLandmarkShapeMu, m_matLandmarkShapePc, m_vecShapeEv, m_nIdPcs) +
			this->coef2Object(exprCoef, m_vecLandmarkExprMu, m_matLandmarkExprPc, m_vecExprEv, m_nExprPcs);
	}


	template<typename _Tp>
	Matrix<_Tp, Dynamic, 1> genLandmarkFaceByShape(const _Tp * const aShapeCoef) const 
	{
		return this->coef2Object(aShapeCoef, m_vecLandmarkShapeMu, m_matLandmarkShapePc, m_vecShapeEv, m_nIdPcs) +
			m_vecLandmarkCurrentExpr.template cast<_Tp>();
	}


	template<typename _Tp>
	Matrix<_Tp, Dynamic, 1> genLandmarkFaceByExpr(const _Tp * const aExprCoef) const 
	{
		return m_vecLandmarkCurrentShape.template cast<_Tp>() + 
			this->coef2Object(aExprCoef, m_vecLandmarkExprMu, m_matLandmarkExprPc, m_vecExprEv, m_nExprPcs);
	}


	void genTransMat();


	void genRMat();


	void genTVec();


	void genExtParams();


	void accExtParams(double *x);


	void writePly(std::string fn = "face.ply", long mode = ModelWriteMode_None) const;


	void writeFpPly(std::string fn = "fp_face.ply") const;

	
	void clrExtParams();


/*************************************************************************************************************/
/***************************************** Set & Get Function ************************************************/
/*************************************************************************************************************/


	inline const unsigned int getNIdPcs() const { return m_nIdPcs; }
	inline const unsigned int getNExprPcs() const { return m_nExprPcs; }
	inline const unsigned int getNFaces() const { return m_nFaces; }
	inline const unsigned int getNVertices() const { return m_nVertices; }
	inline const unsigned int getNLandmarks() const { return m_nLandmarks; }
	
	inline double *getMutableShapeCoef() { return m_aShapeCoef; }
	inline double *getMutableTexCoef() { return m_aTexCoef; }
	inline double *getMutableExprCoef() { return m_aExprCoef; }
	inline double *getMutableExtParams() { return m_aExtParams; }
	inline double *getMutableIntParams() { return m_aIntParams; }
	inline const double *getExtParams() const { return m_aExtParams; }
	inline const double *getIntParams() const { return m_aIntParams; }

	inline const Matrix3d getMatR() const { return m_matR; }
	inline const Vector3d getVecT() const { return m_vecT; }

	inline const double getFx() const { return m_aIntParams[0]; }
	inline const double getFy() const { return m_aIntParams[1]; }
	inline const double getCx() const { return m_aIntParams[2]; }
	inline const double getCy() const { return m_aIntParams[3]; }
	inline const double getYaw() const { return m_aExtParams[0]; }
	inline const double getPitch() const { return m_aExtParams[1]; }
	inline const double getRoll() const { return m_aExtParams[2]; }
	inline const double getTx() const { return m_aExtParams[3]; }
	inline const double getTy() const { return m_aExtParams[4]; }
	inline const double getTz() const { return m_aExtParams[5]; }
	inline void setYaw(double yaw)     { m_aExtParams[0] = yaw;   genRMat();}
	inline void setPitch(double pitch) { m_aExtParams[1] = pitch; genRMat();}
	inline void setRoll(double roll)   { m_aExtParams[2] = roll;  genRMat();}
	inline void setRotation(double yaw, double pitch, double roll) 
	{
		setYaw(yaw); 
		setPitch(pitch); 
		setRoll(roll);
	}
	inline void setTx(double tx) { m_aExtParams[3] = tx; m_vecT(0) = tx; }
	inline void setTy(double ty) { m_aExtParams[4] = ty; m_vecT(1) = ty; }
	inline void setTz(double tz) { m_aExtParams[5] = tz; m_vecT(2) = tz; }
	inline void setMatR(const Matrix3d &r_mat) { m_matR = r_mat; }
	inline void setMatR(const cv::Mat &r_mat) { cv::cv2eigen(r_mat, m_matR); }
	inline void setMatR(CvMat *r_mat)
	{
		m_matR(0, 0) = cvmGet(r_mat, 0, 0); m_matR(0, 1) = cvmGet(r_mat, 0, 1); m_matR(0, 2) = cvmGet(r_mat, 0, 2);
		m_matR(1, 0) = cvmGet(r_mat, 1, 0); m_matR(1, 1) = cvmGet(r_mat, 1, 1); m_matR(1, 2) = cvmGet(r_mat, 1, 2);
		m_matR(2, 0) = cvmGet(r_mat, 2, 0); m_matR(2, 1) = cvmGet(r_mat, 2, 1); m_matR(2, 2) = cvmGet(r_mat, 2, 2);		
	}
	inline void setVecT(const Vector3d &t_vec) { m_vecT = t_vec; }
	inline void setVecT(const cv::Mat &t_vec) { cv::cv2eigen(t_vec, m_vecT); }
	inline void setVecT(CvMat *t_vec)
	{
		m_vecT(0) = cvmGet(t_vec, 0, 0);
		m_vecT(1) = cvmGet(t_vec, 1, 0);
		m_vecT(2) = cvmGet(t_vec, 2, 0);		
	}

	inline const VectorXd &getCurrentShape() const { return m_vecCurrentShape; }
	inline const VectorXd &getCurrentTex() const { return m_vecCurrentTex; }
	inline VectorXd getStdTex() const 
	{
		VectorXd res(m_nVertices * 3);
		for(auto i = 0; i < res.size(); i++)
			res(i) = m_vecTexMu(i) / 255.0;
		return res;
	} 
	inline const VectorXd &getCurrentExpr() const { return m_vecCurrentExpr; }
	inline const VectorXd &getCurrentBlendshape() const { return m_vecCurrentBlendshape; }
	inline const VectorXd &getLandmarkCurrentBlendshape() const { return m_vecLandmarkCurrentBlendshape; }
	VectorXd getLandmarkCurrentBlendshapeTransformed() { return bfm_utils::TransPoints(m_matR, m_vecT, m_vecLandmarkCurrentBlendshape); }
	VectorXd getCurrentBlendshapeTransformed() { return bfm_utils::TransPoints(m_matR, m_vecT, m_vecCurrentBlendshape); }
	inline const Matrix<unsigned int, Dynamic, 1> &getTriangleList() const 	{ return m_vecTriangleList; }


/*************************************************************************************************************/
/************************************** Print Function (for Debug) *******************************************/
/*************************************************************************************************************/


	inline void check() const
	{
		BFM_DEBUG("check data\n");
		BFM_DEBUG("	(1) shape mu: \n");
		BFM_DEBUG("		Yours:   %lf\n", m_vecShapeMu(0));
		BFM_DEBUG("		Ref: -57239 42966 80410\n\n");
		BFM_DEBUG("	(2) shape ev: \n");
		BFM_DEBUG("		Yours:   %lf, %lf\n", m_vecShapeEv(0), m_vecShapeEv(1));
		BFM_DEBUG("		Ref: 884340 555880\n\n");
		BFM_DEBUG("	(3) shape pc: \n");
		BFM_DEBUG("		Yours:   %lf\n", m_matShapePc(0, 0));
		BFM_DEBUG("		Ref: -0.0024\n\n");
		BFM_DEBUG("	(4) texture mu: \n");
		BFM_DEBUG("		Yours:   %lf\n", m_vecTexMu(0));
		BFM_DEBUG("		Ref: 182.8750 135.0400 107.1400\n\n");
		BFM_DEBUG("	(5) texture ev: \n");
		BFM_DEBUG("		Yours:   %lf, %lf\n", m_vecTexEv(0), m_vecTexEv(1));
		BFM_DEBUG("		Ref: 4103.2 2024.1\n\n");
		BFM_DEBUG("	(6) texture pc: \n");
		BFM_DEBUG("		Yours:   %lf\n", m_matTexPc(0, 0));
		BFM_DEBUG("		Ref: -0.0028\n\n");
		BFM_DEBUG("	(7) expression mu: \n");
		BFM_DEBUG("		Yours:   %lf\n", m_vecExprMu(0));
		BFM_DEBUG("		Ref: 182.8750 135.0400 107.1400\n\n");
		BFM_DEBUG("	(8) expression ev: \n");
		BFM_DEBUG("		Yours:   %lf, %lf\n", m_vecExprEv(0), m_vecExprEv(1));
		BFM_DEBUG("		Ref: 4103.2 2024.1\n\n");
		BFM_DEBUG("	(9) expression pc: \n");
		BFM_DEBUG("		Yours:   %lf\n", m_matExprPc(0, 0));
		BFM_DEBUG("		Ref: -0.0028\n\n");
		BFM_DEBUG("	(10) triangle list: \n");
		BFM_DEBUG("		Yours:   %u, %u\n", m_vecTriangleList(0), m_vecTriangleList(1));
		BFM_DEBUG("		Ref: -0.0028\n\n");
	}

	void printFpShapeMu() const { BFM_DEBUG("fp - shape mu: \n%s", bfm_utils::NumMat2Str(m_vecLandmarkShapeMu).c_str()); }
	void printFpShapePc() const { BFM_DEBUG("fp - shape pc: \n%s", bfm_utils::NumMat2Str(m_matLandmarkShapePc).c_str()); }
	void printShapeEv() const { BFM_DEBUG("shape variance: \n%s", bfm_utils::NumMat2Str(m_vecShapeEv).c_str()); }

	inline void printExtParams() const 
	{
		BFM_DEBUG("yaw: %lf ", m_aExtParams[0]);
		BFM_DEBUG("(%lf')\n", (m_aExtParams[0] * 180.0 / M_PI));
		BFM_DEBUG("pitch: %lf ", m_aExtParams[1]);
		BFM_DEBUG("(%lf')\n", (m_aExtParams[1] * 180.0 / M_PI));
		BFM_DEBUG("roll: %lf ", m_aExtParams[2]);
		BFM_DEBUG("(%lf')\n", (m_aExtParams[2] * 180.0 / M_PI));
		BFM_DEBUG("tx: %lf\n", m_aExtParams[3]);
		BFM_DEBUG("ty: %lf\n", m_aExtParams[4]);
		BFM_DEBUG("tz: %lf\n", m_aExtParams[5]);
	}

	inline void printIntParams() const
	{
		BFM_DEBUG("fx: %lf\n", m_aIntParams[0]);
		BFM_DEBUG("fy: %lf\n", m_aIntParams[1]);
		BFM_DEBUG("cx: %lf\n", m_aIntParams[2]);
		BFM_DEBUG("cy: %lf\n", m_aIntParams[3]);
	}
	
	void printShapeCoef() const 
	{ 
		BFM_DEBUG("shape coef:\n");
		bfm_utils::PrintArr(m_aShapeCoef, m_nIdPcs);
	}
	
	void printExprCoef() const 
	{ 
		BFM_DEBUG("expression coef:\n");
		bfm_utils::PrintArr(m_aExprCoef, m_nExprPcs);
	}

	inline void printRMat() const { BFM_DEBUG("R: \n%s", bfm_utils::NumMat2Str(m_matR).c_str()); }
	inline void printTVec() const { BFM_DEBUG("T: \n%s", bfm_utils::NumMat2Str(m_vecT).c_str()); }


private:


	void alloc();


	bool load();


	void extractLandmarks();


	template<typename Derived>
	Matrix<Derived, Dynamic, 1> coef2Object(const Derived *const aCoef, 
		const VectorXd &vecMu, const MatrixXd &matPc, const VectorXd &vecEv, unsigned int nLength) const
	{
		assert(aCoef != nullptr);
		assert(nLength >= 0);

		Matrix<Derived, Dynamic, 1> tmpCoef(nLength);
		for(int i = 0; i < nLength; i++)
			tmpCoef(i) = aCoef[i];

		Matrix<Derived, Dynamic, 1> tmpMu = vecMu.cast<Derived>();
		Matrix<Derived, Dynamic, 1> tmpEv = vecEv.cast<Derived>();
		Matrix<Derived, Dynamic, Dynamic> tmpPc = matPc.cast<Derived>();
		return tmpMu + tmpPc * tmpCoef.cwiseProduct(tmpEv);
	}


	std::string m_strModelPath;
	std::string m_strLandmarkIdxPath;
	std::string m_strShapeMuH5Path;
	std::string m_strShapeEvH5Path;
	std::string m_strShapePcH5Path;
	std::string m_strTexMuH5Path;
	std::string m_strTexEvH5Path;
	std::string m_strTexPcH5Path;
	std::string m_strExprMuH5Path;
	std::string m_strExprEvH5Path;
	std::string m_strExprPcH5Path;
	std::string m_strTriangleListH5Path;

	unsigned int m_nVertices;
	unsigned int m_nFaces;
	unsigned int m_nIdPcs;
	unsigned int m_nExprPcs;

	/* ZYX - euler angle */
	/* yaw:   rotate around z axis */
	/* pitch: rotate around y axis */
    /* roll:  rotate around x axis */
	Matrix3d m_matR;
	Vector3d m_vecT;
	double m_aExtParams[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };	/* yaw pitch roll tx ty tz */
	double m_aIntParams[4] = { 0.0, 0.0, 0.0, 0.0 };	/* fx fy cx cy */

	double *m_aShapeCoef;
	VectorXd m_vecShapeMu;
	VectorXd m_vecShapeEv;
	MatrixXd m_matShapePc;

	double *m_aTexCoef;
	VectorXd m_vecTexMu;
	VectorXd m_vecTexEv;
	MatrixXd m_matTexPc;

	double *m_aExprCoef;
	VectorXd m_vecExprMu;
	VectorXd m_vecExprEv;
	MatrixXd m_matExprPc;

	Matrix<unsigned int, Dynamic, 1> m_vecTriangleList;	/* triangle list */

	VectorXd m_vecCurrentShape;
	VectorXd m_vecCurrentTex;
	VectorXd m_vecCurrentExpr;
	VectorXd m_vecCurrentBlendshape;

	bool m_bUseLandmark;
	unsigned int m_nLandmarks;
	std::vector<int> m_vecLandmarkIndices; 
	VectorXd m_vecLandmarkShapeMu;
	MatrixXd m_matLandmarkShapePc;
	VectorXd m_vecLandmarkExprMu;
	MatrixXd m_matLandmarkExprPc;
	VectorXd m_vecLandmarkCurrentShape;
	VectorXd m_vecLandmarkCurrentExpr;
	VectorXd m_vecLandmarkCurrentBlendshape;

public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};


#endif // BFM_MANAGER_H