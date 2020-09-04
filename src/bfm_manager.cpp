#include "bfm_manager.h"

CBaselFaceModelManager::CBaselFaceModelManager(
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
	unsigned int num_fp,
	std::string fp_idx_path) :
	bfm_h5_path_(bfm_h5_path),
	num_vertice_(num_vertice),
	num_face_(num_face),
	num_id_pc_(num_id_pc),
	num_expr_pc_(num_expr_pc),
	num_fp_(num_fp),
	fp_idx_path_(fp_idx_path),
	shape_mu_h5_path_(shape_mu_h5_path),
	shape_ev_h5_path_(shape_ev_h5_path),
	shape_pc_h5_path_(shape_pc_h5_path),
	tex_mu_h5_path_(tex_mu_h5_path),
	tex_ev_h5_path_(tex_ev_h5_path),
	tex_pc_h5_path_(tex_pc_h5_path),
	expr_mu_h5_path_(expr_mu_h5_path),
	expr_ev_h5_path_(expr_ev_h5_path),
	expr_pc_h5_path_(expr_pc_h5_path),
	tl_h5_path_(tl_h5_path) 
{
	for(unsigned int i = 0; i < 4; i++)
		int_params_[i] = int_params[i];
	if(!num_fp)
		use_fp_ = false;

	Alloc();
	Load();
	if(use_fp_)
	{
		ExtractFp();
		GenFpFace();
	}
	GenAvgFace();
}


void CBaselFaceModelManager::Alloc() {
	shape_coef_ = new double[num_id_pc_];
	fill(shape_coef_, shape_coef_ + num_id_pc_, 0.f);
	shape_mu_.resize(num_vertice_ * 3);
	shape_ev_.resize(num_id_pc_);
	shape_pc_.resize(num_vertice_ * 3, num_id_pc_);

	tex_coef_ = new double[num_id_pc_];
	fill(tex_coef_, tex_coef_ + num_id_pc_, 0.f);
	tex_mu_.resize(num_vertice_ * 3);
	tex_ev_.resize(num_id_pc_);
	tex_pc_.resize(num_vertice_ * 3, num_id_pc_);

	expr_coef_ = new double[num_expr_pc_];
	fill(expr_coef_, expr_coef_ + num_expr_pc_, 0.f);
	expr_mu_.resize(num_vertice_ * 3);
	expr_ev_.resize(num_expr_pc_);
	expr_pc_.resize(num_vertice_ * 3, num_expr_pc_);

	tl_.resize(num_face_ * 3);

	current_shape_.resize(num_vertice_ * 3);
	current_tex_.resize(num_vertice_ * 3);
	current_expr_.resize(num_vertice_ * 3);
	current_blendshape_.resize(num_vertice_ * 3);

	if (use_fp_) {
		fp_idx_.resize(num_fp_);
		fp_shape_mu_.resize(num_fp_ * 3);
		fp_shape_pc_.resize(num_fp_ * 3, num_id_pc_);
		fp_expr_mu_.resize(num_fp_ * 3);
		fp_expr_pc_.resize(num_fp_ * 3, num_expr_pc_);
	}
}


bool CBaselFaceModelManager::Load() {
	float *shape_mu_raw = new float[num_vertice_ * 3];
	float *shape_ev_raw = new float[num_id_pc_];
	float *shape_pc_raw = new float[num_vertice_ * 3 * num_id_pc_];
	float *tex_mu_raw = new float[num_vertice_ * 3];
	float *tex_ev_raw = new float[num_id_pc_];
	float *tex_pc_raw = new float[num_vertice_ * 3 * num_id_pc_];
	float *expr_mu_raw = new float[num_vertice_ * 3];
	float *expr_ev_raw = new float[num_expr_pc_];
	float *expr_pc_raw = new float[num_vertice_ * 3 * num_expr_pc_];
	unsigned int *tl_raw = new unsigned int[num_face_ * 3];

	H5File file(bfm_h5_path_, H5F_ACC_RDONLY);
	LOAD_H5_MODEL(shape_mu_, shape_mu_h5_path_, PredType::NATIVE_FLOAT);
	LOAD_H5_MODEL(shape_ev_, shape_ev_h5_path_, PredType::NATIVE_FLOAT);
	LOAD_H5_MODEL(shape_pc_, shape_pc_h5_path_, PredType::NATIVE_FLOAT);

	LOAD_H5_MODEL(tex_mu_, tex_mu_h5_path_, PredType::NATIVE_FLOAT);
	LOAD_H5_MODEL(tex_ev_, tex_ev_h5_path_, PredType::NATIVE_FLOAT);
	LOAD_H5_MODEL(tex_pc_, tex_pc_h5_path_, PredType::NATIVE_FLOAT);

	LOAD_H5_MODEL(expr_mu_, expr_mu_h5_path_, PredType::NATIVE_FLOAT);
	LOAD_H5_MODEL(expr_ev_, expr_ev_h5_path_, PredType::NATIVE_FLOAT);
	LOAD_H5_MODEL(expr_pc_, expr_pc_h5_path_, PredType::NATIVE_FLOAT);

	LOAD_H5_MODEL(tl_, tl_h5_path_, PredType::NATIVE_UINT32);

	file.close();
	shape_mu_ = shape_mu_ * 1000.0;

	if(use_fp_)
	{
		ifstream in(fp_idx_path_, std::ios::in);
		if (!in) 
		{
			BFM_DEBUG("[ERROR] Can't open %s.", fp_idx_path_.c_str());
			return false;
		}

		for (unsigned int i = 0; i < num_fp_; i++) 
		{
			int tmp_idx, idx;
			in >> tmp_idx;
			fp_idx_[i] = tmp_idx - 1;
		}
	}
	
	return true;
}


void CBaselFaceModelManager::ExtractFp() 
{
	for(unsigned int i = 0; i < num_fp_; i++) 
	{
		unsigned int idx = fp_idx_[i];
		fp_shape_mu_(i*3) = shape_mu_(idx*3);
		fp_shape_mu_(i*3+1) = shape_mu_(idx*3+1);
		fp_shape_mu_(i*3+2) = shape_mu_(idx*3+2);
		fp_expr_mu_(i*3) = expr_mu_(idx*3);
		fp_expr_mu_(i*3+1) = expr_mu_(idx*3+1);
		fp_expr_mu_(i*3+2) = expr_mu_(idx*3+2);

		for(unsigned int j = 0; j < num_id_pc_; j++) 
		{
			fp_shape_pc_(i*3, j) = shape_pc_(idx*3, j);
			fp_shape_pc_(i*3+1, j) = shape_pc_(idx*3+1, j);
			fp_shape_pc_(i*3+2, j) = shape_pc_(idx*3+2, j);	
		}

		for(unsigned int j = 0; j < num_expr_pc_; j++) 
		{
			fp_expr_pc_(i*3, j) = expr_pc_(idx*3, j);
			fp_expr_pc_(i*3+1, j) = expr_pc_(idx*3+1, j);
			fp_expr_pc_(i*3+2, j) = expr_pc_(idx*3+2, j);
		}
	}
}



void CBaselFaceModelManager::GenRndFace(double scale) 
{
	BFM_DEBUG("init random numbers (using the same scale) - ");
	shape_coef_ = randn(num_id_pc_, scale);
	tex_coef_   = randn(num_id_pc_, scale);
	expr_coef_  = randn(num_expr_pc_, scale);
	BFM_DEBUG("success\n");
	GenFace();
}


void CBaselFaceModelManager::GenRndFace(double shape_scale, double tex_scale, double expr_scale) 
{
	BFM_DEBUG("init random numbers (using different scales) - ");
	shape_coef_ = randn(num_id_pc_, shape_scale);
	tex_coef_   = randn(num_id_pc_, tex_scale);
	expr_coef_  = randn(num_expr_pc_, expr_scale);
	BFM_DEBUG("success\n");
	GenFace();
}


void CBaselFaceModelManager::GenFace() 
{
	BFM_DEBUG("generate face - ");
	current_shape_ = Coef2Object(shape_coef_, shape_mu_, shape_pc_, shape_ev_, num_id_pc_);
	current_tex_   = Coef2Object(tex_coef_, tex_mu_, tex_pc_, tex_ev_, num_id_pc_);
	current_expr_  = Coef2Object(expr_coef_, expr_mu_, expr_pc_, expr_ev_, num_expr_pc_);
	current_blendshape_ = current_shape_ + current_expr_;
	BFM_DEBUG("success\n");
}


void CBaselFaceModelManager::GenFpFace()  
{
	BFM_DEBUG("generate feature point face - ");
	fp_current_shape_ = Coef2Object(shape_coef_, fp_shape_mu_, fp_shape_pc_, shape_ev_, num_id_pc_);
	fp_current_expr_ = Coef2Object(expr_coef_, fp_expr_mu_, fp_expr_pc_, expr_ev_, num_expr_pc_);
	fp_current_blendshape_ = fp_current_shape_ + fp_current_expr_;
	BFM_DEBUG("success\n");
}


void CBaselFaceModelManager::GenRMat() 
{
	BFM_DEBUG("generate rotation matrix - ");
	const double &yaw   = ext_params_[0];
	const double &pitch = ext_params_[1];
	const double &roll  = ext_params_[2];
	r_mat_ = bfm_utils::Euler2Mat(yaw, pitch, roll, false);
	BFM_DEBUG("success\n");
}


void CBaselFaceModelManager::GenTVec()
{
	BFM_DEBUG("generate translation vector - ");	
	const double &tx = ext_params_[3];
	const double &ty = ext_params_[4];
	const double &tz = ext_params_[5];
	t_vec_ << tx, ty, tz;	
	BFM_DEBUG("success\n");
	PrintTVec();
}

void CBaselFaceModelManager::GenTransMat()
{
	BFM_DEBUG("generate transform matrix (rotation + translation):\n");
	GenRMat();
	GenTVec();
}


void CBaselFaceModelManager::GenExtParams()
{
	BFM_DEBUG("generate external paramter:\n");
	if(!bfm_utils::IsRMat(r_mat_))
	{
		BFM_DEBUG("	detect current matrix does not satisfy constraints - ");
		bfm_utils::SatisfyExtMat(r_mat_, t_vec_);
		BFM_DEBUG("solve\n");
	}
	double sy = sqrt(r_mat_(0,0) * r_mat_(0,0) +  r_mat_(1,0) * r_mat_(1,0));
    bool is_singular = sy < 1e-6;

    if (!is_singular) 
	{
        ext_params_[2] = atan2(r_mat_(2,1) , r_mat_(2,2));
        ext_params_[1] = atan2(-r_mat_(2,0), sy);
        ext_params_[0] = atan2(r_mat_(1,0), r_mat_(0,0));
    } 
	else 
	{
        ext_params_[2] = atan2(-r_mat_(1,2), r_mat_(1,1));
        ext_params_[1] = atan2(-r_mat_(2,0), sy);
        ext_params_[0] = 0;
    }
	ext_params_[3] = t_vec_(0, 0);
	ext_params_[4] = t_vec_(1, 0);
	ext_params_[5] = t_vec_(2, 0);
	GenTransMat();
}


void CBaselFaceModelManager::AccExtParams(double *x) 
{
	/* in every iteration, P = R`(RP+t)+t`, 
	 * R_{new} = R`R_{old}
	 * t_{new} = R`t_{old} + t`
	 */

	Matrix3d d_r_mat;
	Vector3d d_t_vec;	
	double d_yaw   = x[0];
	double d_pitch = x[1];
	double d_roll  = x[2];
	double d_tx = x[3];
	double d_ty = x[4];
	double d_tz = x[5];

	/* accumulate rotation */
	d_r_mat = bfm_utils::Euler2Mat(d_yaw, d_pitch, d_roll, true);
	r_mat_ = d_r_mat * r_mat_;

	/* accumulate translation */
	d_t_vec << d_tx, d_ty, d_tz;
	t_vec_ = d_r_mat * t_vec_ + d_t_vec;
}	


void CBaselFaceModelManager::WritePly(std::string fn, model_write_mode mode) const 
{
	std::ofstream out;
	/* Note: In Linux Cpp, we should use std::ios::BFM_OUT as flag, which is not necessary in Windows */
	out.open(fn, std::ios::out | std::ios::binary);
	if (!out) 
	{
		BFM_DEBUG("Creation of %s failed.\n", fn.c_str());
		return;
	}
	out << "ply\n";
	out << "format binary_little_endian 1.0\n";
	out << "comment Made from the 3D Morphable Face Model of the Univeristy of Basel, Switzerland.\n";
	out << "element vertex " << num_vertice_ << "\n";
	out << "property float x\n";
	out << "property float y\n";
	out << "property float z\n";
	out << "property uchar red\n";
	out << "property uchar green\n";
	out << "property uchar blue\n";
	out << "element face " << num_face_ << "\n";
	out << "property list uchar int vertex_indices\n";
	out << "end_header\n";

	int cnt = 0;
	for (int i = 0; i < num_vertice_; i++) 
	{
		float x, y, z;
		if(mode & NO_EXPR) 
		{
			x = float(current_shape_(i * 3));
			y = float(current_shape_(i * 3 + 1));
			z = float(current_shape_(i * 3 + 2));
		} 
		else 
		{
			x = float(current_blendshape_(i * 3));
			y = float(current_blendshape_(i * 3 + 1));
			z = float(current_blendshape_(i * 3 + 2));
		}

		if(mode & CAMERA_COORD) 
		{
			bfm_utils::Trans(ext_params_, x, y, z);
			y = -y; z = -z;
		}

		unsigned char r, g, b;
		if ((mode & PICK_FP) && std::find(fp_idx_.begin(), fp_idx_.end(), i) != fp_idx_.end()) 
		{
			r = 255;
			g = 0;
			b = 0;
			cnt++;
		} 
		else 
		{
			r = current_tex_(i * 3);
			g = current_tex_(i * 3 + 1);
			b = current_tex_(i * 3 + 2);
		}

		out.write((char *)&x, sizeof(x));
		out.write((char *)&y, sizeof(y));
		out.write((char *)&z, sizeof(z));
		out.write((char *)&r, sizeof(r));
		out.write((char *)&g, sizeof(g));
		out.write((char *)&b, sizeof(b));
	}

	if ((mode & PICK_FP) && cnt != num_fp_) 
	{
		BFM_DEBUG("[ERROR] Pick too less landmarks.\n");
		BFM_DEBUG("Number of picked points is %d.\n", cnt);
	}

	unsigned char N_VER_PER_FACE = 3;
	for (int i = 0; i < num_face_; i++) 
	{
		out.write((char *)&N_VER_PER_FACE, sizeof(N_VER_PER_FACE));
		int x = tl_(i * 3) - 1;
		int y = tl_(i * 3 + 1) - 1;
		int z = tl_(i * 3 + 2) - 1;
		out.write((char *)&y, sizeof(y));
		out.write((char *)&x, sizeof(x));
		out.write((char *)&z, sizeof(z));
	}

	out.close();
}


void CBaselFaceModelManager::WriteFpPly(std::string fn) const {
	std::ofstream out;
	/* Note: In Linux Cpp, we should use std::ios::BFM_OUT as flag, which is not necessary in Windows */
	out.open(fn, std::ios::out | std::ios::binary);
	if (!out) 
	{
		BFM_DEBUG("Creation of %s failed.\n", fn.c_str());
		return;
	}

	out << "ply\n";
	out << "format binary_little_endian 1.0\n";
	out << "comment Made from the 3D Morphable Face Model of the Univeristy of Basel, Switzerland.\n";
	out << "element vertex " << num_fp_ << "\n";
	out << "property float x\n";
	out << "property float y\n";
	out << "property float z\n";
	out << "end_header\n";

	int cnt = 0;
	for (int i = 0; i < num_fp_; i++) 
	{
		float x, y, z;
		x = float(fp_current_blendshape_(i * 3));
		y = float(fp_current_blendshape_(i * 3 + 1));
		z = float(fp_current_blendshape_(i * 3 + 2));
		out.write((char *)&x, sizeof(x));
		out.write((char *)&y, sizeof(y));
		out.write((char *)&z, sizeof(z));
	}

	out.close();	
}


void CBaselFaceModelManager::ClrExtParams()
{
	std::fill(ext_params_, ext_params_ + 6, 0.0);
	GenTransMat();
	GenFace();
}