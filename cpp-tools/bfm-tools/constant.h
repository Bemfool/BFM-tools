#pragma once
#include <string>

static std::string bfm_path = "D:\\database\\bfm\\2009\\PublicMM1\\";
static std::string shape_mu_path = bfm_path + "shapeMU.dat";
static std::string shape_ev_path = bfm_path + "shapeEV.dat";
static std::string shape_pc_path = bfm_path + "shapePC.dat";
static std::string tex_mu_path = bfm_path + "texMU.dat";
static std::string tex_ev_path = bfm_path + "texEV.dat";
static std::string tex_pc_path = bfm_path + "texPC.dat";
static std::string tl_path = bfm_path + "tl.dat";

#define N_VERTICE 53490
#define N_FACE 106466
#define N_PC 199

#define FAIL -1
#define SUCCESS 0

inline void set_bfm_path(std::string new_path) { bfm_path = new_path; }