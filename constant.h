#pragma once
#include <string>
using namespace std;

static string bfm_path = "D:\\database\\bfm\\2009\\PublicMM1\\";
static string shape_mu_path = bfm_path + "shapeMU.dat";
static string shape_ev_path = bfm_path + "shapeEV.dat";
static string shape_pc_path = bfm_path + "shapePC.dat";
static string tex_mu_path = bfm_path + "texMU.dat";
static string tex_ev_path = bfm_path + "texEV.dat";
static string tex_pc_path = bfm_path + "texPC.dat";
static string tl_path = bfm_path + "tl.dat";

#define N_VERTICE 53490
#define N_FACE 106466
#define N_PC 199

#define FAIL -1
#define SUCCESS 0