#pragma once
#include <fstream>
#include <iostream>
#include <vector>
#include "vec3.h"
#include "constant.h"
using namespace std;

#define load_model(model_type) { \
	cout << "	" << #model_type << " - ";	\
	in.open(model_type##_path);	\
	if (!in) {	\
		cout << "fail" << endl;	\
		res += FAIL;	\
	} else {	\
		for (auto it = model_type.begin(); it != model_type.end(); it++)	\
			in >> (*it);	\
		in.close();	\
		cout << "success" << endl;	\
	} \
}

#define load_pc_model(model_type) { \
	cout << "	" << #model_type << " - ";	\
	in.open(model_type##_path);	\
	if (!in) {	\
		cout << "fail" << endl;	\
		res += FAIL;	\
	} else {	\
		int flag = 0;	\
		for(int i=0; i<N_VERTICE; i++) {	\
			for(int j=0; j<N_PC; j++) {	\
				if(flag == 0)	\
					in >> model_type[j][i].x;	\
				else if(flag == 1)	\
					in >> model_type[j][i].y;	\
				else  \
					in >> model_type[j][i].z;	\
			}	\
			flag = (flag + 1) % 3;	\
			if(flag != 0)	\
				i--;	\
		}	\
		in.close();	\
		cout << "success" << endl;	\
	}	\
}

int load(vector<vec3> &shape_mu, vector<double> &shape_ev, vector<vector<vec3>> &shape_pc,
	vector<vec3> &tex_mu, vector<double> &tex_ev, vector<vector<vec3>> &tex_pc, vector<vec3> &tl);