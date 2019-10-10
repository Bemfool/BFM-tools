#include "data.h"

int load(vector<vec3> &shape_mu, vector<double> &shape_ev, vector<vector<vec3>> &shape_pc,
		 vector<vec3> &tex_mu, vector<double> &tex_ev, vector<vector<vec3>> &tex_pc, vector<vec3> &tl) {
	cout << "data list: " << endl;
	ifstream in;
	int res = 0;
	load_model(shape_mu);
	load_model(shape_ev);
	load_pc_model(shape_pc);
	load_model(tex_mu);
	load_model(tex_ev);
	load_pc_model(tex_pc);
	load_model(tl);
	return (res < 0) ? FAIL : SUCCESS;
}
