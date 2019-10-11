#include "data.h"

vector<vec3> shape_mu(N_VERTICE);
vector<double> shape_ev(N_PC);
vector<vector<vec3>> shape_pc(N_PC);
vector<vec3> tex_mu(N_VERTICE);
vector<double> tex_ev(N_PC);
vector<vector<vec3>> tex_pc(N_PC);
vector<vec3> tl(N_FACE);

int load() {
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
