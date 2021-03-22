#include "bfm_manager.h"

#include <fstream>
#include <iostream>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>


namespace fs = boost::filesystem;
namespace po = boost::program_options;
using namespace std;


const std::string LOG_PATH = R"(./log)";


int main(int argc, char *argv[])
{
    // logging
    google::InitGoogleLogging(argv[0]); 
    FLAGS_logtostderr = false;
    if(fs::exists(LOG_PATH)) 
        fs::remove_all(LOG_PATH);
    fs::create_directory(LOG_PATH);
    FLAGS_alsologtostderr = true;
    FLAGS_log_dir = LOG_PATH;
    FLAGS_log_prefix = true; 
    FLAGS_colorlogtostderr =true;

	std::ifstream in;
	if(argc > 1)
		in.open(argv[1], std::ios::in);
	else
		in.open("./example/example-inputs.txt", std::ios::in);
	if(!in.is_open())
	{
		std::cout << "can't open inputs.txt" << std::endl;
		return -1;
	}

	std::string strBfmH5Path;
	unsigned int nVertice, nFace, nIdPc, nExprPc;
	std::string strIntParam;
	double aIntParams[4] = { 0.0 };
	unsigned int nFp;
	std::string strFpIdxPath = "";
	in >> strBfmH5Path;
	in >> nVertice >> nFace >> nIdPc >> nExprPc;
	for(auto i = 0; i < 4; i++)
	{
		in >> strIntParam;
		aIntParams[i] = atof(strIntParam.c_str());
	}
	in >> nFp;
	if(nFp != 0) in >> strFpIdxPath;
	in.close();

	BaselFaceModelManager *modelManager = new BaselFaceModelManager(
		strBfmH5Path,
		aIntParams,
		nFp,
		strFpIdxPath
	);

	modelManager->genAvgFace();
	modelManager->writePly("avg_face.ply", ModelWriteMode_None);

	modelManager->genRndFace(1.0);
	modelManager->writePly("rnd_face.ply", ModelWriteMode_None);

	google::ShutdownGoogleLogging();
	return 0;
}
