#include "bfm_manager.h"

#include <fstream>
#include <iostream>
#include <array>
#include <memory>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>


namespace fs = boost::filesystem;
namespace po = boost::program_options;


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
		LOG(ERROR) << "Cannot open inputs.txt";
		exit(-1);
	}

	std::string sBfmH5Path, sLandmarkIdxPath = "";
	std::array<double, N_INT_PARAMS> aIntParams = { };
	
	in >> sBfmH5Path;
	for(auto i = 0; i < N_INT_PARAMS; i++)
		in >> aIntParams[i];
	if(!in.eof())
		in >> sLandmarkIdxPath;
	else
		sLandmarkIdxPath = "";
	in.close();

	std::unique_ptr<BfmManager> pBfmManager(new BfmManager(sBfmH5Path, aIntParams, sLandmarkIdxPath));

	pBfmManager->genAvgFace();
	pBfmManager->writePly("avg_face.ply", ModelWriteMode_None);

	pBfmManager->genRndFace(1.0);
	pBfmManager->writePly("rnd_face.ply", ModelWriteMode_None);

	google::ShutdownGoogleLogging();
	return 0;
}
