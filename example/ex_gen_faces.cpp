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

    // command options
    po::options_description opts("Options");
    po::variables_map vm;

    std::string sBfmH5Path, sLandmarkIdxPath;
	double dFx = 0.0, dFy = 0.0, dCx = 0.0, dCy = 0.0;

    opts.add_options()
        ("bfm_h5_path", po::value<string>(&sBfmH5Path)->default_value(
            R"(/home/keith/Data/BaselFaceModel_mod.h5)"), 
            "Path of Basel Face Model.")
        ("landmark_idx_path", po::value<string>(&sLandmarkIdxPath)->default_value(
            R"(/home/keith/Project/head-pose-estimation/data/example_landmark_68.anl)"), 
            "Path of corresponding between dlib and model vertex index.")
        ("fx", po::value<double>(&dFx)->default_value(1744.327628674942))
		("fy", po::value<double>(&dFx)->default_value(1747.838275588676))
		("cx", po::value<double>(&dFx)->default_value(800))
		("cy", po::value<double>(&dFx)->default_value(600))
        ("help,h", "Help message"); 
    try
    {
        po::store(po::parse_command_line(argc, argv, opts), vm);
    }
    catch(...)
    {
        LOG(ERROR) << "These exists undefined command options.";
        return -1;
    }

    po::notify(vm);
    if(vm.count("help"))
    {
        LOG(INFO) << opts;
        return 0;
    }

	std::array<double, N_INT_PARAMS> aIntParams = { dFx, dFy, dCx, dCy };
	std::unique_ptr<BfmManager> pBfmManager(new BfmManager(sBfmH5Path, aIntParams, sLandmarkIdxPath));

    pBfmManager->writeLandmarkPly("landmarks.ply");

	pBfmManager->genAvgFace();
	pBfmManager->writePly("avg_face.ply", ModelWriteMode_None);

	pBfmManager->genRndFace(1.0);
	pBfmManager->writePly("rnd_face.ply", ModelWriteMode_None);

	google::ShutdownGoogleLogging();
	return 0;
}
