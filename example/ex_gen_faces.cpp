#include "bfm_manager.h"
#include <fstream>
#include <iostream>

int main(int argc, char *argv[])
{
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
	std::string strShapeMuH5Path, strShapeEvH5Path, strShapePcH5Path;
	std::string strTexMuH5Path, strTexEvH5Path, strTexPcH5Path;
	std::string strExprMuH5Path, strExprEvH5Path, strExprPcH5Path;
	std::string strTlH5Path;
	unsigned int nFp;
	std::string strFpIdxPath = "";
	in >> strBfmH5Path;
	in >> nVertice >> nFace >> nIdPc >> nExprPc;
	for(auto i = 0; i < 4; i++)
	{
		in >> strIntParam;
		aIntParams[i] = atof(strIntParam.c_str());
	}
	in >> strShapeMuH5Path >> strShapeEvH5Path >>strShapePcH5Path;
	in >> strTexMuH5Path >> strTexEvH5Path >> strTexPcH5Path;
	in >> strExprMuH5Path >> strExprEvH5Path >> strExprPcH5Path;
	in >> strTlH5Path;
	in >> nFp;
	if(nFp != 0) in >> strFpIdxPath;
	in.close();

	BaselFaceModelManager *modelManager = new BaselFaceModelManager(
		strBfmH5Path,
		nVertice, nFace, nIdPc, nExprPc,
		aIntParams,
		strShapeMuH5Path, strShapeEvH5Path, strShapePcH5Path,
		strTexMuH5Path, strTexEvH5Path, strTexPcH5Path,
		strExprMuH5Path, strExprEvH5Path, strExprPcH5Path,
		strTlH5Path,
		nFp,
		strFpIdxPath
	);

	modelManager->genAvgFace();
	modelManager->writePly("avg_face.ply", NONE_MODE);

	modelManager->genRndFace(1.0);
	modelManager->writePly("rnd_face.ply", NONE_MODE);

	return 0;
}
