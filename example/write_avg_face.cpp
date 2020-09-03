#include "bfm_manager.h"
#include <fstream>
#include <iostream>

int main()
{
	
	std::ifstream in;
	in.open("example/example-inputs.txt", std::ios::in);
	std::string strBfmH5Path;
	unsigned int nVertice, nFace, nIdPc, nExprPc;
	double aIntParams[4] = { 0.0 };
	std::string strShapeMuH5Path, strShapeEvH5Path, strShapePcH5Path;
	std::string strTexMuH5Path, strTexEvH5Path, strTexPcH5Path;
	std::string strExprMuH5Path, strExprEvH5Path, strExprPcH5Path;
	std::string strTlH5Path;
	unsigned int nFp;
	std::string strFpIdxPath = "";
	in >> strBfmH5Path;
	in >> nVertice >> nFace >> nIdPc >> nExprPc;
	in >> aIntParams[0] >> aIntParams[1] >> aIntParams[2] >> aIntParams[3];
	in >> strShapeMuH5Path >> strShapeEvH5Path >>strShapePcH5Path;
	in >> strTexMuH5Path >> strTexEvH5Path >> strTexPcH5Path;
	in >> strExprMuH5Path >> strExprEvH5Path >> strExprPcH5Path;
	in >> strTlH5Path;
	in >> nFp;
	if(nFp != 0) in >> strFpIdxPath;
	in.close();

	CBaselFaceModelManager *modelManager = new CBaselFaceModelManager(
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

	modelManager->GenAvgFace();
	modelManager->WritePly("rnd_face.ply", NONE_MODE);

	return 0;
}
