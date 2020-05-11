#include "svg_manager.h"

using namespace std;

SVGManager::SVGManager() :
	svgHtmlFileStream()
{
	fileInit();
}

//*************************************************************************************************************
//*************************************************************************************************************

SVGManager::~SVGManager() {
	fileClose();
}

//*************************************************************************************************************
//*************************************************************************************************************

void SVGManager::fileInit() {
	svgHtmlFileStream.open(SVG_HTML_FILE_NAME, ofstream::out | ofstream::trunc);
	svgHtmlFileStream << FILE_START;
}

//*************************************************************************************************************
//*************************************************************************************************************

void SVGManager::fileDone() {
	svgHtmlFileStream << FILE_END;
}

//*************************************************************************************************************
//*************************************************************************************************************

void SVGManager::fileClose() {
	svgHtmlFileStream.close();
}

//*************************************************************************************************************
//*************************************************************************************************************

void SVGManager::filePrintStr(string strToPrint) {
	svgHtmlFileStream << strToPrint;
}

//*************************************************************************************************************
//*************************************************************************************************************

string SVGManager::constructGId(int id) const {
	string res = ID_BEGIN;
	res.append(to_string(id));
	res.append(ID_END);

	return res;
}

//*************************************************************************************************************
//*************************************************************************************************************

void SVGManager::constructPaths(const vector<pair<float, float>> (&podsPaths)[4], const int populationIdx, const bool enemySimulation) {
	string polyLines{};

	for (int podIdx = 0; podIdx < 4; ++podIdx) {
		if (enemySimulation && podIdx < 2) {
			continue;
		}
		else if (!enemySimulation && podIdx >= 2) {
			continue;
		}
		else {
			polyLines += POLYLINE_BEGIN;
			for (size_t pIdx = 0; pIdx < podsPaths[podIdx].size(); ++pIdx) {
				polyLines += to_string(podsPaths[podIdx][pIdx].first);
				polyLines += COMMA_SEPARATOR;
				polyLines += to_string(podsPaths[podIdx][pIdx].second);
				polyLines += SPACE_SEPARATOR;
			}

			if (2 == podIdx) {
				polyLines += POLYLINE_MY_TEAM_END;
			}
			else {
				polyLines += POLYLINE_ENEMY_TEAM_END;
			}
		}
	}

	filePrintStr(polyLines);
}
