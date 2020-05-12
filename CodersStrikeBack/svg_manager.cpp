#include "svg_manager.h"

using namespace std;

SVGManager::SVGManager() :
	svgHtmlFileStream()
{
}

//*************************************************************************************************************
//*************************************************************************************************************

SVGManager::~SVGManager() {
}

//*************************************************************************************************************
//*************************************************************************************************************

void SVGManager::fileInit(const int turnIdx, const bool enemyTeam) {
	string svgHtmlFileName = "";
	string turnText = TURN_TEXT_BEGIN;
	turnText += to_string(turnIdx);
	turnText += TURN_TEXT_MIDDLE;

	if (enemyTeam) {
		svgHtmlFileName = SVG_HTML_FILE_NAME_ENEMY_BEGIN;
		turnText += ENEMY;
	}
	else {
		svgHtmlFileName = SVG_HTML_FILE_NAME_MY_BEGIN;
		turnText += MY;
	}

	turnText += TURN_TEXT_END;

	svgHtmlFileName += to_string(turnIdx);
	svgHtmlFileName += SVG_HTML_FILE_NAME_END;

	svgHtmlFileStream.open(svgHtmlFileName, ofstream::out | ofstream::trunc);
	svgHtmlFileStream << FILE_START;
	svgHtmlFileStream << turnText;
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

void SVGManager::constructPaths(vector<pair<float, float>> (&podsPaths)[4], const bool enemySimulation) {
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

			if (enemySimulation) {
				polyLines += POLYLINE_ENEMY_TEAM_END;
			}
			else {
				polyLines += POLYLINE_MY_TEAM_END;
			}
		}

		podsPaths[podIdx].clear();
	}

	filePrintStr(polyLines);
}
