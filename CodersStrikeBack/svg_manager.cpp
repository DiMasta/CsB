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

std::string SVGManager::constructGId(int id) const {
	string res = ID_BEGIN;
	res.append(to_string(id));
	res.append(ID_END);

	return res;
}