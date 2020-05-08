#pragma once

#include <iostream>
#include <fstream>

#include "svg_consts.h"

class SVGManager {
public:
	SVGManager();
	~SVGManager();

	void fileInit();
	void fileDone();
	void fileClose();
	void filePrintStr(std::string strToPrint);

	std::string constructGId(int id) const;

private:
	std::ofstream svgHtmlFileStream;
};