#pragma once

#include <iostream>
#include <fstream>
#include <vector>

#include "svg_consts.h"

class SVGManager {
public:
	SVGManager();
	~SVGManager();

	void fileInit(const int turnIdx, const bool enemyTeam);
	void fileDone();
	void fileClose();
	void filePrintStr(std::string strToPrint);

	std::string constructGId(int id) const;

	/// Construct the polylines for the given pods paths
	/// @param[in] podsPaths the paths for all 4 pods
	/// @param[in] enemySimulation true if the data is form enemy simulation
	void constructPaths(
		std::vector<std::pair<float, float>> (&podsPaths)[4],
		const bool enemySimulation
	);

private:
	std::ofstream svgHtmlFileStream;
};