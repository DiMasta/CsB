#pragma once

#include <iostream>
#include <fstream>
#include <vector>

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

	/// Construct the polylines for the given pods paths
	/// @param[in] podsPaths the paths for all 4 pods
	/// @param[in] populationIdx current population index
	/// @param[in] enemySimulation true if the data is form enemy simulation
	void constructPaths(
		const std::vector<std::pair<float, float>> (&podsPaths)[4],
		const int populationIdx,
		const bool enemySimulation
	);

private:
	std::ofstream svgHtmlFileStream;
};