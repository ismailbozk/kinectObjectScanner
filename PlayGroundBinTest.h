#ifndef PLAYGROUNDBINTEST_H
#define PLAYGROUNDBINTEST_H

#include "Models\DepthScale.h"
#include <vector>

class PlayGroundBinTest
{
public:
	PlayGroundBinTest(void);
	void startToPlay();
private:
	std::vector<std::vector<int>> PlayGroundBinTest::GetDsIndicesDivideByBins(std::vector<DepthScale> depthScales, std::vector<int> bins);
};
#endif
