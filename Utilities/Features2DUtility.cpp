#include "Features2DUtility.h"
#include <iostream>

using namespace std;

void Features2DUtility::VoteForUniqueness(std::vector<std::vector<cv::DMatch>> matches, float threshold, std::vector<bool> &mask)
{
	if (mask.size() == matches.size())
	{
		for (int i = 0; i < matches.size(); i++)
		{
			if (matches[i].size() >= 2) //if there is at least two matches
			{
				cv::DMatch firstCand = matches[i][0];
				cv::DMatch secondCand = matches[i][1];

				float distRatio = firstCand.distance / secondCand.distance;
				if (distRatio > threshold)
				{
					mask[i] = false;
				}
			}
		}
	}
	else
	{
		string errorMessage = "Features2DUtility::VoteForUniqueness mask and match size must be equal!!!";
		cout << errorMessage << endl;
		throw std::invalid_argument(errorMessage );
	}
}

