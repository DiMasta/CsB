#ifndef __DEBUG_H__
#define __DEBUG_H__

#define DOCTEST_CONFIG_IMPLEMENT
#include "doctest.h"

//TEST_CASE("Test MiniMax with hardcoded evaluation!") {
//	Game game;
//
//	State state;
//	Action action = game.chooseAction(&state, PR_INVALID);
//
//}

TEST_CASE("Test Simulation of a turn!") {
	const Pod pods[PODS_COUNT] = {
		{ 7779, 7416, 0, 0, -1, 1 },
		{ 8185, 8330, 0, 0, -1, 1 },
		{ 7372, 6503, 0, 0, -1, 1 },
		{ 8592, 9243, 0, 0, -1, 1 }
	};

	const int actualCPCount = 4;
	const Coords checkPoints[MAX_CHECKPOINTS_COUNT] = {
		{ 7982, 7873 },
		{ 13284, 5513 },
		{ 9539, 1380 },
		{ 3637, 4405 },
		{},
		{},
		{},
		{}
	};

	const vector<vector<Action>> actions = {
		{
			{ { 13284, 5513 }, "BOOST" },
			{ { 13284, 5513 }, "100" },
			{ { 13284, 5513 }, "100" },
			{ { 13284, 5513 }, "100" }
		},
		{
			{ { 13284, 5513 }, "100" },
			{ { 13284, 5513 }, "100" },
			{ { 12952, 5569 }, "BOOST"},
			{ { 13020, 5721 }, "100" }
		},
		{
			{ { 13284, 5513 }, "100" },
			{ { 13284, 5513 }, "100" },
			{ { 10824, 5925 }, "100" },
			{ { 12796, 5901 }, "100" }
		},
		{
			{ { 13284, 5513 }, "100" },
			{ { 13284, 5513 }, "100" },
			{ { 10860, 5917 }, "100" },
			{ { 12604, 6053 }, "100" }
		},
		{
			{ { 13284, 5513 }, "100" },
			{ { 13284, 5513 }, "100" },
			{ { 11396, 6265 }, "100" },
			{ { 12440, 6181 }, "100" }
		},
		{
			{ { 13284, 5513 }, "100" },
			{ { 13284, 5513 }, "100" },
			{ { 11344, 6121 }, "100" },
			{ { 12116, 5817 }, "100" }
		},
		{
			{ { 13284, 5513 }, "100" },
			{ { 13284, 5513 }, "100" },
			{ { 7555, 1840 }, "0" },
			{ { 12060, 6013 }, "100" }
		},
		{
			{ { 13284, 5513 }, "100" },
			{ { 13284, 5513 }, "100" },
			{ { 7855, 1768 }, "100" },
			{ { 12016, 6185 }, "100" }
		},
		{
			{ { 13284, 5513 }, "100" },
			{ { 13284, 5513 }, "100" },
			{ { 7807, 1860 }, "100" },
			{ { 11984, 6337 }, "100" }
		},
		{
			{ { 13284, 5513 }, "100" },
			{ { 13284, 5513 }, "100" },
			{ { 7827, 2024 }, "0" },
			{ { 11964, 6473 }, "100" }
		},
		{
			{ { 13284, 5513 }, "100" },
			{ { 13284, 5513 }, "100" },
			{ { 8087, 1924 }, "0" },
			{ { 11952, 6593 }, "100" }
		},
		{
			{ { 13284, 5513 }, "100" },
			{ { 13284, 5513 }, "100" },
			{ { 8307, 1840 }, "100" },
			{ { 8211, 2572 }, "100" }
		},
		{
			{ { 13284, 5513 }, "100" },
			{ { 13284, 5513 }, "100" },
			{ { 8543, 2104 }, "100" },
			{ { 8311, 2716 }, "100" }
		},
		{
			{ { 13284, 5513 }, "100" },
			{ { 13284, 5513 }, "100" },
			{ { 8847, 2296 }, "SHIELD" },
			{ { 8583, 2160 }, "100" }
		},
		{
			{ { 13284, 5513 }, "100" },
			{ { 13284, 5513 }, "100" },
			{ { 8835, 2324 }, "100" },
			{ { 8839, 2364 }, "100" }
		},
		{
			{ { 13284, 5513 }, "100" },
			{ { 13284, 5513 }, "100" },
			{ { 8943, 2180 }, "100" },
			{ { 9151, 2488 }, "100" }
		},
		{
			{ { 13284, 5513 }, "100" },
			{ { 13284, 5513 }, "100" },
			{ { 9035, 2060 }, "100" },
			{ { 9471, 2540 }, "100" }
		},
		{
			{ { 13284, 5513 }, "100" },
			{ { 13284, 5513 }, "100" },
			{ { 9111, 1956 }, "100" },
			{ { 9743, 2576 }, "100" }
		},
		{
			{ { 13284, 5513 }, "100" },
			{ { 13284, 5513 }, "100" },
			{ { 9499, 1980 }, "100" },
			{ { 9979, 2600 }, "100" }
		},
		{
			{ { 13284, 5513 }, "100" },
			{ { 13284, 5513 }, "100" },
			{ { 9827, 1996 }, "100" },
			{ { 10187, 2616 }, "100" }
		}
	};

	Track track(checkPoints, actualCPCount);
	RaceSimulator raceSimulator(pods, track);

	raceSimulator.simulate(actions, true);

	SUBCASE("Tests after several turns") {
		Pod cgSimulatedPods[PODS_COUNT] {
			{ 14624, 2712, -216, 49, 119, 2 },
			{ 14721, 6782, 107, 345, 119, 2 },
			{ 13922, 3176, -142, -156, 198, 2 },
			{ 12218, 3815, -207, -310, 215, 2 }
		};

		for (int simPodIdx = 0; simPodIdx < PODS_COUNT; ++simPodIdx) {
			const Pod& simPod = raceSimulator.getPod(simPodIdx);

			CHECK(simPod == cgSimulatedPods[simPodIdx]);
		}
	}
}

#endif //__DEBUG_H__
