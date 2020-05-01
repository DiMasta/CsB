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

	const vector<Action[PODS_COUNT]> actions = {
		{
			{ 13284, 5513, "BOOST" },
			{ 13284, 5513, "100" },
			{ 13284, 5513, "100" },
			{ 13284, 5513, "100" }
		},
		{
			{ 13284, 5513, "100" },
			{ 13284, 5513, "100" },
			{ 12952, 5569, "BOOST"},
			{ 13020, 5721, "100" }
		},
		{
			{ 13284, 5513, "100" },
			{ 13284, 5513, "100" },
			{ 10824, 5925, "100" },
			{ 12796, 5901, "100" }
		},
		{
			{ 13284, 5513, "100" },
			{ 13284, 5513, "100" },
			{ 10860, 5917, "100" },
			{ 12604, 6053, "100" }
		},
		{
			{ 13284, 5513, "100" },
			{ 13284, 5513, "100" },
			{ 11396, 6265, "100" },
			{ 12440, 6181, "100" }
		},
		{
			{ 13284, 5513, "100" },
			{ 13284, 5513, "100" },
			{ 11344, 6121, "100" },
			{ 12116, 5817, "100" }
		},
		{
			{ 13284, 5513, "100" },
			{ 13284, 5513, "100" },
			{ 7555, 1840, "0" },
			{ 12060, 6013, "100" }
		},
		{
			{ 13284, 5513, "100" },
			{ 13284, 5513, "100" },
			{ 7855, 1768, "100" },
			{ 12016, 6185, "100" }
		},
		{
			{ 13284, 5513, "100" },
			{ 13284, 5513, "100" },
			{ 7807, 1860, "100" },
			{ 11984, 6337, "100" }
		},
		{
			{ 13284, 5513, "100" },
			{ 13284, 5513, "100" },
			{ 7827, 2024, "0" },
			{ 11964, 6473, "100" }
		},
		{
			{ 13284, 5513, "100" },
			{ 13284, 5513, "100" },
			{ 8087, 1924, "0" },
			{ 11952, 6593, "100" }
		},
		{
			{ 13284, 5513, "100" },
			{ 13284, 5513, "100" },
			{ 8307, 1840, "100" },
			{ 8211, 2572, "100" }
		},
		{
			{ 13284, 5513, "100" },
			{ 13284, 5513, "100" },
			{ 8543, 2104, "100" },
			{ 8311, 2716, "100" }
		},
		{
			{ 13284, 5513, "100" },
			{ 13284, 5513, "100" },
			{ 8847, 2296, "SHIELD" },
			{ 8583, 2160, "100" }
		},
		{
			{ 13284, 5513, "100" },
			{ 13284, 5513, "100" },
			{ 8835, 2324, "100" },
			{ 8839, 2364, "100" }
		},
		{
			{ 13284, 5513, "100" },
			{ 13284, 5513, "100" },
			{ 8943, 2180, "100" },
			{ 9151, 2488, "100" }
		},
		{
			{ 13284, 5513, "100" },
			{ 13284, 5513, "100" },
			{ 9035, 2060, "100" },
			{ 9471, 2540, "100" }
		},
		{
			{ 13284, 5513, "100" },
			{ 13284, 5513, "100" },
			{ 9111, 1956, "100" },
			{ 9743, 2576, "100" }
		},
		{
			{ 13284, 5513, "100" },
			{ 13284, 5513, "100" },
			{ 9499, 1980, "100" },
			{ 9979, 2600, "100" }
		},
		{
			{ 13284, 5513, "100" },
			{ 13284, 5513, "100" },
			{ 9827, 1996, "100" },
			{ 10187, 2616, "100" }
		}
	};

	Track track(checkPoints, actualCPCount);
	RaceSimulator raceSimulator(pods, track);

	raceSimulator.simulate(actions);

	//CheckPoint* cp0 = new CheckPoint(Coords(7982.f, 7873.f), Coords(0.f, 0.f), CHECKPOINT_RADIUS, 0);
	//CheckPoint* cp1 = new CheckPoint(Coords(13284.f, 5513.f), Coords(0.f, 0.f), CHECKPOINT_RADIUS, 1);
	//CheckPoint* cp2 = new CheckPoint(Coords(9539.f, 1380.f), Coords(0.f, 0.f), CHECKPOINT_RADIUS, 2);
	//CheckPoint* cp3 = new CheckPoint(Coords(3637.f, 4405.f), Coords(0.f, 0.f), CHECKPOINT_RADIUS, 3);
	//
	//Pod** debugPods = new Pod*[GAME_PODS_COUNT];
	//CheckPoint** debugCheckPoints = new CheckPoint*[GAME_PODS_COUNT];
	//
	//debugPods[0] = pod0;
	//debugPods[1] = pod1;
	//debugPods[2] = pod2;
	//debugPods[3] = pod3;
	//
	//debugCheckPoints[0] = cp0;
	//debugCheckPoints[1] = cp1;
	//debugCheckPoints[2] = cp2;
	//debugCheckPoints[3] = cp3;
	//
	//State* debugState = new State(GAME_PODS_COUNT, debugPods, GAME_PODS_COUNT, debugCheckPoints);
	//
	//Action actionForSimulation[GAME_PODS_COUNT];
	//
	//actionForSimulation[0] = Action(Coords(13284.f, 5513.f), false, 100, 0); 
	//actionForSimulation[1] = Action(Coords(13284.f, 5513.f), false, 100, 0);
	//actionForSimulation[2] = Action(Coords(10892.f, 5909.f), false, 100, 0);
	//actionForSimulation[3] = Action(Coords(12440.f, 6181.f), false, 100, 0);
	//
	//// MY_POD: Coords:(X = 8933, Y = 6996) Speed : (X = 245, Y = -219) Angle : 341
	//// MY_POD : Coords : (X = 9259, Y = 7735) Speed : (X = 255, Y = -211) Angle : 331
	//// ENEMY_POD : Coords : (X = 10311, Y = 6011) Speed : (X = 592, Y = -98) Angle : 350
	//// ENEMY_POD : Coords : (X = 9562, Y = 8509) Speed : (X = 316, Y = -16) Angle : 321
	//debugState->simulateTurn(actionForSimulation);
	//
	//State* turnNplus1 = new State(debugState);
	//actionForSimulation[0] = Action(Coords(13284.f, 5513.f), false, 100, 0);
	//actionForSimulation[1] = Action(Coords(13284.f, 5513.f), false, 100, 0);
	//actionForSimulation[2] = Action(Coords(7171.f, 1772.f), false, 100, 0);
	//actionForSimulation[3] = Action(Coords(12020.f, 5577.f), false, 100, 0);
	//
	////MY_POD: Coords:(X = 9273, Y = 6745) Speed : (X = 288, Y = -213) Angle : 341
	////MY_POD : Coords : (X = 9602, Y = 7476) Speed : (X = 291, Y = -220) Angle : 331
	////ENEMY_POD : Coords : (X = 10992, Y = 5867) Speed : (X = 578, Y = -122) Angle : 332
	////ENEMY_POD : Coords : (X = 9942, Y = 8416) Speed : (X = 323, Y = -78) Angle : 310
	//turnNplus1->simulateTurn(actionForSimulation);
	//
	//State* turnNplus2 = new State(turnNplus1);
	//actionForSimulation[0] = Action(Coords(13284.f, 5513.f), false, 100, 0);
	//actionForSimulation[1] = Action(Coords(13284.f, 5513.f), false, 100, 0);
	//actionForSimulation[2] = Action(Coords(7227.f, 1868.f), false, 100, 0);
	//actionForSimulation[3] = Action(Coords(11992.f, 5825.f), false, 100, 0);
	//
	//// MY_POD: Coords:(X = 9606, Y = 6390) Speed : (X = 279, Y = -308) Angle : 343
	//// MY_POD : Coords : (X = 10032, Y = 7321) Speed : (X = 368, Y = -124) Angle : 332
	//// ENEMY_POD : Coords : (X = 11640, Y = 5673) Speed : (X = 550, Y = -164) Angle : 314
	//// ENEMY_POD : Coords : (X = 10327, Y = 8260) Speed : (X = 327, Y = -132) Angle : 308
	//turnNplus2->simulateTurn(actionForSimulation);
	//
	//State* turnNplus3 = new State(turnNplus2);
	//actionForSimulation[0] = Action(Coords(13284.f, 5513.f), false, 100, 0);
	//actionForSimulation[1] = Action(Coords(13284.f, 5513.f), false, 100, 0);
	//actionForSimulation[2] = Action(Coords(7339.f, 2036.f), false, 100, 0);
	//actionForSimulation[3] = Action(Coords(11976.f, 6041.f), false, 100, 0);
	//
	//// MY_POD: Coords:(X = 9982, Y = 6059) Speed : (X = 319, Y = -281) Angle : 347
	//// MY_POD : Coords : (X = 10487, Y = 7148) Speed : (X = 387, Y = -146) Angle : 331
	//// ENEMY_POD : Coords : (X = 12234, Y = 5419) Speed : (X = 505, Y = -215) Angle : 296
	//// ENEMY_POD : Coords : (X = 10714, Y = 8048) Speed : (X = 328, Y = -180) Angle : 307
	//turnNplus3->simulateTurn(actionForSimulation);
	//
	//State* turnNplus4 = new State(turnNplus3);
	//actionForSimulation[0] = Action(Coords(13284.f, 5513.f), false, 100, 0);
	//actionForSimulation[1] = Action(Coords(13284.f, 5513.f), false, 100, 0);
	//actionForSimulation[2] = Action(Coords(7519.f, 2240.f), false, 0, 0);
	//actionForSimulation[3] = Action(Coords(11972.f, 6233.f), false, 100, 0);
	//
	//// MY_POD: Coords:(X = 10400, Y = 5762) Speed : (X = 355, Y = -252) Angle : 351
	//// MY_POD : Coords : (X = 10960, Y = 6952) Speed : (X = 402, Y = -166) Angle : 330
	//// ENEMY_POD : Coords : (X = 12739, Y = 5204) Speed : (X = 429, Y = -182) Angle : 278
	//// ENEMY_POD : Coords : (X = 11099, Y = 7786) Speed : (X = 327, Y = -222) Angle : 305
	//turnNplus4->simulateTurn(actionForSimulation);

	//SUBCASE("Tests for turn 5") {
	//	CHECK(turnNplus4->getPod(0)->getPosition() == Coords(10400.f, 5762.f));
	//	CHECK(turnNplus4->getPod(1)->getPosition() == Coords(10960.f, 6952.f));
	//	CHECK(turnNplus4->getPod(2)->getPosition() == Coords(12739.f, 5204.f));
	//	CHECK(turnNplus4->getPod(3)->getPosition() == Coords(11099.f, 7786.f));
	//
	//	CHECK(turnNplus4->getPod(0)->getSpeedVector() == Coords(355.f, -252.f));
	//	CHECK(turnNplus4->getPod(1)->getSpeedVector() == Coords(402.f, -166.f));
	//	CHECK(turnNplus4->getPod(2)->getSpeedVector() == Coords(429.f, -182.f));
	//	CHECK(turnNplus4->getPod(3)->getSpeedVector() == Coords(327.f, -222.f));
	//
	//	CHECK(round(turnNplus4->getPod(0)->getAngle()) == 351);
	//	CHECK(round(turnNplus4->getPod(1)->getAngle()) == 330);
	//	CHECK(round(turnNplus4->getPod(2)->getAngle()) == 278);
	//	CHECK(round(turnNplus4->getPod(3)->getAngle()) == 305);
	//}

	//delete debugState;
	//delete turnNplus1;
	//delete turnNplus2;
	//delete turnNplus3;
	//delete turnNplus4;
	//
	//delete cp0;
	//delete cp1;
	//delete cp2;
	//delete cp3;
	//
	//delete[] debugCheckPoints;
}



#endif //__DEBUG_H__
