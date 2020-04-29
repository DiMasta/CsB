#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <list>
#include <stack>
#include <set>
#include <queue>
#include <algorithm>
#include <ctime>
#include <deque>
#include <math.h>
#include <climits>
#include <cstring>
#include <fstream>
#include <iterator>
#include <bitset>
#include <cmath>
#include <iomanip>
#include <chrono>

using namespace std;

#define REDIRECT_INPUT
//#define OUTPUT_GAME_DATA
//#define TIME_MEASURERMENT
#define DEBUG_ONE_TURN
//#define USE_UNIFORM_RANDOM

//static const string INPUT_FILE_NAME = "input.txt";
static const string INPUT_FILE_NAME = "input_classic_track.txt";
static const string OUTPUT_FILE_NAME = "output.txt";
static const string EMPTY_STRING = "";
static const string SPACE = " ";

static constexpr int INVALID_ID = -1;
static constexpr int INVALID_IDX = -1;
static constexpr int INVALID_NODE_DEPTH = -1;
static constexpr int TREE_ROOT_NODE_DEPTH = 1;
static constexpr int ZERO_CHAR = '0';
static constexpr int DIRECTIONS_COUNT = 8;
static constexpr int BYTE_SIZE = 8;
static constexpr int PAIR = 2;
static constexpr int BASE_2 = 2;
static constexpr int BASE_10 = 10;
static constexpr int BASE_16 = 16;

static constexpr int INVALID_COORD = -1;
static constexpr int MAX_CHECKPOINTS_COUNT = 8;
static constexpr int PODS_COUNT = 4;
static constexpr int TEAM_PODS_COUNT = PODS_COUNT / 2;
static constexpr int INITIAL_ANGLE = -1;
static constexpr int INITIAL_NEXT_CHECKPOINT = 1;

const float FLOAT_MAX_RAND = static_cast<float>(RAND_MAX);

//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------

struct Coords {
	Coords() : 
		x{ INVALID_COORD },
		y{ INVALID_COORD }
	{}

	Coords(int x, int y) :
		x{ x },
		y{ y }
	{}

	int x; ///< X Coordinate
	int y; ///< Y Coordinate
};

//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------

/// Holds information for the checkpoints
class Track {
public:
	Track();

	/// Add the checkopoint, with the given coordinate,
	/// @param[in] checkpointX the X coordinate of the checkpoint ot add
	/// @param[in] checkpointY the Y coordinate of the checkpoint ot add
	void addCheckpoint(const int checkpointX, const int checkpointY);

private:
	Coords checkpoints[MAX_CHECKPOINTS_COUNT]; ///< Checkpoints spread on the track
	int checkpointsCount; ///< How much checkpoints are there on the track
};

//*************************************************************************************************************
//*************************************************************************************************************

Track::Track() :
	checkpointsCount{0}
{
}

//*************************************************************************************************************
//*************************************************************************************************************

void Track::addCheckpoint(const int checkpointX, const int checkpointY) {
	checkpoints[checkpointsCount] = { checkpointX, checkpointY };
	++checkpointsCount;
}

//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------

/// Represents a pod, which partisipates in the race
class Pod {
public:
	Pod();

	/// Reset the pod to its initial state
	void reset();

	/// Fill pod data
	/// @param[in] x the X coordinate of the Pod
	/// @param[in] y the Y coordinate of the Pod
	/// @param[in] vx the X coordinate of the velocity of the Pod
	/// @param[in] vy the Y coordinate of the velocity of the Pod
	/// @param[in] angle the angle of the Pod
	/// @param[in] nextCheckPointId the id of the next checkpoint, which the Pod must go throug
	void fillData(
		const int x,
		const int y,
		const int vx,
		const int vy,
		const int angle,
		const int nextCheckPointId
	);

private:
	Coords initialTurnPosition; ///< Where the pod starts the turn
	Coords position; ///< Where it is on the track
	Coords initialTurnVelocity; ///< The speed vector of the Pod at the start of the turn
	Coords velocity; ///< The speed vector of the Pod
	int initialTurnAngle; ///< The facing angle of the Pod at the start of the turn
	int angle; ///< The facing angle of the Pod
	int initialTurnNextCheckopoint; ///< The id of the checkopoint, which the Pod must cross next at the start of the turn
	int nextCheckopoint; ///< The id of the checkopoint, which the Pod must cross next

	unsigned int flags; ///< Flags need for the simulation and the search algorithm
};

//*************************************************************************************************************
//*************************************************************************************************************

Pod::Pod() :
	initialTurnAngle{ INITIAL_ANGLE },
	angle{ INITIAL_ANGLE },
	initialTurnNextCheckopoint{ INITIAL_NEXT_CHECKPOINT },
	nextCheckopoint{ INITIAL_NEXT_CHECKPOINT },
	flags{ 0 }
{
}

//*************************************************************************************************************
//*************************************************************************************************************

void Pod::reset() {
	position = initialTurnPosition;
	velocity = initialTurnVelocity;
	angle = initialTurnAngle;
	nextCheckopoint = initialTurnNextCheckopoint;
	flags = 0;
}

//*************************************************************************************************************
//*************************************************************************************************************

void Pod::fillData(
	const int x,
	const int y,
	const int vx,
	const int vy,
	const int angle,
	const int nextCheckPointId
)
{
	this->initialTurnPosition = { x, y };
	this->position = { x, y };
	this->initialTurnVelocity= { vx, vy };
	this->velocity = { vx, vy };
	this->initialTurnAngle = angle;
	this->angle = angle;
	this->initialTurnNextCheckopoint = nextCheckPointId;
	this->nextCheckopoint= nextCheckPointId;
}

//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------

/// Represents the whole race, holds information for the track, the pods, simulate pods and performs minimax
class RaceSimulator {
public:

	/// Add the checkopoint, with the given coordinate, to the track
	/// @param[in] checkpointX the X coordinate of the checkpoint ot add
	/// @param[in] checkpointY the Y coordinate of the checkpoint ot add
	void addCheckpoint(const int checkpointX, const int checkpointY);

	/// Fill data for podIdxth pod
	/// @param[in] podIdx the index of the Pod to be filled
	/// @param[in] x the X coordinate of the Pod
	/// @param[in] y the Y coordinate of the Pod
	/// @param[in] vx the X coordinate of the velocity of the Pod
	/// @param[in] vy the Y coordinate of the velocity of the Pod
	/// @param[in] angle the angle of the Pod
	/// @param[in] nextCheckPointId the id of the next checkpoint, which the Pod must go throug
	void fillPodData(
		const int podIdx,
		const int x,
		const int y,
		const int vx,
		const int vy,
		const int angle,
		const int nextCheckPointId
	);

private:
	Pod pods[PODS_COUNT]; ///< Pods participating in the race
	Track track; ///< The track on which the race is performed
	// Simulate
	// Minimax
};

//*************************************************************************************************************
//*************************************************************************************************************

void RaceSimulator::addCheckpoint(const int checkpointX, const int checkpointY) {
	track.addCheckpoint(checkpointX, checkpointY);
}

//*************************************************************************************************************
//*************************************************************************************************************

void RaceSimulator::fillPodData(
	const int podIdx,
	const int x,
	const int y,
	const int vx,
	const int vy,
	const int angle,
	const int nextCheckPointId
)
{
	pods[podIdx].fillData(x, y, vx, vy, angle, nextCheckPointId);
}

//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------

class Game {
public:
	Game();
	~Game();

	void initGame();
	void gameBegin();
	void gameEnd();
	void gameLoop();
	void getGameInput();
	void getTurnInput();
	void turnBegin();
	void makeTurn();
	void turnEnd();
	void play();

	void debug() const;

private:
	// Game specific members
	RaceSimulator raceSimulator; ///< Whole race manager

	int turnsCount;
	int stopGame;
};

//*************************************************************************************************************
//*************************************************************************************************************

Game::Game() :
	turnsCount{ 0 },
	stopGame{ false }
{

}

//*************************************************************************************************************
//*************************************************************************************************************

Game::~Game() {
}

//*************************************************************************************************************
//*************************************************************************************************************

void Game::initGame() {
}

//*************************************************************************************************************
//*************************************************************************************************************

void Game::gameBegin() {
}

//*************************************************************************************************************
//*************************************************************************************************************

void Game::gameEnd() {
}

//*************************************************************************************************************
//*************************************************************************************************************

void Game::gameLoop() {
	while (!stopGame) {
		getTurnInput();
		turnBegin();
		makeTurn();
		turnEnd();

#ifdef DEBUG_ONE_TURN
		break;
#endif // DEBUG_ONE_TURN
	}
}

//*************************************************************************************************************
//*************************************************************************************************************

void Game::getGameInput() {
	int laps;
	cin >> laps; cin.ignore();

#ifdef OUTPUT_GAME_DATA
	cerr << laps << endl;
#endif // OUTPUT_GAME_DATA

	int checkpointCount;
	cin >> checkpointCount; cin.ignore();

#ifdef OUTPUT_GAME_DATA
	cerr << checkpointCount << endl;
#endif // OUTPUT_GAME_DATA

	for (int i = 0; i < checkpointCount; i++) {
		int checkpointX;
		int checkpointY;
		cin >> checkpointX >> checkpointY; cin.ignore();

#ifdef OUTPUT_GAME_DATA
		cerr << checkpointX << SPACE << checkpointY << endl;
#endif // OUTPUT_GAME_DATA

		raceSimulator.addCheckpoint(checkpointX, checkpointY);
	}
}

//*************************************************************************************************************
//*************************************************************************************************************

void Game::getTurnInput() {
	for (int i = 0; i < 2; i++) {
		int x; // x position of your pod
		int y; // y position of your pod
		int vx; // x speed of your pod
		int vy; // y speed of your pod
		int angle; // angle of your pod
		int nextCheckPointId; // next check point id of your pod
		cin >> x >> y >> vx >> vy >> angle >> nextCheckPointId; cin.ignore();

#ifdef OUTPUT_GAME_DATA
		cerr << x << SPACE << y << SPACE << vx << SPACE << vy << SPACE << angle << SPACE << nextCheckPointId << endl;
#endif // OUTPUT_GAME_DATA

		raceSimulator.fillPodData(i, x, y, vx, vy, angle, nextCheckPointId);
	}

	for (int i = 0; i < 2; i++) {
		int x2; // x position of the opponent's pod
		int y2; // y position of the opponent's pod
		int vx2; // x speed of the opponent's pod
		int vy2; // y speed of the opponent's pod
		int angle2; // angle of the opponent's pod
		int nextCheckPointId2; // next check point id of the opponent's pod
		cin >> x2 >> y2 >> vx2 >> vy2 >> angle2 >> nextCheckPointId2; cin.ignore();

#ifdef OUTPUT_GAME_DATA
		cerr << x2 << SPACE << y2 << SPACE << vx2 << SPACE << vy2 << SPACE << angle2 << SPACE << nextCheckPointId2 << endl;
#endif // OUTPUT_GAME_DATA

		raceSimulator.fillPodData(TEAM_PODS_COUNT + i, x2, y2, vx2, vy2, angle2, nextCheckPointId2);
	}
}

//*************************************************************************************************************
//*************************************************************************************************************

void Game::turnBegin() {
}

//*************************************************************************************************************
//*************************************************************************************************************

void Game::makeTurn() {
	cout << "8000 4500 100" << endl;
	cout << "8000 4500 100" << endl;
}

//*************************************************************************************************************
//*************************************************************************************************************

void Game::turnEnd() {
	++turnsCount;
}

//*************************************************************************************************************
//*************************************************************************************************************

void Game::play() {
	initGame();
	getGameInput();
	gameBegin();
	gameLoop();
	gameEnd();
}

//*************************************************************************************************************
//*************************************************************************************************************

void Game::debug() const {
}

//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------

#ifdef TESTS
#include "debug.h"
#endif // TESTS

int main(int argc, char** argv) {
#ifdef TESTS
	doctest::Context context;
	int res = context.run();
#else

#ifdef REDIRECT_INPUT
	cerr << endl << endl << endl << "!!! REDIRECT_INPUT !!!" << endl << endl << endl;
	ifstream in(INPUT_FILE_NAME);
	streambuf *cinbuf = cin.rdbuf();
	cin.rdbuf(in.rdbuf());

	ofstream out(OUTPUT_FILE_NAME);
	streambuf *coutbuf = cout.rdbuf();
	cout.rdbuf(out.rdbuf());
#endif // REDIRECT_INPUT

#ifdef TIME_MEASURERMENT
	chrono::steady_clock::time_point begin = chrono::steady_clock::now();
#endif // TIME_MEASURERM

	Game game;
	game.play();

#ifdef TIME_MEASURERMENT
	chrono::steady_clock::time_point end = chrono::steady_clock::now();
	cerr << "Execution time: " << chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " [ms]" << std::endl;
#endif // TIME_MEASURERMENT	

#endif // TESTS

	return 0;
}

// Game simulation parameters
/*
seed = 681000254
pod_per_player = 2
pod_timeout = 100
map = 7982 7873 13284 5513 9539 1380 3637 4405
*/
