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

//#define REDIRECT_INPUT
//#define OUTPUT_GAME_DATA
//#define TIME_MEASURERMENT
#define DEBUG_ONE_TURN
//#define USE_UNIFORM_RANDOM
#define M_PI 3.14159265358979323846

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
static constexpr int SHEILD_TURNS = 3;

static constexpr unsigned int THRUST_MASK = 0b0000'0000'0000'0000'0000'0000'1111'1111;
static constexpr unsigned int SHIELD_FLAG = 0b0000'0000'0000'0000'0100'0000'0000'0000;
static constexpr unsigned int BOOST_FLAG  = 0b0000'0000'0000'0000'1000'0000'0000'0000;

static constexpr float MAX_ANGLE_PER_TURN = 18.f;

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
		x{ static_cast<float>(x) },
		y{ static_cast<float>(y) }
	{}

	Coords(float x, float y) :
		x{ x },
		y{ y }
	{}

	/// Calculate the square of the distance to the given point
	/// @param[in] point the point to which to calculate distance
	/// @return the square of the distance to point
	float distanceSquare(const Coords point) const;

	/// Calculate the distance to the given point
	/// @param[in] point the point to which to calculate distance
	/// @return the distance to point
	float distance(const Coords point) const;

	float x; ///< X Coordinate
	float y; ///< Y Coordinate
};

//*************************************************************************************************************
//*************************************************************************************************************

float Coords::distanceSquare(Coords point) const {
	return
		(x - point.x) * (x - point.x) +
		(y - point.y) * (y - point.y);
}

//*************************************************************************************************************
//*************************************************************************************************************

float Coords::distance(Coords point) const {
	return sqrt(distanceSquare(point));
}

//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------

class Action {
public:
	void setTarget(const Coords traget) { this->target = target; }
	Coords getTarget() const { return target; }

	/// Update the bits for the thrust power
	/// @param[in] thrust the thrust power
	void setThrust(const int thrust);

	/// Extract the thrust value from flags
	/// @return the integer for the thrust
	int getThrust() const;

	/// Flags helpers
	void setFlag(const unsigned int flag);
	void unsetFlag(const unsigned int flag);
	bool hasFlag(const unsigned int flag) const;

private:
	Coords target; ///< Target point for the pod
	unsigned int flags; ///< Thrust(in the first 8 bits) and flags for shield and boost
};

//*************************************************************************************************************
//*************************************************************************************************************

void Action::setThrust(const int thrust) {
	flags &= ~THRUST_MASK; // First zero out only the bits for the thrust
	flags |= static_cast<unsigned int>(thrust); // Assign the new thrust
}

//*************************************************************************************************************
//*************************************************************************************************************

int Action::getThrust() const {
	return static_cast<int>(THRUST_MASK & flags);
}

//*************************************************************************************************************
//*************************************************************************************************************

void Action::setFlag(const unsigned int flag) {
	flags |= flag;
}

//*************************************************************************************************************
//*************************************************************************************************************

void Action::unsetFlag(const unsigned int flag) {
	flags &= ~flag;
}

//*************************************************************************************************************
//*************************************************************************************************************

bool Action::hasFlag(const unsigned int flag) const {
	return flag & flags;
}

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

	/// Rotate Pod towards the given target, obeying the rules for rotation
	/// @param[in] target the target point towards which to rotate the Pod
	void rotate(const Coords target);

	/// Activate Shield which will hold for 3 turns
	void activateShield();

	/// Apply the given thrust power to the speed of the Pod
	/// @param[in] thrust the thrust power to apply
	void applyThrust(const int thrust);

	/// Move the pod for the given period of time
	/// @param[in] time the period of time to consider 1.f means the whole turn
	void move(const float time);

	/// Apply the given action, updating all pods parameters for the given action
	/// @param[in] action the action to apply
	void applyAction(Action action);

	/// Calculate angle, clamped and clock oriented to turn towards the given target point
	/// @param[in] target the point towards which to turn
	/// @return the angle for rotation
	float calcDircetionToTurn(const Coords target) const;

	/// Calculate angle to turn towards the given target point
	/// @param[in] target the point towards which to turn
	/// @return the angle for rotation
	float calcAngleToTarget(const Coords target) const;

	/// Flags helpers
	void setFlag(const unsigned int flag);
	void unsetFlag(const unsigned int flag);
	bool hasFlag(const unsigned int flag) const;

private:
	Coords initialTurnPosition; ///< Where the pod starts the turn
	Coords position; ///< Where it is on the track
	Coords initialTurnVelocity; ///< The speed vector of the Pod at the start of the turn
	Coords velocity; ///< The speed vector of the Pod
	float initialTurnAngle; ///< The facing angle of the Pod at the start of the turn
	float angle; ///< The facing angle of the Pod
	int initialTurnNextCheckopoint; ///< The id of the checkopoint, which the Pod must cross next at the start of the turn
	int nextCheckopoint; ///< The id of the checkopoint, which the Pod must cross next
	int initialSheildTurnsLeft; ///< How many turns are left for the shield at the start of the turn
	int sheildTurnsLeft; ///< How many turns are left for the shield
	unsigned int initialTurnFlags; ///< Flags need for the simulation and the search algorithm at the start of the turn
	unsigned int flags; ///< Flags need for the simulation and the search algorithm
};

//*************************************************************************************************************
//*************************************************************************************************************

Pod::Pod() :
	initialTurnAngle{ INITIAL_ANGLE },
	angle{ INITIAL_ANGLE },
	initialTurnNextCheckopoint{ INITIAL_NEXT_CHECKPOINT },
	nextCheckopoint{ INITIAL_NEXT_CHECKPOINT },
	initialTurnFlags{ 0 },
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
	sheildTurnsLeft = initialSheildTurnsLeft;
	flags = initialTurnFlags;
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
	this->initialTurnAngle = static_cast<float>(angle);
	this->angle = static_cast<float>(angle);
	this->initialTurnNextCheckopoint = nextCheckPointId;
	this->nextCheckopoint= nextCheckPointId;
}

//*************************************************************************************************************
//*************************************************************************************************************

void Pod::rotate(const Coords target) {
	float angleToTurn = calcDircetionToTurn(target);

	// Can't turn by more than 18 in one turn
	if (angleToTurn > MAX_ANGLE_PER_TURN) {
		angleToTurn = MAX_ANGLE_PER_TURN;
	}
	else if (angleToTurn < -MAX_ANGLE_PER_TURN) {
		angleToTurn = -MAX_ANGLE_PER_TURN;
	}

	angle += angleToTurn;

	// The % operator is slow. If we can avoid it, it's better.
	if (angle >= 360.f) {
		angle = angle - 360.f;
	}
	else if (angle < 0.f) {
		angle += 360.f;
	}
}

//*************************************************************************************************************
//*************************************************************************************************************

float Pod::calcDircetionToTurn(const Coords target) const {
	float direction = 0.f;

	float angleToTarget = calcAngleToTarget(target);

	// To know whether we should turn clockwise or not we look at the two ways and keep the smallest
	// The ternary operators replace the use of a modulo operator which would be slower
	float clockwise = angle <= angleToTarget ? angleToTarget - angle : 360.f - angle + angleToTarget;
	float counterClockWise = angle >= angleToTarget ? angle - angleToTarget : angle + 360.f - angleToTarget;

	if (clockwise < counterClockWise) {
		direction = clockwise;
	}
	else {
		// We return a negative angle if we must rotate counterClockWise
		direction = -counterClockWise;
	}

	return direction;
}

//*************************************************************************************************************
//*************************************************************************************************************

float Pod::calcAngleToTarget(const Coords target) const {
	float distanceToTarget = position.distance(target);
	float dx = (target.x - position.x) / distanceToTarget;
	float dy = (target.y - position.y) / distanceToTarget;

	// Simple trigonometry. We multiply by 180.f / PI to convert radiants to degrees.
	float angleToTarget = acos(dx) * 180.f / static_cast<float>(M_PI);

	// If the point I want is below me, I have to shift the angle for it to be correct
	if (dy < 0.f) {
		angleToTarget = 360.f - angleToTarget;
	}

	return angleToTarget;
}

//*************************************************************************************************************
//*************************************************************************************************************

void Pod::activateShield() {
	setFlag(SHIELD_FLAG);
	sheildTurnsLeft = SHEILD_TURNS;
}

//*************************************************************************************************************
//*************************************************************************************************************

void Pod::applyThrust(const int thrust) {
	// Don't forget that a pod which has activated its shield cannot accelerate for 3 turns
	if (!hasFlag(SHIELD_FLAG)) {
		// Conversion of the angle to radiants
		float facingAngleDeg = angle * static_cast<float>(M_PI) / 180.f;

		// Trigonometry
		velocity.x += cos(facingAngleDeg) * thrust;
		velocity.y += sin(facingAngleDeg) * thrust;
	}
}

//*************************************************************************************************************
//*************************************************************************************************************

void Pod::move(const float time) {

}

//*************************************************************************************************************
//*************************************************************************************************************

void Pod::applyAction(Action action) {
	rotate(action.getTarget());

	if (action.hasFlag(SHIELD_FLAG)) {
		activateShield();
	}

	int thrustToApply = action.getThrust();
	if (hasFlag(SHIELD_FLAG)) {
		thrustToApply = 0;
	}

	applyThrust(thrustToApply);
}

//*************************************************************************************************************
//*************************************************************************************************************

void Pod::setFlag(const unsigned int flag) {
	flags |= flag;
}

//*************************************************************************************************************
//*************************************************************************************************************

void Pod::unsetFlag(const unsigned int flag) {
	flags &= ~flag;
}

//*************************************************************************************************************
//*************************************************************************************************************

bool Pod::hasFlag(const unsigned int flag) const {
	return flag & flags;
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

	/// Simulate the given array of actions, one by one for each turn
	/// @param[in] actions the action for each turn to simulate
	void simulate(const vector<Action[PODS_COUNT]>& turnActions);

	/// Simulate pods for the given actions
	/// @param podsActiona commands for all pods
	void simulatePods(const Action (&podsActions)[PODS_COUNT]);

	/// Move all pods at once, considering their parameters
	void movePods();

private:
	Pod pods[PODS_COUNT]; ///< Pods participating in the race
	Track track; ///< The track on which the race is performed
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

//*************************************************************************************************************
//*************************************************************************************************************

void RaceSimulator::simulate(const vector<Action[PODS_COUNT]>& turnActions) {
	for (size_t actionIdx = 0; actionIdx < turnActions.size(); ++actionIdx) {
		simulatePods(turnActions[actionIdx]);
	}
}

//*************************************************************************************************************
//*************************************************************************************************************

void RaceSimulator::simulatePods(const Action(&podsActions)[PODS_COUNT]) {
	for (int podActionIdx = 0; podActionIdx < PODS_COUNT; ++podActionIdx) {
		pods[podActionIdx].applyAction(podsActions[podActionIdx]);
	}

	// Move all pods simulataniously after their actions are applied
	movePods();
}

//*************************************************************************************************************
//*************************************************************************************************************

void RaceSimulator::movePods() {
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
#ifdef TIME_MEASURERMENT
		chrono::steady_clock::time_point begin = chrono::steady_clock::now();
#endif // TIME_MEASURERMENT

		getTurnInput();
		turnBegin();
		makeTurn();
		turnEnd();

#ifdef TIME_MEASURERMENT
		chrono::steady_clock::time_point end = chrono::steady_clock::now();
		cerr << "Turn[" << turnsCount - 1 << "] execution time: " << chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " [ms]" << std::endl;
#endif // TIME_MEASURERMENT

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
	cerr << "Game execution time: " << chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " [ms]" << std::endl;
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
