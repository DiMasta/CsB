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
//#define DEBUG_ONE_TURN
//#define USE_UNIFORM_RANDOM
#define TESTS
#define M_PI 3.14159265358979323846

//static const string INPUT_FILE_NAME = "input.txt";
static const string INPUT_FILE_NAME = "input_classic_track.txt";
static const string OUTPUT_FILE_NAME = "output.txt";
static const string EMPTY_STRING = "";
static const string SPACE = " ";
static const string BOOST = "BOOST";
static const string SHIELD = "SHIELD";

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
static constexpr int CHECKPOINT_RADIUS = 600;
static constexpr int PODS_COUNT = 4;
static constexpr int TEAM_PODS_COUNT = PODS_COUNT / 2;
static constexpr int POD_RADIUS = 400;
static constexpr int INITIAL_ANGLE = -1;
static constexpr int INITIAL_NEXT_CHECKPOINT = 1;
static constexpr int INITIAL_NEXT_CHECKPOINT_TURNS_LEFT = 100;
static constexpr int SHEILD_TURNS = 3;
static constexpr int RACE_LAPS = 3;
static constexpr int BOOST_THRUST = 650;

static constexpr unsigned int THRUST_MASK		= 0b0000'0000'0000'0000'0000'0000'1111'1111;
static constexpr unsigned int SHIELD_FLAG		= 0b0000'0000'0000'0000'0100'0000'0000'0000;
static constexpr unsigned int BOOST_FLAG		= 0b0000'0000'0000'0000'1000'0000'0000'0000;
static constexpr unsigned int FIRST_TURN_FLAG	= 0b0000'0000'0000'0001'0000'0000'0000'0000;
static constexpr unsigned int WINNER_FLAG		= 0b1000'0000'0000'0000'0000'0000'0000'0000;

static constexpr float MAX_ANGLE_PER_TURN = 18.f;
static constexpr float TURN_START_TIME = 0.f;
static constexpr float TURN_END_TIME = 1.f;
static constexpr float FRICTION = .85f;
static constexpr float HALF_MOMENTUM = 120.f;
static constexpr float MASS_WITH_SHEILD = 10.f;
static constexpr float MASS_WITHOUT_SHEILD = 1.f;

const float FLOAT_MAX_RAND = static_cast<float>(RAND_MAX);

enum class CollisionType {
	INVALID = -1,
	WITH_POD,
	WITH_CHECKPOINT,
};

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

	/// Find the colsest point in the AB line
	/// @param[in] linePointA point A
	/// @param[in] linePointB point B
	/// @return the closest point on the line
	Coords closestPointOnLine(Coords linePointA, Coords linePointB) const;

	friend bool operator==(Coords lhs, Coords rhs);

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

//*************************************************************************************************************
//*************************************************************************************************************

Coords Coords::closestPointOnLine(Coords linePointA, Coords linePointB) const {
	float da = linePointB.y - linePointA.y;
	float db = linePointA.x - linePointB.x;
	float c1 = da * linePointA.x + db * linePointA.y;
	float c2 = -db * x + da * y;
	float det = da * da + db * db;

	Coords clossestPoint;

	if (det != 0) {
		clossestPoint.x = (da * c1 - db * c2) / det;
		clossestPoint.y = (da * c2 + db * c1) / det;
	}
	else {
		// The point is already on the line
		clossestPoint.x = x;
		clossestPoint.y = y;
	}

	return clossestPoint;
}

//*************************************************************************************************************
//*************************************************************************************************************

bool operator==(Coords lhs, Coords rhs) {
	return lhs.x == rhs.x && lhs.y == rhs.y;
}

//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------

class Action {
public:
	Action();
	Action(const Coords target, const string& thrustStr);

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

Action::Action() :
	flags{ 0 }
{
}

//*************************************************************************************************************
//*************************************************************************************************************

Action::Action(const Coords target, const string& thrustStr) :
	target{ target },
	flags{ 0 }
{
	if (BOOST == thrustStr) {
		setFlag(BOOST_FLAG);
	}
	else if (SHIELD == thrustStr) {
		setFlag(SHIELD_FLAG);
	}
	else {
		setThrust(stoi(thrustStr));
	}
}

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

class Collision {
public:
	Collision();
	Collision(int podIdx, int collideObjIdx, float collisionTurnTime, CollisionType type);

	int getEntityAIdx() const { return podIdx; }
	int getEntityBIdx() const { return collideObjIdx; }
	float getCollisionTurnTime() const { return collisionTurnTime; }
	CollisionType getType() const { return type; }

	/// Check if the collision is valid
	/// @return true if the collision is valid
	bool isValid() const;

private:
	int podIdx; ///< The pod index, which is colliding with an object
	int collideObjIdx; ///< The object with which the pod is colliding
	float collisionTurnTime; ///< The turn time when the collison is occurs
	CollisionType type; ///< Flag showing if the pod coollides with Checkpoint
};

static const Collision INVALID_COLLISION{}; ///< Default value

//*************************************************************************************************************
//*************************************************************************************************************

Collision::Collision() :
	podIdx{ INVALID_IDX },
	collideObjIdx{ INVALID_IDX },
	collisionTurnTime{ 0.f },
	type{ CollisionType::INVALID }
{
}

//*************************************************************************************************************
//*************************************************************************************************************

Collision::Collision(int podIdx, int collideObjIdx, float collisionTurnTime, CollisionType type) :
	podIdx{ podIdx },
	collideObjIdx{ collideObjIdx },
	collisionTurnTime{ collisionTurnTime },
	type{ type }
{
}

//*************************************************************************************************************
//*************************************************************************************************************

bool Collision::isValid() const {
	return INVALID_IDX != podIdx && INVALID_IDX != collideObjIdx;
}

//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------

/// Holds information for the checkpoints
class Track {
public:
	Track();
	Track(const Coords(&checkpoints)[MAX_CHECKPOINTS_COUNT], const int checkpointsCount);

	Coords getCheckpoint(const int cpIdx) const { return checkpoints[cpIdx]; }
	int getCheckpointsCount() const { return checkpointsCount; }

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

Track::Track(const Coords(&checkpoints)[MAX_CHECKPOINTS_COUNT], const int checkpointsCount) :
	checkpointsCount(checkpointsCount)
{
	for (int cpIdx = 0; cpIdx < checkpointsCount; ++cpIdx) {
		this->checkpoints[cpIdx] = checkpoints[cpIdx];
	}
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
	Pod(
		const int x,
		const int y,
		const int vx,
		const int vy,
		const int angle,
		const int nextCheckPointId
	);

	void setPosition(const Coords position) { this->position = position; }
	void setVelocity(const Coords velocity) { this->velocity = velocity; }

	Coords getPosition() const { return position; }
	Coords getVelocity() const { return velocity; }
	float getAngle() const { return angle; }
	int getNextCheckopoint() const { return nextCheckopoint; }

	/// Initialize the Pod with default parameters
	void init();

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

	/// The Pod reached its next checkpoint, update the information
	/// @param[in] chekpointsCountOnTrack the count of checkpoints on the track
	void nextCPReached(const int chekpointsCountOnTrack);

	/// Compute the bounce after the pod collides with another pod
	/// @param[in] collidePod the pod, with which the collision happens
	void computeBounce(Pod& collidePod);

	/// Manage the shield variables
	void manageSheild();

	/// Correctly truncate the float parameter to match the calculations of the online platform
	/// @param[in] toTruncate the float parameter to truncate
	float truncate(const float toTruncate);

	/// Conclude the turn
	void endTurn();

	/// Flags helpers
	void setFlag(const unsigned int flag);
	void unsetFlag(const unsigned int flag);
	bool hasFlag(const unsigned int flag) const;

	friend bool operator==(const Pod& lhs, const Pod& rhs);

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

	int initialTurnPassedCheckpoints; ///< How many checkpoints the Pod has passed at the start of the turn
	int passedCheckpoints; ///< How many checkpoints the Pod has passed

	int initialTurnTurnsLeft; ///< How many turns left for the shuttle to pass the next checkpoint at the start of the turn
	int turnsLeft; ///< How many turns left for the shuttle to pass the next checkpoint

	unsigned int initialTurnFlags; ///< Flags need for the simulation and the search algorithm at the start of the turn
	unsigned int flags; ///< Flags need for the simulation and the search algorithm
};

//*************************************************************************************************************
//*************************************************************************************************************

Pod::Pod() {
	init();
}

//*************************************************************************************************************
//*************************************************************************************************************

Pod::Pod(
	const int x,
	const int y,
	const int vx,
	const int vy,
	const int angle,
	const int nextCheckPointIdArg
) {
	init();
	fillData(x, y, vx, vy, angle, nextCheckPointIdArg);
}

//*************************************************************************************************************
//*************************************************************************************************************

void Pod::init() {
	initialTurnAngle = INITIAL_ANGLE;
	angle = INITIAL_ANGLE;
	initialTurnNextCheckopoint = INITIAL_NEXT_CHECKPOINT;
	nextCheckopoint = INITIAL_NEXT_CHECKPOINT;
	initialSheildTurnsLeft = 0;
	sheildTurnsLeft = 0;
	initialTurnTurnsLeft = INITIAL_NEXT_CHECKPOINT_TURNS_LEFT;
	initialTurnPassedCheckpoints = 0;
	passedCheckpoints = 0;
	turnsLeft = INITIAL_NEXT_CHECKPOINT_TURNS_LEFT;
	initialTurnFlags = 0;
	flags = 0;
}

//*************************************************************************************************************
//*************************************************************************************************************

void Pod::reset() {
	position = initialTurnPosition;
	velocity = initialTurnVelocity;
	angle = initialTurnAngle;
	nextCheckopoint = initialTurnNextCheckopoint;
	sheildTurnsLeft = initialSheildTurnsLeft;
	turnsLeft = initialTurnTurnsLeft;
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

	// TODO:
	// shield
	// turnsLeft
}

//*************************************************************************************************************
//*************************************************************************************************************

void Pod::rotate(const Coords target) {
	float angleToTurn = calcDircetionToTurn(target);

	if (!hasFlag(FIRST_TURN_FLAG)) {
		// Can't turn by more than 18 in one turn
		if (angleToTurn > MAX_ANGLE_PER_TURN) {
			angleToTurn = MAX_ANGLE_PER_TURN;
		}
		else if (angleToTurn < -MAX_ANGLE_PER_TURN) {
			angleToTurn = -MAX_ANGLE_PER_TURN;
		}
	}
	else {
		// Only the first pod could rotate without limits
		unsetFlag(FIRST_TURN_FLAG);
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
	position.x += velocity.x * time;
	position.y += velocity.y * time;
}

//*************************************************************************************************************
//*************************************************************************************************************

void Pod::applyAction(Action action) {
	rotate(action.getTarget());

	if (action.hasFlag(SHIELD_FLAG)) {
		activateShield();
	}

	int thrustToApply = action.getThrust();
	if (action.hasFlag(SHIELD_FLAG)) {
		thrustToApply = 0;
	}	
	else if (action.hasFlag(BOOST_FLAG)) {
		thrustToApply = BOOST_THRUST;
	}

	applyThrust(thrustToApply);
}

//*************************************************************************************************************
//*************************************************************************************************************

void Pod::nextCPReached(const int chekpointsCountOnTrack) {
	++passedCheckpoints;
	nextCheckopoint = passedCheckpoints % chekpointsCountOnTrack;

	// Fisrt time the 0th checkpoint does not counts
	if (passedCheckpoints >= (RACE_LAPS * chekpointsCountOnTrack) - 1) {
		setFlag(WINNER_FLAG); // :)
	}
}

//*************************************************************************************************************
//*************************************************************************************************************

void Pod::computeBounce(Pod& entity) {
	// If a pod has its shield active its mass is 10 otherwise it's 1
	float m1 = hasFlag(SHIELD_FLAG) ? MASS_WITH_SHEILD : MASS_WITHOUT_SHEILD;
	float m2 = entity.hasFlag(SHIELD_FLAG) ? MASS_WITH_SHEILD : MASS_WITHOUT_SHEILD;
	float mcoeff = (m1 + m2) / (m1 * m2);

	float nx = position.x - entity.getPosition().x;
	float ny = position.y - entity.getPosition().y;

	// Square of the distance between the 2 pods. This value could be hardcoded because it is always 800
	float nxnysquare = nx * nx + ny * ny;

	float dvx = velocity.x - entity.getVelocity().x;
	float dvy = velocity.y - entity.getVelocity().y;

	// fx and fy are the components of the impact vector. product is just there for optimisation purposes
	float product = nx * dvx + ny * dvy;
	float fx = (nx * product) / (nxnysquare * mcoeff);
	float fy = (ny * product) / (nxnysquare * mcoeff);

	// We apply the impact vector once
	velocity.x -= fx / m1;
	velocity.y -= fy / m1;
	entity.setVelocity(
		Coords(
			entity.getVelocity().x + fx / m2,
			entity.getVelocity().y + fy / m2
		)
	);

	// If the norm of the impact vector is less than 120, we normalize it to 120
	float impulse = sqrt(fx*fx + fy * fy);
	if (impulse < HALF_MOMENTUM) {
		fx = fx * HALF_MOMENTUM / impulse;
		fy = fy * HALF_MOMENTUM / impulse;
	}

	// We apply the impact vector a second time
	velocity.x -= fx / m1;
	velocity.y -= fy / m1;
	entity.setVelocity(
		Coords(
			entity.getVelocity().x + fx / m2,
			entity.getVelocity().y + fy / m2
		)
	);

	// This is one of the rare places where a Vector class would have made the code more readable.
	// But this place is called so often that I can't pay a performance price to make it more readable.
}

//*************************************************************************************************************
//*************************************************************************************************************

void Pod::manageSheild() {
	if (hasFlag(SHIELD_FLAG)) {
		if (sheildTurnsLeft > 0) {
			--sheildTurnsLeft;
		}
		else {
			unsetFlag(SHIELD_FLAG);
		}
	}
}

//*************************************************************************************************************
//*************************************************************************************************************

float Pod::truncate(const float toTruncate) {
	float res = floor(toTruncate);

	if (toTruncate < 0) {
		res = ceil(toTruncate);
	}

	return res;
}

//*************************************************************************************************************
//*************************************************************************************************************

void Pod::endTurn() {
	position.x = round(position.x);
	position.y = round(position.y);
	velocity.x = truncate(velocity.x * FRICTION);
	velocity.y = truncate(velocity.y * FRICTION);
	//angle = round(angle);

	// Don't forget that the timeout goes down by 1 each turn. It is reset to 100 when you pass a checkpoint
	--turnsLeft;

	manageSheild();
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

//*************************************************************************************************************
//*************************************************************************************************************

bool operator==(const Pod& lhs, const Pod& rhs) {
	return
		lhs.position == rhs.position &&
		lhs.velocity == rhs.velocity;
}

//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------

/// Represents the whole race, holds information for the track, the pods, simulate pods and performs minimax
class RaceSimulator {
public:
	RaceSimulator() = default;
	RaceSimulator(const Pod(&pods)[PODS_COUNT], const Track& track);

	const Pod& getPod(const int podIdx) const { return pods[podIdx]; };

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
	/// @param[in] firstTurn true if the simulation is for the first turn
	void simulate(const vector<vector<Action>>& turnActions, const bool firstTurn);

	/// Simulate pods for the given actions
	/// @param podsActiona commands for all pods
	void simulatePods(const vector<Action>& podsActions);

	/// Move all pods at once, considering their parameters
	void movePods();

	/// Check if entityA collides in entityB
	/// @param[in] entityAIdx the index of the pod which is checked for collision
	/// @param[in] entityBIdx the index of the pod or checkpoint in which entityAIdx could collide
	/// @param[in] collType type of the collision to check if WITH_CHECKPOINT entityB is a checkpoint
	/// @return the collision between the entities if they collide, invalid one otherwise
	Collision checkForCollision(const int entityAIdx, const int entityBIdx, const CollisionType collType);

	/// Comapare the given collisions
	/// @param[in] collisionA the first collision
	/// @param[in] collisionB the second collision
	/// @return true if the collisions are identical false otherwise
	bool compareCollisions(const Collision& collisionA, const Collision& collisionB);

private:
	Pod pods[PODS_COUNT]; ///< Pods participating in the race
	Track track; ///< The track on which the race is performed
	// Minimax
};

//*************************************************************************************************************
//*************************************************************************************************************

RaceSimulator::RaceSimulator(const Pod(&pods)[PODS_COUNT], const Track& track) :
	track(track)
{
	for (int podIdx = 0; podIdx < PODS_COUNT; ++podIdx) {
		this->pods[podIdx] = pods[podIdx];
	}
}

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

void RaceSimulator::simulate(const vector<vector<Action>>& turnActions, const bool firstTurn) {
	for (int podActionIdx = 0; podActionIdx < PODS_COUNT; ++podActionIdx) {
		pods[podActionIdx].reset();

		if (firstTurn) {
			pods[podActionIdx].setFlag(FIRST_TURN_FLAG);
		}
	}
	
	for (size_t actionIdx = 0; actionIdx < turnActions.size(); ++actionIdx) {
		simulatePods(turnActions[actionIdx]);
	}
}

//*************************************************************************************************************
//*************************************************************************************************************

void RaceSimulator::simulatePods(const vector<Action>& podsActions) {
	for (int podActionIdx = 0; podActionIdx < PODS_COUNT; ++podActionIdx) {
		pods[podActionIdx].applyAction(podsActions[podActionIdx]);
	}

	// Move all pods simulataniously after their actions are applied
	movePods();

	for (int podActionIdx = 0; podActionIdx < PODS_COUNT; ++podActionIdx) {
		pods[podActionIdx].endTurn();
	}
}

//*************************************************************************************************************
//*************************************************************************************************************

void RaceSimulator::movePods() {
	// This tracks the time during the turn. The goal is to reach 1.0
	float t = TURN_START_TIME;

	Collision previousCollision;
	Collision firstCollision;

	while (t < TURN_END_TIME) {
		// We look for all the collisions that are going to occur during the turn
		for (int i = 0; i < PODS_COUNT; ++i) {
			// Collision with another pod?
			for (int j = i + 1; j < PODS_COUNT; ++j) {
				Collision col = checkForCollision(i, j, CollisionType::WITH_POD);

				if (col.isValid() && TURN_START_TIME == col.getCollisionTurnTime() && compareCollisions(previousCollision, col)) {
					col = INVALID_COLLISION;
				}

				// If the collision occurs earlier than the one we currently have we keep it
				if (col.isValid() && col.getCollisionTurnTime() + t < TURN_END_TIME &&
					(!firstCollision.isValid() || col.getCollisionTurnTime() < firstCollision.getCollisionTurnTime())) {
					firstCollision = col;
				}
			}

			// Collision with another checkpoint?
			// It is unnecessary to check all checkpoints here. We only test the pod's next checkpoint.
			// We could look for the collisions of the pod with all the checkpoints, but if such a collision happens it wouldn't impact the game in any way
			Collision col = checkForCollision(i, pods[i].getNextCheckopoint(), CollisionType::WITH_CHECKPOINT);

			if (col.isValid() && previousCollision.isValid() && /*TURN_START_TIME == col->getCollisinTurnTime() &&*/ compareCollisions(previousCollision, col)) {
				col = INVALID_COLLISION;
			}

			// If the collision happens earlier than the current one we keep it
			if (col.isValid() && col.getCollisionTurnTime() + t < TURN_END_TIME &&
				(!firstCollision.isValid() || col.getCollisionTurnTime() < firstCollision.getCollisionTurnTime())) {
				firstCollision = col;
			}
		}

		if (!firstCollision.isValid()) {
			// No collision, we can move the pods until the end of the turn
			for (int i = 0; i < PODS_COUNT; ++i) {
				pods[i].move(TURN_END_TIME - t);
			}

			// End of the turn
			t = TURN_END_TIME;
		}
		else {
			// Move the pods to reach the time t of the collision
			for (int i = 0; i < PODS_COUNT; ++i) {
				pods[i].move(firstCollision.getCollisionTurnTime());
			}

			if (CollisionType::WITH_CHECKPOINT == firstCollision.getType()) {
				// EntityA is pod which collides with the checkpoint
				pods[firstCollision.getEntityAIdx()].nextCPReached(track.getCheckpointsCount());
			}
			else {
				// Play out the collision
				pods[firstCollision.getEntityAIdx()].computeBounce(pods[firstCollision.getEntityBIdx()]);
			}

			t += firstCollision.getCollisionTurnTime();
		}

		previousCollision = firstCollision;
		firstCollision = INVALID_COLLISION;
	}
}

//*************************************************************************************************************
//*************************************************************************************************************

Collision RaceSimulator::checkForCollision(const int entityAIdx, const int entityBIdx, const CollisionType collType) {
	const Coords entityAPosition = pods[entityAIdx].getPosition(); // EnityA is always a Pod
	const Coords entityAVelocity = pods[entityAIdx].getVelocity(); // EnityA is always a Pod
	const int entityARadius = POD_RADIUS;
	Coords entityBPosition; // EntityB could be a Pod or a Checkpoint
	Coords entityBVelocity; // EntityB could be a Pod or a Checkpoint
	int entityBRadius = 0;

	if (CollisionType::WITH_POD == collType) {
		entityBPosition = pods[entityBIdx].getPosition();
		entityBVelocity = pods[entityBIdx].getVelocity();
		entityBRadius = POD_RADIUS;
	}
	else {
		entityBPosition = track.getCheckpoint(entityBIdx);
		entityBVelocity = Coords(0.f, 0.f);
		entityBRadius = CHECKPOINT_RADIUS;
	}

	// Square of the distance
	float dist = entityAPosition.distanceSquare(entityBPosition);

	// Sum of the radii squared
	float sr = (float)(entityARadius + entityBRadius) * (entityARadius + entityBRadius);

	// We take everything squared to avoid calling sqrt uselessly. It is better for performances

	if (dist < sr) {
		// Objects are already touching each other. We have an immediate collision.
		return Collision(entityAIdx, entityBIdx, 0.f, collType);
	}

	// Optimisation. Objects with the same speed will never collide
	if (entityAVelocity.x == entityBVelocity.x &&
		entityAVelocity.y == entityBVelocity.y
		) {
		return INVALID_COLLISION;
	}

	// We place ourselves in the reference frame of u. u is therefore stationary and is at (0,0)
	float x = entityAPosition.x - entityBPosition.x;
	float y = entityAPosition.y - entityBPosition.y;
	Coords myp(x, y);
	float vx = entityAVelocity.x - entityBVelocity.x;
	float vy = entityAVelocity.y - entityBVelocity.y;
	Coords up(0.f, 0.f);

	// We look for the closest point to u (which is in (0,0)) on the line described by our speed vector
	Coords p = up.closestPointOnLine(myp, Coords(x + vx, y + vy));

	// Square of the distance between u and the closest point to u on the line described by our speed vector
	float pdist = up.distanceSquare(p);

	// Square of the distance between us and that point
	float mypdist = myp.distanceSquare(p);

	// If the distance between u and this line is less than the sum of the radii, there might be a collision
	if (pdist < sr) {
		// Our speed on the line
		float length = sqrt(vx * vx + vy * vy);

		// We move along the line to find the point of impact
		float backdist = sqrt(sr - pdist);
		p.x = p.x - backdist * (vx / length);
		p.y = p.y - backdist * (vy / length);

		// If the point is now further away it means we are not going the right way, therefore the collision won't happen
		if (myp.distanceSquare(p) > mypdist) {
			return INVALID_COLLISION;
		}

		pdist = p.distance(myp);

		// The point of impact is further than what we can travel in one turn
		if (pdist > length) {
			return INVALID_COLLISION;
		}

		// Time needed to reach the impact point
		float t = pdist / length;
	
		return Collision(entityAIdx, entityBIdx, t, collType);
	}

	return INVALID_COLLISION;
}

//*************************************************************************************************************
//*************************************************************************************************************

bool RaceSimulator::compareCollisions(const Collision& collisionA, const Collision& collisionB) {
	const bool identicalTypes = collisionA.getType() == collisionB.getType();
	const bool identicalAEntities = collisionA.getEntityAIdx() == collisionB.getEntityAIdx();
	const bool identicalBEntities = collisionA.getEntityBIdx() == collisionB.getEntityBIdx();

	return identicalTypes && identicalAEntities && identicalBEntities;
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
	if (0 == turnsCount) {
		cout << "13284 5513 BOOST" << endl;
	}
	else {
		cout << "13284 5513 100" << endl;
	}

	cout << "13284 5513 100" << endl;
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
