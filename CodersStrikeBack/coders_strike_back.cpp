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
#include <climits>
#include <cstring>
#include <fstream>
#include <iterator>
#include <bitset>
#include <cmath>
#include <iomanip>
#include <chrono>

using namespace std;

//#define SVG
//#define REDIRECT_INPUT
//#define OUTPUT_GAME_DATA
#define TIME_MEASURERMENT
//#define DEBUG_ONE_TURN
//#define TESTS
#define M_PI 3.14159265358979323846

#ifdef SVG
#include "svg_manager.h"
#endif // SVG

using ChromEvalIdxMap = std::multimap<float, int>;

//static const string INPUT_FILE_NAME = "input.txt";
//static const string INPUT_FILE_NAME = "input_classic_track.txt";
static const string INPUT_FILE_NAME = "input_triangle.txt";
static const string OUTPUT_FILE_NAME = "output.txt";
static const string SPACE = " ";
static const string BOOST = "BOOST";
static const string SHIELD = "SHIELD";

static constexpr int INVALID_ID = -1;
static constexpr int INVALID_IDX = -1;
static constexpr int PAIR = 2;
static constexpr int TRIPLET = 3;

static constexpr int INVALID_COORD = -1;
static constexpr int MAX_CHECKPOINTS_COUNT = 8;
static constexpr int CHECKPOINT_RADIUS = 600;
static constexpr int PODS_COUNT = 4;
static constexpr int TEAM_PODS_COUNT = PODS_COUNT / 2;
static constexpr int POD_RADIUS = 400;
static constexpr int INITIAL_ANGLE = -1;
static constexpr int INITIAL_NEXT_CHECKPOINT = 1;
static constexpr int INITIAL_CHECKPOINTS_PASSED = 1;
static constexpr int INITIAL_NEXT_CHECKPOINT_TURNS_LEFT = 100;
static constexpr int SHEILD_TURNS = 3;
static constexpr int RACE_LAPS = 3;
static constexpr int MIN_THRUST = 0;
static constexpr int MAX_THRUST = 100;
static constexpr int BOOST_THRUST = 650;

static constexpr unsigned int THRUST_MASK		= 0b0000'0000'0000'0000'0000'0000'1111'1111;

static constexpr unsigned int RUNNER_FLAG		= 0b0000'0000'0000'0000'0000'0001'0000'0000;
static constexpr unsigned int HUNTER_FLAG		= 0b0000'0000'0000'0000'0000'0010'0000'0000;
static constexpr unsigned int SHIELD_FLAG		= 0b0000'0000'0000'0000'0100'0000'0000'0000;

static constexpr float MIN_ANGLE = -18.f;
static constexpr float MAX_ANGLE = 18.f;
static constexpr float MAX_ANGLE_DOUBLED = MAX_ANGLE * 2.f;
static constexpr float TURN_START_TIME = 0.f;
static constexpr float TURN_END_TIME = 1.f;
static constexpr float FRICTION = .85f;
static constexpr float HALF_MOMENTUM = 120.f;
static constexpr float MASS_WITH_SHEILD = 10.f;
static constexpr float MASS_WITHOUT_SHEILD = 1.f;
static constexpr float FLOAT_MAX_RAND = static_cast<float>(RAND_MAX);

/// Weights
static constexpr float PASSED_CPS_WEIGHT		= 50'000.f;
static constexpr float SCORE_DIFF_WEIGHT		= 50.f;
static constexpr float HUNTER_DISTANCE_WEIGHT	= 1.f;
static constexpr float HUNTER_ANGLE_WEIGHT		= 1.f;

/// GA consts
static constexpr int TURNS_TO_SIMULATE = 5;
static constexpr int CHROMOSOME_SIZE = TURNS_TO_SIMULATE * TRIPLET * PAIR; // 3 genes per turn for a pod, first half is for 0th pod second half is for 1st pod
static constexpr int POPULATION_SIZE = 16;
static constexpr int ENEMY_MAX_POPULATION = 32;
static constexpr int MY_MAX_POPULATION = 80;
static constexpr float ELITISM_RATIO = 0.2f; // The perscentage of the best chromosomes to transfer directly to the next population, unchanged, after other operators are done!
static constexpr float PROBABILITY_OF_MUTATION = 0.01f; // The probability to mutate a gene

static constexpr int CHROMOSOME_HALF_SIZE = CHROMOSOME_SIZE / PAIR;
static constexpr float BIAS = 50.f;
static constexpr float FLOAT_MAX = numeric_limits<float>::max();
static constexpr float PLUS_INFINITY = (FLOAT_MAX / POPULATION_SIZE) - BIAS;
static constexpr float MINUS_INFINITY = -PLUS_INFINITY;

/// GA flags
static const unsigned int COPIED_FLAG = 1 << 0;

enum class CollisionType {
	INVALID = -1,
	WITH_POD,
	WITH_CHECKPOINT,
};

enum class Team {
	INVALID = -1,
	MY,
	ENEMY,
};

static float randomFloatBetween0and1() {
	return static_cast<float>(rand()) / FLOAT_MAX_RAND;
}

class Chromosome;

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

	/// Return an std::pair of the coords
	pair<float, float> toPair() const { return { x, y }; }

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

//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------

class Action {
public:
	Action();

	void setAngle(const float angle) { this->angle = angle; }
	float getAngle() const { return angle; }

	/// Based on the given genes, construct the action
	/// @param[in] gene0 the gene value to consider
	/// @param[in] gene1 the gene value to consider
	/// @param[in] gene2 the gene value to consider
	void parseGenes(const float gene0, const float gene1, const float gene2);

	/// Update the bits for the thrust power
	/// @param[in] thrust the thrust power
	void setThrust(const int thrust);

	/// Extract the thrust value from flags
	/// @return the integer for the thrust
	int getThrust() const;

	/// Output the action for in CG format
	/// @param[in] podPosition the current position of the pod
	/// @param[in] podAngle the current angle of the pod
	void output(const Coords podPosition, const float podAngle) const;

	/// Flags helpers
	void setFlag(const unsigned int flag);
	void unsetFlag(const unsigned int flag);
	bool hasFlag(const unsigned int flag) const;

private:
	float angle; ///< Turn angle in the range [-18.f; 18.f]
	unsigned int flags; ///< Thrust(in the first 8 bits) and flags for shield and boost
};

//*************************************************************************************************************
//*************************************************************************************************************

Action::Action() :
	angle{},
	flags{}
{
}

//*************************************************************************************************************
//*************************************************************************************************************

void Action::parseGenes(const float gene0, const float gene1, const float gene2) {
	if (gene0 > 0.95f) {
		setFlag(SHIELD_FLAG);
	}
	else {
		unsetFlag(SHIELD_FLAG);
	}

	if (gene1 < 0.25f) {
		setAngle(MIN_ANGLE);
	}
	else if (gene1 > 0.75f) {
		setAngle(MAX_ANGLE);
	}
	else {
		setAngle(MIN_ANGLE + MAX_ANGLE_DOUBLED * ((gene1 - 0.25f) * 2.f));
	}

	if (gene2 < 0.25f) {
		setThrust(MIN_THRUST);
	}
	else if (gene2 > 0.75f) {
		setThrust(MAX_THRUST);
	}
	else {
		setThrust(static_cast<int>(MAX_THRUST * ((gene2 - 0.25f) * 2.f)));
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

void Action::output(const Coords podPosition, const float podAngle) const {
	float a = podAngle + angle;

	if (a >= 360.f) {
		a = a - 360.f;
	}
	else if (a < 0.f) {
		a += 360.f;
	}

	// Look for a point corresponding to the angle we want
	// Multiply by 10000.0 to limit rounding errors
	a = a * static_cast<float>(M_PI / 180.0);
	float px = podPosition.x + cos(a) * 10000.f;
	float py = podPosition.y + sin(a) * 10000.f;

	if (hasFlag(SHIELD_FLAG)) {
		cout << static_cast<int>(round(px)) << SPACE << static_cast<int>(round(py)) << SPACE << "SHIELD" << endl;
	}
	else {
		cout << static_cast<int>(round(px)) << SPACE << static_cast<int>(round(py)) << SPACE << getThrust() << endl;
	}
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

	void setVelocity(const Coords velocity) { this->velocity = velocity; }
	void setInitalShieldTurns(const int shieldTurns) { this->initialSheildTurnsLeft = shieldTurns; }

	Coords getPosition() const { return position; }
	Coords getInitialTurnPosition() const { return initialTurnPosition; }
	Coords getVelocity() const { return velocity; }
	float getAngle() const { return angle; }
	float getInitialTurnAngle() const { return initialTurnAngle; }
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

	/// Rotate Pod by the given angle
	/// @param[in] rotAngle the rotate angle
	void rotate(float rotAngle);

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
	void turnEnd();

	/// Return true if the pod has won the race
	/// @param[in] chekpointsCountOnTrack the CPs count for the race
	bool winner(const int chekpointsCountOnTrack) const;

	/// Retunr true if the Pod is eliminated form the race
	bool eliminated() const;

	/// Evaluate the pod
	/// @param[in] track the information for the checkpoitns
	/// @return the score from the evaluation
	float score(const Track& track) const;

	/// Flags helpers
	void setFlag(const unsigned int flag, const bool initialTurn = false);
	void unsetFlag(const unsigned int flag, const bool initialTurn = false);
	bool hasFlag(const unsigned int flag, const bool initialTurn = false) const;

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

void Pod::init() {
	initialTurnAngle = INITIAL_ANGLE;
	angle = INITIAL_ANGLE;
	initialTurnNextCheckopoint = INITIAL_NEXT_CHECKPOINT;
	nextCheckopoint = INITIAL_NEXT_CHECKPOINT;
	initialSheildTurnsLeft = 0;
	sheildTurnsLeft = 0;
	initialTurnTurnsLeft = INITIAL_NEXT_CHECKPOINT_TURNS_LEFT;
	initialTurnPassedCheckpoints = INITIAL_CHECKPOINTS_PASSED;
	passedCheckpoints = INITIAL_CHECKPOINTS_PASSED;
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
	passedCheckpoints = initialTurnPassedCheckpoints;
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
	if (nextCheckPointId != this->nextCheckopoint) {
		// Checkpooint reached
		initialTurnTurnsLeft = INITIAL_NEXT_CHECKPOINT_TURNS_LEFT;
		turnsLeft = INITIAL_NEXT_CHECKPOINT_TURNS_LEFT;

		++initialTurnPassedCheckpoints;
		++passedCheckpoints;
	}
	else {
		initialTurnTurnsLeft = turnsLeft; // turnsLeft are managed in turnEnd()
	}

	if (hasFlag(SHIELD_FLAG)) {
		initialSheildTurnsLeft = sheildTurnsLeft;

		if (initialSheildTurnsLeft <= 0) {
			unsetFlag(SHIELD_FLAG, true);
		}
	}

	this->initialTurnPosition = { x, y };
	this->position = { x, y };
	this->initialTurnVelocity= { vx, vy };
	this->velocity = { vx, vy };
	this->initialTurnAngle = static_cast<float>(angle);
	this->angle = static_cast<float>(angle);
	this->initialTurnNextCheckopoint = nextCheckPointId;
	this->nextCheckopoint = nextCheckPointId;
}

//*************************************************************************************************************
//*************************************************************************************************************

void Pod::rotate(const Coords target) {
	float angleToTurn = calcDircetionToTurn(target);
	rotate(angleToTurn);
}

//*************************************************************************************************************
//*************************************************************************************************************

void Pod::rotate(float rotAngle) {
	// Can't turn by more than 18 in one turn
	if (rotAngle > MAX_ANGLE) {
		rotAngle = MAX_ANGLE;
	}
	else if (rotAngle < MIN_ANGLE) {
		rotAngle = MIN_ANGLE;
	}

	angle += rotAngle;

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
	rotate(action.getAngle());

	if (action.hasFlag(SHIELD_FLAG)) {
		activateShield();
	}

	int thrustToApply = action.getThrust();
	if (action.hasFlag(SHIELD_FLAG)) {
		thrustToApply = 0;
	}

	applyThrust(thrustToApply);
}

//*************************************************************************************************************
//*************************************************************************************************************

void Pod::nextCPReached(const int chekpointsCountOnTrack) {
	turnsLeft = INITIAL_NEXT_CHECKPOINT_TURNS_LEFT;
	++passedCheckpoints;
	nextCheckopoint = passedCheckpoints % chekpointsCountOnTrack;
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

void Pod::turnEnd() {
	position.x = round(position.x);
	position.y = round(position.y);
	velocity.x = truncate(velocity.x * FRICTION);
	velocity.y = truncate(velocity.y * FRICTION);

	// Don't forget that the timeout goes down by 1 each turn. It is reset to 100 when you pass a checkpoint
	--turnsLeft;

	manageSheild();
}

//*************************************************************************************************************
//*************************************************************************************************************

bool Pod::winner(const int chekpointsCountOnTrack) const {
	// 0th checkpoint must be reached again
	return passedCheckpoints >= (RACE_LAPS * chekpointsCountOnTrack) + 1;
}

//*************************************************************************************************************
//*************************************************************************************************************

bool Pod::eliminated() const {
	return turnsLeft <= 0;
}

//*************************************************************************************************************
//*************************************************************************************************************

float Pod::score(const Track& track) const {
	const float distanceToNextCP = position.distance(track.getCheckpoint(nextCheckopoint));
	return (PASSED_CPS_WEIGHT * passedCheckpoints) - distanceToNextCP;
}

//*************************************************************************************************************
//*************************************************************************************************************

void Pod::setFlag(const unsigned int flag, const bool initialTurn) {
	flags |= flag;

	if (initialTurn) {
		initialTurnFlags |= flag;
	}
}

//*************************************************************************************************************
//*************************************************************************************************************

void Pod::unsetFlag(const unsigned int flag, const bool initialTurn) {
	flags &= ~flag;

	if (initialTurn) {
		initialTurnFlags &= ~flag;
	}
}

//*************************************************************************************************************
//*************************************************************************************************************

bool Pod::hasFlag(const unsigned int flag, const bool initialTurn) const {
	bool has = flag & flags;

	if (initialTurn) {
		has = flag & initialTurnFlags;
	}

	return has;
}

//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------

class Chromosome {
public:
	Chromosome();

	void setEvaluation(const float evaluation) { this->evaluation = evaluation; }
	void setOriginalEvaluation(const float originalEvaluation) { this->originalEvaluation = originalEvaluation; }
	void setGene(const int geneIdx, const float geneValue) { genes[geneIdx] = geneValue; }
	void setFlags(const unsigned int flags) { this->flags = flags; }
	void setFlag(const unsigned int flag) { flags |= flag; }
	void unsetFlag(const unsigned int flag) { flags &= ~flag; }

	float getEvaluation() const { return evaluation; }
	float getOriginalEvaluation() const { return originalEvaluation; }
	float getGene(const int geneIdx)const { return genes[geneIdx]; }
	unsigned int getFlags() const { return flags; }
	bool hasFlag(const unsigned int flag) const { return flag & flags; }

	/// Initalize the chromosome with random genes
	void initRandom();

	/// Mutate the Chromosome using CGA technique
	void mutate();

	/// Reset for simulation
	void reset();

	/// Copy the given chromosome into this
	/// @param[in] source the source chromosome
	void copy(const Chromosome& source);

private:
	/// Chromosome's genes
	/// First half of the genes represent actions for the pod[0] of the team
	/// Second half of the genes represent actions for the pod[1] of the team
	float genes[CHROMOSOME_SIZE];

	float evaluation; ///< The evaluation value for this genes, set by race simulator, will be normalized
	float originalEvaluation; ///< Not normalized evaluation
	unsigned int flags; /// Stored chromosome properties
};

//*************************************************************************************************************
//*************************************************************************************************************

Chromosome::Chromosome() :
	evaluation{},
	originalEvaluation{},
	flags{}
{}

//*************************************************************************************************************
//*************************************************************************************************************

void Chromosome::initRandom() {
	reset();

	for (int geneIdx = 0; geneIdx < CHROMOSOME_SIZE; ++geneIdx) {
		genes[geneIdx] = randomFloatBetween0and1();
	}
}

//*************************************************************************************************************
//*************************************************************************************************************

void Chromosome::mutate() {
	for (int geneIdx = 0; geneIdx < CHROMOSOME_SIZE; ++geneIdx) {
		const float r = randomFloatBetween0and1();

		if (r < PROBABILITY_OF_MUTATION) {
			// Not sure if this is the best mutation
			// Use the logic from the initRandomPopulation
			// May the restrictions from the previous turn could be taken in account
			genes[geneIdx] = randomFloatBetween0and1();
		}
	}
}

//*************************************************************************************************************
//*************************************************************************************************************

void Chromosome::reset() {
	flags = 0;
}

//*************************************************************************************************************
//*************************************************************************************************************

void Chromosome::copy(const Chromosome& source) {
	for (int geneIdx = 0; geneIdx < CHROMOSOME_SIZE; ++geneIdx) {
		genes[geneIdx] = source.genes[geneIdx];
	}
}

//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------

/// Represents the whole race, holds information for the track, the pods, simulate pods and performs minimax
class RaceSimulator {
public:
	const Pod& getPod(const int podIdx) const { return pods[podIdx]; };
	const Track getTrack() const { return track; }
	
	/// Return the pod from the team, with the flag
	/// @param[in] team the team to check
	/// @param[in] flag the flag to check
	/// @return the wanted pod
	const Pod& getPod(const Team team, const unsigned int flag) const;

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
	/// @param[in] actionsToSimulate actions to test
	/// @param[in] enemyActions conditional other player actions
	/// @param[in] team the team which is being simulated
	void simulate(const Chromosome& actionsToSimulate, Chromosome* enemyActions, const Team team);

	/// Simulate pods for the given actions
	/// @param[in] podsActiona commands for all pods
	/// @param[in] team the team which is being simulated
	void simulatePods(Action(&podsActions)[PODS_COUNT], const Team team);

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

	/// Set the roles of each pod in each team
	void setPodsRoles();

	/// Update pods for the end of the turn
	void turnEnd();

	/// Consider the turn action for the given pod
	/// @param[in] turnAction the action which the Pod makes
	/// @param[in] podIdx the pod index which makes the turn
	void manageTurnAction(const Action turnAction, const int podIdx);

	/// Evaluate the current state of the pods
	/// @param[in] team the team to consider first person when evaluating
	/// @return the evaluation for the pods
	float evaluate(const Team team);

	/// Perform the first turn. Rotate pods as much as needed to the checkpoint.
	/// Use maximum acceleration
	void makeFirstTurn();

#ifdef SVG
	vector<pair<float, float>> podsPaths[PODS_COUNT]; ///< Path for each pod
#endif // SVG

private:
	/// Check if the given team have won
	/// @param[in] team which team to check
	/// @return true if the given team have won the race
	bool teamWon(const Team team);

	/// Check if the given team have won
	/// @param[in] team which team to check
	/// @return true if the given team have won the race
	bool teamLost(const Team team);

	Pod pods[PODS_COUNT]; ///< Pods participating in the race
	Track track; ///< The track on which the race is performed
};

//*************************************************************************************************************
//*************************************************************************************************************

const Pod& RaceSimulator::getPod(const Team team, const unsigned int flag) const {
	const Pod* pod = &pods[0]; //!!!

	const int firstPodIdx = (Team::MY == team) ? 0 : TEAM_PODS_COUNT;
	const int lastPodIdx = (Team::MY == team) ? (TEAM_PODS_COUNT - 1) : PODS_COUNT - 1;

	for (int podIdx = firstPodIdx; podIdx <= lastPodIdx; ++podIdx) {
		pod = &pods[podIdx];
		if (pod && pod->hasFlag(flag)) {
			break;
		}
	}

	return *pod;
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

void RaceSimulator::simulate(const Chromosome& actionsToSimulate, Chromosome* enemyActions, const Team team) {
	for (int podActionIdx = 0; podActionIdx < PODS_COUNT; ++podActionIdx) {
		pods[podActionIdx].reset();
#ifdef SVG
		podsPaths[podActionIdx].push_back(pods[podActionIdx].getPosition().toPair());
#endif // SVG
	}
	
	for (int geneIdx = 0; geneIdx < CHROMOSOME_HALF_SIZE; geneIdx += TRIPLET) {
		Action podsActions[PODS_COUNT];

		const float g00 = actionsToSimulate.getGene(geneIdx);
		const float g01 = actionsToSimulate.getGene(geneIdx + 1);
		const float g02 = actionsToSimulate.getGene(geneIdx + 2);

		const float g10 = actionsToSimulate.getGene(CHROMOSOME_HALF_SIZE + geneIdx);
		const float g11 = actionsToSimulate.getGene(CHROMOSOME_HALF_SIZE + geneIdx + 1);
		const float g12 = actionsToSimulate.getGene(CHROMOSOME_HALF_SIZE + geneIdx + 2);

		if (enemyActions) {
			podsActions[0].parseGenes(g00, g01, g02);
			podsActions[1].parseGenes(g10, g11, g12);

			const float e00 = enemyActions->getGene(geneIdx);
			const float e01 = enemyActions->getGene(geneIdx + 1);
			const float e02 = enemyActions->getGene(geneIdx + 2);

			const float e10 = enemyActions->getGene(CHROMOSOME_HALF_SIZE + geneIdx);
			const float e11 = enemyActions->getGene(CHROMOSOME_HALF_SIZE + geneIdx + 1);
			const float e12 = enemyActions->getGene(CHROMOSOME_HALF_SIZE + geneIdx + 2);

			podsActions[2].parseGenes(e00, e01, e02);
			podsActions[3].parseGenes(e10, e11, e12);
		}
		else {
			// When simulating the enemy, consider my pods still
			podsActions[0].setThrust(0);
			podsActions[1].setThrust(0);
			podsActions[2].parseGenes(g00, g01, g02);
			podsActions[3].parseGenes(g10, g11, g12);
		}

		simulatePods(podsActions, team);
	}
}

//*************************************************************************************************************
//*************************************************************************************************************

void RaceSimulator::simulatePods(Action (&podsActions)[PODS_COUNT], const Team team) {
	for (int podActionIdx = 0; podActionIdx < PODS_COUNT; ++podActionIdx) {
		pods[podActionIdx].applyAction(podsActions[podActionIdx]);
	}

	// Move all pods simulataniously after their actions are applied
	movePods();

	for (int podActionIdx = 0; podActionIdx < PODS_COUNT; ++podActionIdx) {
		pods[podActionIdx].turnEnd();

#ifdef SVG
		// My pods are still when simulating the enemy
		if (Team::ENEMY == team && podActionIdx < TEAM_PODS_COUNT) {
			continue;
		}

		podsPaths[podActionIdx].push_back(pods[podActionIdx].getPosition().toPair());
#endif // SVG
	}
}

//*************************************************************************************************************
//*************************************************************************************************************

void RaceSimulator::movePods() {
	// This tracks the time during the turn. The goal is to reach 1.0
	float t = TURN_START_TIME;

	Collision previousCollision = INVALID_COLLISION;
	while (t < TURN_END_TIME) {
		Collision firstCollision = INVALID_COLLISION;

		// We look for all the collisions that are going to occur during the turn
		for (int i = 0; i < PODS_COUNT; ++i) {
			// Collision with another pod?
			for (int j = i + 1; j < PODS_COUNT; ++j) {
				Collision col = checkForCollision(i, j, CollisionType::WITH_POD);

				// If the collision occurs earlier than the one we currently have we keep it
				if (col.getCollisionTurnTime() > 0.f &&
					col.isValid() &&
					!compareCollisions(previousCollision, col) &&
					col.getCollisionTurnTime() + t < TURN_END_TIME &&
					(!firstCollision.isValid() || col.getCollisionTurnTime() < firstCollision.getCollisionTurnTime())
				) {
					firstCollision = col;
				}
			}

			// Collision with another checkpoint?
			// It is unnecessary to check all checkpoints here. We only test the pod's next checkpoint.
			// We could look for the collisions of the pod with all the checkpoints, but if such a collision happens it wouldn't impact the game in any way
			Collision col = checkForCollision(i, pods[i].getNextCheckopoint(), CollisionType::WITH_CHECKPOINT);

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
			previousCollision = firstCollision;
		}
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
		//entityBRadius = CHECKPOINT_RADIUS;
		entityBRadius = CHECKPOINT_RADIUS - POD_RADIUS; // Center of the Pod must be in the CP radius
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

//*************************************************************************************************************
//*************************************************************************************************************

void RaceSimulator::setPodsRoles() {
	// Assuming each team has 2 Pods
	Pod& myPod0 = pods[0];
	Pod& myPod1 = pods[1];
	myPod0.unsetFlag(RUNNER_FLAG | HUNTER_FLAG, true);
	myPod1.unsetFlag(RUNNER_FLAG | HUNTER_FLAG, true);

	if (myPod0.score(track) > myPod1.score(track)) {
		myPod0.setFlag(RUNNER_FLAG, true);
		myPod1.setFlag(HUNTER_FLAG, true);
	}
	else {
		myPod0.setFlag(HUNTER_FLAG, true);
		myPod1.setFlag(RUNNER_FLAG, true);
	}

	Pod& enemyPod0 = pods[2];
	Pod& enemyPod1 = pods[3];
	enemyPod0.unsetFlag(RUNNER_FLAG | HUNTER_FLAG, true);
	enemyPod1.unsetFlag(RUNNER_FLAG | HUNTER_FLAG, true);

	if (enemyPod0.score(track) > enemyPod1.score(track)) {
		enemyPod0.setFlag(RUNNER_FLAG, true);
		enemyPod1.setFlag(HUNTER_FLAG, true);
	}
	else {
		enemyPod0.setFlag(HUNTER_FLAG, true);
		enemyPod1.setFlag(RUNNER_FLAG, true);
	}
}

//*************************************************************************************************************
//*************************************************************************************************************

void RaceSimulator::turnEnd() {
	for (int podIdx = 0; podIdx < PODS_COUNT; ++podIdx) {
		pods[podIdx].reset(); // After the last simulation
		pods[podIdx].turnEnd();
	}
}

//*************************************************************************************************************
//*************************************************************************************************************

void RaceSimulator::manageTurnAction(const Action turnAction, const int podIdx) {
	if (turnAction.hasFlag(SHIELD_FLAG)) {
		pods[podIdx].activateShield();
		pods[podIdx].setFlag(SHIELD_FLAG, true);
		pods[podIdx].setInitalShieldTurns(SHEILD_TURNS);
	}
}

//*************************************************************************************************************
//*************************************************************************************************************

float RaceSimulator::evaluate(const Team team) {
	float evaluation = 0.f;

	Team firstPersonTeam = Team::MY;
	Team opponentTeam = Team::ENEMY;
	if (Team::ENEMY == team) {
		firstPersonTeam = Team::ENEMY;
		opponentTeam = Team::MY;
	}

	// When simulating enemy my pods are still
	//if ((Team::ENEMY != team && teamWon(opponentTeam)) || teamLost(firstPersonTeam)) {
	//	evaluation = MINUS_INFINITY;
	//	cerr << "MINUS_INF" << endl;
	//}
	//else if ((Team::ENEMY != team && teamLost(opponentTeam)) || teamWon(firstPersonTeam)) {
	//	evaluation = PLUS_INFINITY;
	//	cerr << "PLUS_INF" << endl;
	//}
	//else {
		const Pod& myRunner = getPod(firstPersonTeam, RUNNER_FLAG);
		const Pod& myHunter = getPod(firstPersonTeam, HUNTER_FLAG);
		const Pod& enemyRunner = getPod(opponentTeam, RUNNER_FLAG);
		const Pod& enemyHUnter = getPod(opponentTeam, HUNTER_FLAG);

		if (Team::MY == team) {
			evaluation += SCORE_DIFF_WEIGHT * (myRunner.score(track) - enemyRunner.score(track));
			evaluation -= HUNTER_DISTANCE_WEIGHT * myHunter.getPosition().distance(track.getCheckpoint(enemyRunner.getNextCheckopoint()));
			evaluation -= HUNTER_ANGLE_WEIGHT * myHunter.calcAngleToTarget(enemyRunner.getPosition());
		}
		else {
			evaluation += myRunner.score(track);
			evaluation += myHunter.score(track);
		}
	//}

	return evaluation;
}

//*************************************************************************************************************
//*************************************************************************************************************

void RaceSimulator::makeFirstTurn() {
	Coords firstCheckPoint = track.getCheckpoint(1);
	const int cpX = static_cast<int>(firstCheckPoint.x);
	const int cpY = static_cast<int>(firstCheckPoint.y);

	cout << cpX << SPACE << cpY << SPACE << MAX_THRUST << endl;
	cout << cpX << SPACE << cpY << SPACE << BOOST << endl;
}

//*************************************************************************************************************
//*************************************************************************************************************

bool RaceSimulator::teamWon(const Team team) {
	bool won = false;
	const int firstPodIdx = (Team::MY == team) ? 0 : TEAM_PODS_COUNT;
	const int lastPodIdx = (Team::MY == team) ? (TEAM_PODS_COUNT - 1) : PODS_COUNT - 1;

	for (int podIdx = firstPodIdx; podIdx <= lastPodIdx; ++podIdx) {
		if (pods[podIdx].winner(track.getCheckpointsCount())) {
			won = true;
			break;
		}
	}

	return won;
}

//*************************************************************************************************************
//*************************************************************************************************************

bool RaceSimulator::teamLost(const Team team) {
	const int firstPodIdx = (Team::MY == team) ? 0 : TEAM_PODS_COUNT;
	const int lastPodIdx = (Team::MY == team) ? (TEAM_PODS_COUNT - 1) : PODS_COUNT - 1;

	return pods[firstPodIdx].eliminated() && pods[lastPodIdx].eliminated();
}

//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------

/// Performs genetic algorithm first on the enemy pods (while my pods are not moving) to predict enemy moves
/// Then run GA on my pods based on the nemy best moves
class GA {
public:
	GA(RaceSimulator& raceSimulator);
	~GA();

	Action getTurnAction(const int actionIdx) const { return turnActions[actionIdx]; }

	/// Perform the genetic algorithm
	/// @param[in] turnIdx the turn for which the simulation is run
	void run(const int turnIdx);

private:
	/// Run GA for the given team
	/// Enemy: Optimise actions for the enemy team, assuming my pods are still
	/// My: Optimise actions for my team, based on the best actions chosen for the enemy
	/// @param[in] turnIdx the turn for which the simulation is run
	void runForTeam(const Team team, const int turnIdx);

	/// Init random population
	void init();

	/// Simulate the GA data for the given team
	/// @param[in] team the team which will be simulated
	void simulate(const Team team);

	/// Evaluate all chromosomes
	/// @param[in] team the team which will be simulated
	void evaluate(const Team team);

	/// Prepare the population for the roullete wheel selection:
	///		- calc the sum of evaluations
	///		- normalize the evalutions
	///		- sort the population's chromosome based on the cumulative sum
	///		- calc the cumulative sum
	void prepareForRoulleteWheel();

	/// Select parents' indecies, for crossover, using the roullete wheel technique
	/// @param[out] parent0Idx the first parent's index, which will be used for the crossover
	/// @param[out] parent1Idx the second parent's index, which will be used for the crossover
	void selectParentsIdxs(int& parent0Idx, int& parent1Idx);

	/// Crossover pair of chromosomes to make new pair and add them to the new children
	/// @param[in] parent0Idx the first parent's index, which will be used for the crossover
	/// @param[in] parent1Idx the second parent's index, which will be used for the crossover
	/// @param[in] childrenCount how many children are already created
	void crossover(int parent0Idx, int parent1Idx, int childrenCount);

	/// Mutate the last two chromosomes in the children array, they are the new from the crossover
	/// @param[in] childrenCount how many children are already created
	void mutate(int childrenCount);

	/// Get the best chromosomes from the current population and pass them unchanged to the next
	void elitism();

	/// Use the Continuos Genetic Algorithm methods to make the children for the new generation
	void makeChildren();

	/// Reset stats for each induvidual to the default values
	void resetPopulation();

	/// Reset the whole algortihm
	void reset();

	/// Plain copy chromosome: TODO: may be optimized
	/// @param[in] destIdx the chromosome in the new population
	/// @param[in] sourceIdx the chromosome in the old population
	void copyChromosomeToNewPopulation(int destIdx, int sourceIdx);

	/// Switch form ENEMY simulation to MY team simulation, using the best result from ENEMY simulation
	/// @param[in] simulationTeam the team which will be simulated
	void switchTeamsForSimulation(const Team simulationTeam);

	/// Get the first genes from thethe best chromosome from the last simulation
	void chooseTurnActions();

	/// Will change the content in A when B is active and vise versa
	Chromosome populationA[POPULATION_SIZE];
	Chromosome populationB[POPULATION_SIZE];

	/// Holds chromosomes' evaluations as keys and chromosomes' indecies as values
	/// using map to hold this information, because every time a key is inserted it is stored in place (sorted)
	/// the map is represented as tree and every time an element is inserted I think is faster and sorting whole array of chromosomes
	/// TODO: measure the speed and if needed implement own hash map, no easy way to reserve map elements in advance,
	/// but I think map of floats and ints shouldn't be the bottle neck of the program
	ChromEvalIdxMap chromEvalIdxPairs;

	Chromosome* population; ///< Points to active population
	Chromosome* newPopulation; ///< Points to newly created population

	float evaluationsSum; ///< Sum of all chromosome evaluations, needed for faster roullete wheel
	float minEvaluation; ///< The minumum evaluation for the current iteration of the algorithm
	int populationIdx; ///< The index of the current population

	/// Game specific members
	Chromosome enemyActions; ///< Best actions for the enemy
	Action turnActions[TEAM_PODS_COUNT]; ///< Turn action for pods
	RaceSimulator& raceSimulator; ///< Pods controller
	int populationSize; ///< How many population simulations to perform

#ifdef SVG
	SVGManager svgManager; ///< The svg manager for debug
#endif // SVG
};

//*************************************************************************************************************
//*************************************************************************************************************

GA::GA(RaceSimulator& raceSimulator) :
	evaluationsSum{ 0.f },
	minEvaluation{ 0.f },
	populationIdx{ 0 },
	raceSimulator{ raceSimulator },
	populationSize{ ENEMY_MAX_POPULATION }
{
	reset();
}

//*************************************************************************************************************
//*************************************************************************************************************

GA::~GA() {
#ifdef SVG
	svgManager.fileDone();
#endif // SVG
}

//*************************************************************************************************************
//*************************************************************************************************************

void GA::run(const int turnIdx) {
	runForTeam(Team::ENEMY, turnIdx);
	runForTeam(Team::MY, turnIdx);
	chooseTurnActions();
}

//*************************************************************************************************************
//*************************************************************************************************************

void GA::runForTeam(const Team team, const int turnIdx) {
#ifdef SVG
	svgManager.fileInit(turnIdx, Team::ENEMY == team);

	const Track& track = raceSimulator.getTrack();
	for (int cpIdx = 0; cpIdx < track.getCheckpointsCount(); ++cpIdx) {
		Coords cpCoords = track.getCheckpoint(cpIdx);
		string circleStr = CIRCLE_BEGIN;
		circleStr += to_string(cpCoords.x);
		circleStr += CIRCLE_MIDDLE;
		circleStr += to_string(cpCoords.y);
		circleStr += CIRCLE_END;
		svgManager.filePrintStr(circleStr);

		string cpIdStr = TEXT_BEGIN;
		cpIdStr += to_string(cpCoords.x - 80.f);
		cpIdStr += TEXT_MIDDLE;
		cpIdStr += to_string(cpCoords.y + 80.f);
		cpIdStr += TEXT_STYLE;
		cpIdStr += to_string(cpIdx);
		cpIdStr += TEXT_END;
		svgManager.filePrintStr(cpIdStr);
	}
#endif // SVG

	switchTeamsForSimulation(team);
	init();

	// populationSize must be adjusted for the given time
	// There is no single guaranteed solution, evolve while you can
	while (populationIdx < populationSize) {
		simulate(team);
		prepareForRoulleteWheel();
		makeChildren();
		elitism();

		resetPopulation();
		++populationIdx;
	}

#ifdef SVG
	svgManager.fileDone();
	svgManager.fileClose();
#endif // SVG
}

//*************************************************************************************************************
//*************************************************************************************************************

void GA::init() {
	for (int chromIdx = 0; chromIdx < POPULATION_SIZE; ++chromIdx) {
		population[chromIdx].initRandom();
	}
}

//*************************************************************************************************************
//*************************************************************************************************************

void GA::simulate(const Team team) {
#ifdef SVG
	svgManager.filePrintStr(svgManager.constructGId(populationIdx));
#endif // SVG

	for (int chromIdx = 0; chromIdx < POPULATION_SIZE; ++chromIdx) {
		Chromosome& chromosome = population[chromIdx];
		if (chromosome.hasFlag(COPIED_FLAG)) {
			const float originalEvaluation = chromosome.getOriginalEvaluation();
			minEvaluation = min(minEvaluation, originalEvaluation);
			evaluationsSum += originalEvaluation;
			chromosome.setEvaluation(originalEvaluation);
			chromosome.unsetFlag(COPIED_FLAG);
			continue;
		}

		if (Team::MY == team) {
			raceSimulator.simulate(chromosome, &enemyActions, team);
		}
		else {
			raceSimulator.simulate(chromosome, nullptr, team);
		}

		const float chromEvaluation = raceSimulator.evaluate(team);
		minEvaluation = min(minEvaluation, chromEvaluation);
		evaluationsSum += chromEvaluation;
		chromosome.setEvaluation(chromEvaluation);
		chromosome.setOriginalEvaluation(chromEvaluation);

#ifdef SVG
		svgManager.constructPaths(raceSimulator.podsPaths, Team::ENEMY == team);
#endif // SVG
	}

#ifdef SVG
	svgManager.filePrintStr(CLOSE_GROUP);
#endif // SVG
}

//*************************************************************************************************************
//*************************************************************************************************************

void GA::prepareForRoulleteWheel() {
	//cerr << "minEvaluation: " << minEvaluation << endl;
	const float evaluationScaling = abs(minEvaluation);

	for (int chromIdx = 0; chromIdx < POPULATION_SIZE; ++chromIdx) {
		Chromosome& chromosome = population[chromIdx];
		const float scaledEvalaution = chromosome.getEvaluation() + evaluationScaling;
		const float scaledEvalautionSum = evaluationsSum + (POPULATION_SIZE * evaluationScaling);
		const float normalizedEvaluation = scaledEvalaution / scaledEvalautionSum; // normalize the evalutions

		chromosome.setEvaluation(normalizedEvaluation);

		chromEvalIdxPairs.insert(pair<float, int>(normalizedEvaluation, chromIdx)); // Is it good think to use floats as keys
	}

#ifdef REDIRECT_INPUT
	float sum = 0.f;
	for (const pair<float, int>& p : chromEvalIdxPairs) {
		sum += p.first;
	}

	int debug = 0;
	++debug;
#endif // REDIRECT_INPUT
}

//*************************************************************************************************************
//*************************************************************************************************************

void GA::selectParentsIdxs(int& parent0Idx, int& parent1Idx) {
	float r = randomFloatBetween0and1();

	float cumulativeSum = 0.f;
	parent0Idx = chromEvalIdxPairs.rbegin()->second; // If r is too big the loop will break before setting the parentIdx
	for (ChromEvalIdxMap::const_iterator it = chromEvalIdxPairs.begin(); it != chromEvalIdxPairs.end(); ++it) {
		cumulativeSum += it->first;
		parent0Idx = it->second;

		if (r < cumulativeSum) {
			break;
		}
	}

	parent1Idx = parent0Idx;
	while (parent1Idx == parent0Idx) {
		r = randomFloatBetween0and1();

		cumulativeSum = 0.f;
		parent1Idx = chromEvalIdxPairs.rbegin()->second; // If r is too big the loop will break before setting the parentIdx
		// Code duplication, at the moment I cannot think of more elegant layout
		for (ChromEvalIdxMap::const_iterator it = chromEvalIdxPairs.begin(); it != chromEvalIdxPairs.end(); ++it) {
			cumulativeSum += it->first;
			parent1Idx = it->second;

			if (r < cumulativeSum) {
				break;
			}
		}
	}
}

//*************************************************************************************************************
//*************************************************************************************************************

void GA::crossover(int parent0Idx, int parent1Idx, int childrenCount) {
	const Chromosome& parent0 = population[parent0Idx];
	const Chromosome& parent1 = population[parent1Idx];

	for (int geneIdx = 0; geneIdx < CHROMOSOME_SIZE; ++geneIdx) {
		float parent0Gene = parent0.getGene(geneIdx);
		float parent1Gene = parent1.getGene(geneIdx);

		const float beta = randomFloatBetween0and1();

		const float child0Gene = (beta * parent0Gene) + ((1.f - beta) * parent1Gene);
		const float child1Gene = ((1.f - beta) * parent0Gene) + (beta * parent1Gene);

		newPopulation[childrenCount].setGene(geneIdx, child0Gene);
		newPopulation[childrenCount + 1].setGene(geneIdx, child1Gene);
	}
}

//*************************************************************************************************************
//*************************************************************************************************************

void GA::mutate(int childrenCount) {
	// Last two chromosomes are made during the crossover
	newPopulation[childrenCount].mutate();
	newPopulation[childrenCount + 1].mutate();
}

//*************************************************************************************************************
//*************************************************************************************************************

void GA::elitism() {
	const int elitsCount = static_cast<int>(POPULATION_SIZE * ELITISM_RATIO);

	// Use the map, which holds sorted evaluations as keys
	ChromEvalIdxMap::const_reverse_iterator it = chromEvalIdxPairs.rbegin();
	for (int elitIdx = 0; elitIdx < elitsCount; ++elitIdx) {
		const int chromosomeIdx = it->second;

		copyChromosomeToNewPopulation(elitIdx, chromosomeIdx);
		++it;
	}
}

//*************************************************************************************************************
//*************************************************************************************************************

void GA::makeChildren() {
	// While the new population is not completly filled
	// select a pair of parents
	// crossover those parents using the Continuos Genetic Algorithm technique
	// mutate them using the Continuos Genetic Algorithm technique
	for (int childrenCount = 0; childrenCount < POPULATION_SIZE; childrenCount += 2) {
		int parent0Idx = INVALID_ID; // For safety reasons
		int parent1Idx = INVALID_ID; // For safety reasons

		selectParentsIdxs(parent0Idx, parent1Idx);

		crossover(parent0Idx, parent1Idx, childrenCount);
		mutate(childrenCount);
	}
}

//*************************************************************************************************************
//*************************************************************************************************************

void GA::resetPopulation() {
	evaluationsSum = 0.f;
	minEvaluation = 0.f;
	chromEvalIdxPairs.clear();

	// If copied directly from previous population do not reset
	for (int chromIdx = 0; chromIdx < POPULATION_SIZE; ++chromIdx) {
		Chromosome& chromosome = newPopulation[chromIdx];
		if (!chromosome.hasFlag(COPIED_FLAG)) {
			chromosome.reset();
		}
	}

	// Switch population arrays
	if (0 == (populationIdx % 2)) {
		population = populationB;
		newPopulation = populationA;
	}
	else {
		population = populationA;
		newPopulation = populationB;
	}
}

//*************************************************************************************************************
//*************************************************************************************************************

void GA::reset() {
	evaluationsSum = 0.f;
	minEvaluation = 0.f;
	populationIdx = 0;
	chromEvalIdxPairs.clear();

	population = populationA;
	newPopulation = populationB;
}

//*************************************************************************************************************
//*************************************************************************************************************

void GA::copyChromosomeToNewPopulation(int destIdx, int sourceIdx) {
	const Chromosome& sourceChromosome = population[sourceIdx];
	Chromosome& destinationChromosome = newPopulation[destIdx];

	destinationChromosome.setEvaluation(sourceChromosome.getEvaluation());
	destinationChromosome.setOriginalEvaluation(sourceChromosome.getOriginalEvaluation());
	destinationChromosome.setFlags(sourceChromosome.getFlags());
	for (int geneIdx = 0; geneIdx < CHROMOSOME_SIZE; ++geneIdx) {
		destinationChromosome.setGene(geneIdx, sourceChromosome.getGene(geneIdx));
	}

	// Set at the end of copying to not be overwritten
	destinationChromosome.setFlag(COPIED_FLAG);
}

//*************************************************************************************************************
//*************************************************************************************************************

void GA::switchTeamsForSimulation(const Team simulationTeam) {
	if (Team::MY == simulationTeam) {
		enemyActions.copy(population[0]); // Last elitism stored the best chromosome in 0th position
		populationSize = MY_MAX_POPULATION;

#ifdef SVG
		raceSimulator.simulate(population[0], nullptr, Team::ENEMY);
		svgManager.constructPaths(raceSimulator.podsPaths, true);
#endif // SVG
	}
	else {
		populationSize = ENEMY_MAX_POPULATION;
	}

	reset();
}

//*************************************************************************************************************
//*************************************************************************************************************

void GA::chooseTurnActions() {
	const Chromosome& bestChromosome = population[0]; // Last elitism stored the best chromosome in 0th position

	turnActions[0].parseGenes(bestChromosome.getGene(0), bestChromosome.getGene(1), bestChromosome.getGene(2));
	turnActions[1].parseGenes(
		bestChromosome.getGene(CHROMOSOME_HALF_SIZE + 0),
		bestChromosome.getGene(CHROMOSOME_HALF_SIZE + 1),
		bestChromosome.getGene(CHROMOSOME_HALF_SIZE + 2)
	);
}

//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------

class Game {
public:
	Game();

	void gameLoop();
	void getGameInput();
	void getTurnInput();
	void turnBegin();
	void makeTurn();
	void turnEnd();
	void play();

private:
	// Game specific members
	RaceSimulator raceSimulator; ///< Whole race manager
	GA ga; ///< The genetic algorithm manager

	int turnsCount;
	int stopGame;
};

//*************************************************************************************************************
//*************************************************************************************************************

Game::Game() :
	ga{ raceSimulator },
	turnsCount{ 0 },
	stopGame{ false }
{

}

//*************************************************************************************************************
//*************************************************************************************************************

void Game::gameLoop() {
	while (!stopGame) {
#ifdef TIME_MEASURERMENT
		clock_t turnBeginTime = clock();
#endif // TIME_MEASURERMENT

		getTurnInput();
		turnBegin();
		makeTurn();
		turnEnd();

#ifdef TIME_MEASURERMENT
		clock_t tunrEndTime = clock();
		double elapsed_secs = double(tunrEndTime - turnBeginTime) / CLOCKS_PER_SEC;
		cerr << "Turn[" << turnsCount - 1 << "] execution time: " << elapsed_secs * 1000 << endl;
#endif // TIME_MEASURERMENT

#ifdef DEBUG_ONE_TURN
		if (2 == turnsCount) {
			break;
		}
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

#ifdef REDIRECT_INPUT
		if (INVALID_ID == x) {
			stopGame = true;
		}
#endif // REDIRECT_INPUT

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
	raceSimulator.setPodsRoles();
	ga.run(turnsCount);
}

//*************************************************************************************************************
//*************************************************************************************************************

void Game::makeTurn() {
	if (0 == turnsCount) {
		raceSimulator.makeFirstTurn();
	}
	else {
		const Action pod0Action = ga.getTurnAction(0);
		raceSimulator.manageTurnAction(pod0Action, 0);
		pod0Action.output(raceSimulator.getPod(0).getInitialTurnPosition(), raceSimulator.getPod(0).getInitialTurnAngle());
	
		const Action pod1Action = ga.getTurnAction(1);
		raceSimulator.manageTurnAction(pod1Action, 1);
		pod1Action.output(raceSimulator.getPod(1).getInitialTurnPosition(), raceSimulator.getPod(1).getInitialTurnAngle());
	}
}

//*************************************************************************************************************
//*************************************************************************************************************

void Game::turnEnd() {
	raceSimulator.turnEnd();
	++turnsCount;
}

//*************************************************************************************************************
//*************************************************************************************************************

void Game::play() {
	getGameInput();
	gameLoop();
}

//-------------------------------------------------------------------------------------------------------------
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

	Game game;
	game.play();

#endif // TESTS

	return 0;
}

// Game simulation parameters
/*
seed = 681000254
pod_per_player = 2
pod_timeout = 100
map = 7982 7873 13284 5513 9539 1380 3637 4405

seed=934785373
pod_per_player=2
pod_timeout=100
map=11506 6100 9112 1855 5027 5287
*/
