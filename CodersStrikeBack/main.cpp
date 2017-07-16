#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <climits>
#include <ctime>
#include <fstream>

#define M_PI 3.14159265358979323846
//#define TESTS

const int USE_HARDCODED_INPUT = 1;
const int USE_INVALID_ROLES = 0;
//const int POD_ACTIONS_COUNT = 7;
const int POD_ACTIONS_COUNT = 3; // For debuging
const int MINIMAX_DEPTH = 8;
const int PRINT_MINIMAX_TREE_TO_FILE = 0;
const int SIM_TURNS = 11;

using namespace std;

const float INVALID_COORD = 0.f;
const float MAX_ANGLE_PER_TURN = 18.f;
const float DEFAULT_ANGLE = -1.f;
const float FRICTION = .85f;
const float HALF_MOMENTUM = 120.f;
const float MASS_WITH_SHEILD = 10.f;
const float MASS_WITHOUT_SHEILD = 1.f;
const float TURN_START_TIME = 0.f;
const float TURN_END_TIME = 1.f;
const float MAX_DIST = 99999.f;
const float MAX_ANGLE = 360.f;
const float MIN_ANGLE = 0.f;
const float TARGET_OFFSET = 5000.f;

const int POD_RADIUS = 400;
const int CHECKPOINT_RADIUS = 600;
const int MAX_THRUST = 100;
const int TEAM_PODS_COUNT = 2;
const int GAME_PODS_COUNT = 4;
const int DEFAULT_POD_TURNS = 100;
const int INVALID_ID = -1;
const int SHEILD_TURNS = 3;
const int FIRST_TURN = 0;
const int SUBSTATE_PODS_COUNT = 2;
const int FIRST_GOAL_CP_ID = 1;
const int POD_DIRECTIONS_COUNT = 5;
const int POD_THRUSTS_COUNT = 3;
const int POD_SHEILD_FLAGS_COUNT = 2;
const int ALL_POSSIBLE_POD_ACTIONS_COUNT = POD_DIRECTIONS_COUNT * POD_THRUSTS_COUNT * POD_SHEILD_FLAGS_COUNT;

const int LAPS_COUNT = 3;

const int PASSED_CPS_WEIGHT = 5000;
const int DIST_TO_NEXT_CP_WEIGHT = 3;
const int ANGLE_TO_NEXT_CP_WEIGHT = 200;
const int DIST_TO_HUNTER_WEIGHT = 5;
const int ANGLE_HUNTER_WEIGHT = 100;

const int ENEMY_DIST_TO_NEXT_CP_WEIGHT = 200;
const int ENEMY_ANGLE_TO_NEXT_CP_WEIGHT = 1500;

const string SHEILD = "SHIELD";
const string BOOST = "BOOST";
const string RUNNER_TREE_FILE = "minimaxRunnerTree.gv";
const string HUNTER_TREE_FILE = "minimaxHunterTree.gv";
const string PARENT_PATH = "P";

const char PARENT_LABEL = 'P';

const float DIRECTION_ANGLES[POD_DIRECTIONS_COUNT] = { -18.f, -9.f, 0.f, 9.f, 18.f };
const int THRUST_VALUES[POD_THRUSTS_COUNT] = { 0, 50, 100 };
const bool SHEILD_FLAGS[POD_SHEILD_FLAGS_COUNT] = { true, false };

enum MaximizeMinimize {
	MM_MAXIMIZE = 0,
	MM_MINIMIZE
};

enum TurnStatePodIdx {
	TSPI_MY_POD_IDX = 0,
	TSPI_ENEMY_POD_IDX = 1,
};

enum PodRole {
	PR_INVALID,
	PR_MY_RUNNER,
	PR_MY_HUNTER,
	PR_ENEMY_RUNNER,
	PR_ENEMY_HUNTER,
};

enum PodDirection {
	PD_INVALID = -1,
	PD_LEFT = 0,
	PD_SLIGHT_LEFT,
	PD_FORWARD,
	PD_SLIGHT_RIGHT,
	PD_RIGHT,
};

enum PodThusts {
	PT_INVALID = -1,
	PT_MIN = 0,
	PT_HALF,
	PT_MAX,
};

enum PodShieldFlags {
	PSF_INALID = -1,
	PSF_ON = 0,
	PSF_OFF,
};

//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------

struct Coords {
	Coords() : xCoord(INVALID_COORD), yCoord(INVALID_COORD) {}
	Coords(float xCoord, float yCoord) : xCoord(xCoord), yCoord(yCoord) {}

	bool operator==(const Coords& coords) const;

	float distanceSquare(Coords p) const;
	float distance(Coords p) const;
	Coords closestPointOnLine(Coords linePointA, Coords linePointB) const;

	float xCoord;
	float yCoord;
};

//*************************************************************************************************************
//*************************************************************************************************************

bool Coords::operator==(const Coords& coords) const {
	return xCoord == coords.xCoord && yCoord == coords.yCoord;
}

//*************************************************************************************************************
//*************************************************************************************************************

float Coords::distanceSquare(Coords point) const {
	return
		(xCoord - point.xCoord) * (xCoord - point.xCoord) +
		(yCoord - point.yCoord) * (yCoord - point.yCoord);
}

//*************************************************************************************************************
//*************************************************************************************************************

float Coords::distance(Coords point) const {
	return sqrt(distanceSquare(point));
}

//*************************************************************************************************************
//*************************************************************************************************************

Coords Coords::closestPointOnLine(Coords linePointA, Coords linePointB) const {
	float da = linePointB.yCoord - linePointA.yCoord;
	float db = linePointA.xCoord - linePointB.xCoord;
	float c1 = da * linePointA.xCoord + db * linePointA.yCoord;
	float c2 = -db * xCoord + da * yCoord;
	float det = da * da + db * db;

	Coords clossestPoint;

	if (det != 0) {
		clossestPoint.xCoord = (da * c1 - db * c2) / det;
		clossestPoint.yCoord = (da * c2 + db * c1) / det;
	}
	else {
		// The point is already on the line
		clossestPoint.xCoord = xCoord;
		clossestPoint.yCoord = yCoord;
	}

	return clossestPoint;
}

//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------

class Action {
public:
	Action();
	Action(Coords target, bool useSheild, int thrust, float directionAngle);
	~Action();

	void setTarget(Coords target) { this->target = target; }
	void setUseSheild(bool useSheild) { this->useSheild = useSheild; }
	void setThrust(int thrust) { this->thrust = thrust; }
	void setDirectionAngle(float directionAngle) { this->directionAngle = directionAngle; }

	Coords getTarget() const { return target; }
	bool getUseSheild() const { return useSheild; }
	int getThrust() const { return thrust; }
	float getDirectionAngle() const { return directionAngle; }

	void fillAction(Coords target, bool useSheild, int thrust);
	bool isValid() const;

	void printAction() const;

	void debug() const;
private:
	Coords target;
	bool useSheild;
	int thrust;
	float directionAngle;
};

//*************************************************************************************************************
//*************************************************************************************************************

Action::Action() : target(), useSheild(false), thrust(0), directionAngle(0.f){

}

//*************************************************************************************************************
//*************************************************************************************************************

Action::Action(Coords target, bool useSheild, int thrust, float directionAngle) :
	target(target),
	useSheild(useSheild),
	thrust(thrust),
	directionAngle(directionAngle)
{
}

//*************************************************************************************************************
//*************************************************************************************************************

Action::~Action() {

}

//*************************************************************************************************************
//*************************************************************************************************************

void Action::fillAction(Coords target, bool useSheild, int thrust) {
	this->target = target;
	this->useSheild = useSheild;
	this->thrust = thrust;
}

//*************************************************************************************************************
//*************************************************************************************************************

bool Action::isValid() const {
	return true;
}

//*************************************************************************************************************
//*************************************************************************************************************

void Action::printAction() const {
	cout << round(target.xCoord) << " " << round(target.yCoord) << " ";

	if (useSheild) {
		cout << SHEILD;
	}
	else {
		cout << thrust;
	}

	cout << endl;
}

//*************************************************************************************************************
//*************************************************************************************************************

void Action::debug() const {
}

//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------

class Entity {
public:
	Entity();
	Entity(Coords position, Coords speedVector, int radius);
	~Entity();

	void setSpeedVector(Coords speedVector) { this->speedVector = speedVector; }
	void setPosition(Coords position) { this->position = position; }

	Coords getPosition() const { return position; }
	Coords getSpeedVector() const { return speedVector; }
	int getRadius() const { return radius; }

	virtual bool sheildOn() const { return false; }
	virtual void computeBounce(Entity* entity) {}

	virtual void debug() const = 0;
protected:
	Coords position;
	Coords speedVector;
	int radius;
};

//*************************************************************************************************************
//*************************************************************************************************************

Entity::Entity() : position(), speedVector(), radius(0) {

}

//*************************************************************************************************************
//*************************************************************************************************************

Entity::Entity(Coords position, Coords speedVector, int radius) :
	position(position),
	speedVector(speedVector),
	radius(radius)
{

}

//*************************************************************************************************************
//*************************************************************************************************************

Entity::~Entity() {

}

//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------

class CheckPoint : public Entity {
public:
	CheckPoint();
	CheckPoint(Coords position, Coords speedVector, int radius, int id);
	~CheckPoint();

	int getId() const { return id; }

	void setId(int id) { this->id = id; }

	void debug() const override;
private:
	int id;
};

//*************************************************************************************************************
//*************************************************************************************************************

CheckPoint::CheckPoint() : Entity() {

}

//*************************************************************************************************************
//*************************************************************************************************************

CheckPoint::CheckPoint(Coords position, Coords speedVector, int radius, int id) :
	Entity(position, speedVector, radius),
	id(id)
{
}

//*************************************************************************************************************
//*************************************************************************************************************

CheckPoint::~CheckPoint() {

}

//*************************************************************************************************************
//*************************************************************************************************************

void CheckPoint::debug() const {
	cerr << "CheckPoint " << id << ":" << endl;
	cerr << "Coords: " << "X = " << position.xCoord << " Y = " << position.yCoord << endl;
	cerr << endl;
}

//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------

class Pod : public Entity {
public:
	Pod();
	Pod(
		Coords position,
		Coords speedVector,
		float angle,
		int nextCheckPointId,
		int turnsLeft,
		bool shieldUp,
		int radius,
		int sheildTurns,
		PodRole role,
		int passedCheckPoints
	);
	~Pod();

	void setAngle(float angle) { this->angle = angle; }
	void setNextCheckPointId(int nextCheckPointId) { this->nextCheckPointId = nextCheckPointId; }
	void setTurnsLeft(int turnsLeft) { this->turnsLeft = turnsLeft; }
	void setRole(PodRole role) { this->role = role; }
	void setPassedCheckPoints(int passedCheckPoints) { this->passedCheckPoints = passedCheckPoints; }

	float getAngle() const { return angle; }
	int getNextCheckPointId() const { return nextCheckPointId; }
	int getTurnsLeft() const { return turnsLeft; }
	int getSheildUp() const { return shieldUp; }
	PodRole getRole() const { return role; }
	int getPassedCheckPoints() const { return passedCheckPoints; }

	float calcAngleToTarget(Coords target) const;
	float calcDircetionToTurn(Coords target) const;
	void rotate(Coords target);
	void applyThrust(int thrust);
	void move(float time);
	void end();
	float truncate(float toTruncate);
	void resetCPCounter();
	void activateSheild();
	void manageSheild();
	float clampAngle(float angleToClamp) const;
	Coords calcPodTarget(float angleToTurn);
	void incrementPassedCPCounter();
	void decreaseTurnsLeft();
	void heuristicSimulate(Action* action);
	int heuristicEval(Coords nextCheckPoint, MaximizeMinimize mm) const;

	void computeBounce(Entity* entity) override;
	bool sheildOn() const override;
	void debug() const override;

private:
	float angle;
	int nextCheckPointId;
	int turnsLeft;
	bool shieldUp;
	int sheildTurnsLeft;
	PodRole role;
	int passedCheckPoints;
};

//*************************************************************************************************************
//*************************************************************************************************************

Pod::Pod() :
	Entity(),
	angle(DEFAULT_ANGLE),
	nextCheckPointId(0),
	turnsLeft(DEFAULT_POD_TURNS),
	shieldUp(false),
	sheildTurnsLeft(0),
	role(PR_INVALID),
	passedCheckPoints(0)
{
}

//*************************************************************************************************************
//*************************************************************************************************************

Pod::Pod(
	Coords position,
	Coords speedVector,
	float angle,
	int nextCheckPointId,
	int turnsLeft,
	bool shieldUp,
	int radius,
	int sheildTurnsLeft,
	PodRole role,
	int passedCheckPoints
) :
	Entity(position, speedVector, radius),
	angle(angle),
	nextCheckPointId(nextCheckPointId),
	turnsLeft(turnsLeft),
	shieldUp(shieldUp),
	sheildTurnsLeft(sheildTurnsLeft),
	role(role),
	passedCheckPoints(passedCheckPoints)
{
}

//*************************************************************************************************************
//*************************************************************************************************************

Pod::~Pod() {
}

//*************************************************************************************************************
//*************************************************************************************************************

float Pod::calcAngleToTarget(Coords target) const {
	float distanceToTarget = position.distance(target);
	float dx = (target.xCoord - position.xCoord) / distanceToTarget;
	float dy = (target.yCoord - position.yCoord) / distanceToTarget;

	// Simple trigonometry. We multiply by 180.f / PI to convert radiants to degrees.
	float angleToTarget = acos(dx) * 180.f / (float)M_PI;

	// If the point I want is below me, I have to shift the angle for it to be correct
	if (dy < 0.f) {
		angleToTarget = 360.f - angleToTarget;
	}

	return angleToTarget;
}

//*************************************************************************************************************
//*************************************************************************************************************

float Pod::calcDircetionToTurn(Coords target) const {
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

void Pod::rotate(Coords target) {
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

void Pod::applyThrust(int thrust) {
	// Don't forget that a pod which has activated its shield cannot accelerate for 3 turns
	if (!shieldUp) {
		// Conversion of the angle to radiants
		float facingAngleDeg = angle * (float)M_PI / 180.f;

		// Trigonometry
		speedVector.xCoord += cos(facingAngleDeg) * thrust;
		speedVector.yCoord += sin(facingAngleDeg) * thrust;
	}
}

//*************************************************************************************************************
//*************************************************************************************************************

void Pod::move(float turnTime) {
	position.xCoord += speedVector.xCoord * turnTime;
	position.yCoord += speedVector.yCoord * turnTime;
}

//*************************************************************************************************************
//*************************************************************************************************************

void Pod::end() {
	position.xCoord = round(position.xCoord);
	position.yCoord = round(position.yCoord);
	speedVector.xCoord = truncate(speedVector.xCoord * FRICTION);
	speedVector.yCoord = truncate(speedVector.yCoord * FRICTION);
	//angle = round(angle);

	// Don't forget that the timeout goes down by 1 each turn. It is reset to 100 when you pass a checkpoint
	--turnsLeft;

	manageSheild();
}

//*************************************************************************************************************
//*************************************************************************************************************

float Pod::truncate(float toTruncate) {
	float res = floor(toTruncate);

	if (toTruncate < 0) {
		res = ceil(toTruncate);
	}

	return res;
}

//*************************************************************************************************************
//*************************************************************************************************************

void Pod::computeBounce(Entity* entity) {
	// If a pod has its shield active its mass is 10 otherwise it's 1
	float m1 = sheildOn() ? MASS_WITH_SHEILD : MASS_WITHOUT_SHEILD;
	float m2 = entity->sheildOn() ? MASS_WITH_SHEILD : MASS_WITHOUT_SHEILD;
	float mcoeff = (m1 + m2) / (m1 * m2);

	float nx = position.xCoord - entity->getPosition().xCoord;
	float ny = position.yCoord - entity->getPosition().yCoord;

	// Square of the distance between the 2 pods. This value could be hardcoded because it is always 800
	float nxnysquare = nx * nx + ny * ny;

	float dvx = speedVector.xCoord - entity->getSpeedVector().xCoord;
	float dvy = speedVector.yCoord - entity->getSpeedVector().yCoord;

	// fx and fy are the components of the impact vector. product is just there for optimisation purposes
	float product = nx * dvx + ny * dvy;
	float fx = (nx * product) / (nxnysquare * mcoeff);
	float fy = (ny * product) / (nxnysquare * mcoeff);

	// We apply the impact vector once
	speedVector.xCoord -= fx / m1;
	speedVector.yCoord -= fy / m1;
	entity->setSpeedVector(
		Coords(
			entity->getSpeedVector().xCoord + fx / m2,
			entity->getSpeedVector().yCoord + fy / m2
		)
	);

	// If the norm of the impact vector is less than 120, we normalize it to 120
	float impulse = sqrt(fx*fx + fy*fy);
	if (impulse < HALF_MOMENTUM) {
		fx = fx * HALF_MOMENTUM / impulse;
		fy = fy * HALF_MOMENTUM / impulse;
	}

	// We apply the impact vector a second time
	speedVector.xCoord -= fx / m1;
	speedVector.yCoord -= fy / m1;
	entity->setSpeedVector(
		Coords(
			entity->getSpeedVector().xCoord + fx / m2,
			entity->getSpeedVector().yCoord + fy / m2
		)
	);

	// This is one of the rare places where a Vector class would have made the code more readable.
	// But this place is called so often that I can't pay a performance price to make it more readable.
}

//*************************************************************************************************************
//*************************************************************************************************************

void Pod::resetCPCounter() {
	turnsLeft = DEFAULT_POD_TURNS;
}

//*************************************************************************************************************
//*************************************************************************************************************

void Pod::activateSheild() {
	shieldUp = true;
	sheildTurnsLeft = SHEILD_TURNS;
}

//*************************************************************************************************************
//*************************************************************************************************************

void Pod::manageSheild() {
	if (shieldUp) {
		if (sheildTurnsLeft > 0) {
			--sheildTurnsLeft;
		}
		else {
			shieldUp = false;
		}
	}
}

//*************************************************************************************************************
//*************************************************************************************************************

Coords Pod::calcPodTarget(float angleToTurn) {
	float podTargetAngle = 0.f;

	podTargetAngle = clampAngle(angle + angleToTurn);

	float angleRadians = podTargetAngle * ((float)M_PI / (MAX_ANGLE / 2));

	float sinLeftOffset = sin(angleRadians);
	float cosLeftOffset = cos(angleRadians);

	sinLeftOffset *= TARGET_OFFSET;
	cosLeftOffset *= TARGET_OFFSET;

	return Coords(cosLeftOffset + position.xCoord, sinLeftOffset + position.yCoord);
}

//*************************************************************************************************************
//*************************************************************************************************************

void Pod::incrementPassedCPCounter() {
	++passedCheckPoints;
}

//*************************************************************************************************************
//*************************************************************************************************************

void Pod::decreaseTurnsLeft() {
	--turnsLeft;
}

//*************************************************************************************************************
//*************************************************************************************************************

void Pod::heuristicSimulate(Action* action) {
	Coords podTarget = calcPodTarget(action->getDirectionAngle());
	rotate(podTarget);

	int thrust = action->getThrust();
	if (action->getUseSheild()) {
		thrust = 0;
	}

	applyThrust(thrust);
	move(TURN_END_TIME);
}

//*************************************************************************************************************
//*************************************************************************************************************

int Pod::heuristicEval(Coords nextCheckPoint, MaximizeMinimize mm) const {
	int eval = 0;

	if (PR_MY_RUNNER == role || PR_ENEMY_RUNNER == role) {
		// TODO: Not sure if here have to handel enemy runner diffrentl, debugging will show
		float nextCPDistance = position.distance(nextCheckPoint);
		float angleToNextCP = calcAngleToTarget(nextCheckPoint);

		eval =
			INT_MAX -
			(DIST_TO_NEXT_CP_WEIGHT * (int)nextCPDistance) -
			(ANGLE_TO_NEXT_CP_WEIGHT * (int)angleToNextCP);
	}
	else {

	}

	return eval;
}

//*************************************************************************************************************
//*************************************************************************************************************

float Pod::clampAngle(float angleToClamp) const {
	float clampedAngle = angleToClamp;

	if (angleToClamp > MAX_ANGLE) {
		clampedAngle = angleToClamp - MAX_ANGLE;
	}
	else if (angleToClamp < MIN_ANGLE) {
		clampedAngle = angleToClamp + MAX_ANGLE;
	}

	return clampedAngle;
}

//*************************************************************************************************************
//*************************************************************************************************************

bool Pod::sheildOn() const {
	return shieldUp;
}

//*************************************************************************************************************
//*************************************************************************************************************

void Pod::debug() const {
	cerr << "Coords:(X = " << position.xCoord << ", Y = " << position.yCoord << ") ";
	cerr << "Speed:(X = " << speedVector.xCoord << ", Y = " << speedVector.yCoord << ") ";
	cerr << "Angle:" << angle << " ";
	//cerr << "Next CheckPointID: " << nextCheckPointId << endl;
	//cerr << "Turns left: " << turnsLeft << endl;
	cerr << endl;
}

//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------

class Collision {
public:
	Collision();
	Collision(Entity* entityA, Entity* entityB, float collisionTurnTime);
	~Collision();

	void setCollisionTurnTime(float collisionTurnTime) { this->collisionTurnTime = collisionTurnTime; }
	void setEntityA(Entity* entityA) { this->entityA = entityA; }
	void setEntityB(Entity* entityB) { this->entityB = entityB; }


	float getCollisinTurnTime() const { return collisionTurnTime; }
	Entity* getEntityA() const { return entityA; }
	Entity* getEntityB() const { return entityB; }

	bool isValid() const;
private:
	Entity* entityA;
	Entity* entityB;
	float collisionTurnTime;
};

//*************************************************************************************************************
//*************************************************************************************************************

Collision::Collision() : entityA(NULL), entityB(NULL), collisionTurnTime(0.f) {
}

//*************************************************************************************************************
//*************************************************************************************************************

Collision::Collision(Entity* entityA, Entity* entityB, float collisionTurnTime) :
	entityA(entityA),
	entityB(entityB),
	collisionTurnTime(collisionTurnTime)
{
}

//*************************************************************************************************************
//*************************************************************************************************************

Collision::~Collision() {
}

//*************************************************************************************************************
//*************************************************************************************************************

bool Collision::isValid() const {
	return entityA && entityB;
}

//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------

class State {
public:
	State();
	State(int podsCount);
	State(State* state);
	State(State* state, PodRole role);
	State(int podsCount, Pod** pods, int checkPointsCount, CheckPoint** checkPoints);
	~State();

	Pod** getPods() const { return pods; }
	CheckPoint** getCheckPoints() const { return checkPoints; }
	Pod* getPod(int id) const { return pods[id]; }
	CheckPoint* getCheckPoint(int id) const { return checkPoints[id]; }
	int getPodsCount() const { return podsCount; }
	int getCheckPointsCount() const { return checkPointsCount; }

	void setCheckPointsCount(int checkPointsCount) { this->checkPointsCount = checkPointsCount; }
	void setCheckPoints(CheckPoint** checkPoints) { this->checkPoints = checkPoints; }

	void initState(int podsCount);
	void simulateTurn(Action* podActions);
	Collision checkForCollision(Entity* entityA, Entity* entityB) const;
	void movePods();
	bool compareCollisions(Collision* collisionA, Collision* collisoionB) const;
	void assignRunerRole(PodRole runnerRole, PodRole hunterRole);
	void assignRoles();
	Pod* getPodByRole(PodRole role) const;
	Pod* getHunterWithMostCPs(PodRole hunterRole) const;
	Pod* getClosestToCPHunter(PodRole role) const;
	int getRolePodIdx(PodRole role) const;
	bool isTerminal() const;
	void turnEnd();
	void computeCheckPointCollision(Pod* pod, CheckPoint* checkPoint);

	void debug() const;

private:
	int podsCount;
	Pod** pods;

	int checkPointsCount;
	CheckPoint** checkPoints;
};

//*************************************************************************************************************
//*************************************************************************************************************

State::State() : podsCount(0), pods(NULL), checkPoints(NULL) {

}

//*************************************************************************************************************
//*************************************************************************************************************

State::State(int podsCount) :
	podsCount(podsCount)
{
	initState(podsCount);
}

//*************************************************************************************************************
//*************************************************************************************************************

State::State(State* state) {
	int sourcePodsCount = state->podsCount;
	initState(sourcePodsCount);

	for (int podIdx = 0; podIdx < sourcePodsCount; ++podIdx) {
		memcpy(this->pods[podIdx], state->pods[podIdx], sizeof(Pod));
	}

	this->podsCount = sourcePodsCount;
	this->checkPointsCount = state->checkPointsCount;
	this->checkPoints = state->checkPoints;
}

//*************************************************************************************************************
//*************************************************************************************************************

State::State(State* state, PodRole role) {
	initState(SUBSTATE_PODS_COUNT);

	int sourceMyPodIdx = INVALID_ID;
	int sourceEnemyPodIdx = INVALID_ID;

	if (PR_MY_RUNNER == role) {
		sourceMyPodIdx = state->getRolePodIdx(PR_MY_RUNNER);
		sourceEnemyPodIdx = state->getRolePodIdx(PR_ENEMY_HUNTER);
	}

	if (PR_MY_HUNTER == role) {
		sourceMyPodIdx = state->getRolePodIdx(PR_MY_HUNTER);
		sourceEnemyPodIdx = state->getRolePodIdx(PR_ENEMY_RUNNER);
	}

	memcpy(this->pods[TSPI_MY_POD_IDX], state->pods[sourceMyPodIdx], sizeof(Pod));
	memcpy(this->pods[TSPI_ENEMY_POD_IDX], state->pods[sourceEnemyPodIdx], sizeof(Pod));

	this->podsCount = SUBSTATE_PODS_COUNT;
	this->checkPointsCount = state->checkPointsCount;
	this->checkPoints = state->checkPoints;
}

//*************************************************************************************************************
//*************************************************************************************************************

State::State(int podsCount, Pod** pods, int checkPointsCount, CheckPoint** checkPoints) :
	podsCount(podsCount),
	pods(pods),
	checkPointsCount(checkPointsCount),
	checkPoints(checkPoints)
{
}

//*************************************************************************************************************
//*************************************************************************************************************

State::~State() {
	if (pods) {
		for (int podIdx = 0; podIdx < podsCount; ++podIdx) {
			delete pods[podIdx];
			pods[podIdx] = NULL;
		}

		delete[] pods;
		pods = NULL;
	}
}

//*************************************************************************************************************
//*************************************************************************************************************

void State::initState(int podsCount) {
	pods = new Pod*[podsCount];
	for (int podIdx = 0; podIdx < podsCount; ++podIdx) {
		pods[podIdx] = new Pod();
	}
}

//*************************************************************************************************************
//*************************************************************************************************************

void State::simulateTurn(Action* podActions) {
	for (int podIdx = 0; podIdx < podsCount; ++podIdx) {
		if (!podActions[podIdx].isValid()) {
			continue;
		}

		pods[podIdx]->rotate(podActions[podIdx].getTarget());

		if (podActions[podIdx].getUseSheild()) {
			pods[podIdx]->activateSheild();
		}

		int thrustToApply = podActions[podIdx].getThrust();
		if (pods[podIdx]->getSheildUp()) {
			thrustToApply = 0;
		}

		pods[podIdx]->applyThrust(thrustToApply);
	}

	movePods();

	for (int podIdx = 0; podIdx < podsCount; ++podIdx) {
		pods[podIdx]->end();
	}
}

//*************************************************************************************************************
//*************************************************************************************************************

Collision State::checkForCollision(Entity* entityA, Entity* entityB) const {

	// Square of the distance
	float dist = entityA->getPosition().distanceSquare(entityB->getPosition());

	// Sum of the radii squared
	float sr = (float)(entityA->getRadius() + entityB->getRadius()) * (entityA->getRadius() + entityB->getRadius());

	// We take everything squared to avoid calling sqrt uselessly. It is better for performances

	if (dist < sr) {
		// Objects are already touching each other. We have an immediate collision.		
		return Collision(entityA, entityB, 0.0);
	}

	// Optimisation. Objects with the same speed will never collide
	if (entityA->getSpeedVector().xCoord == entityB->getSpeedVector().xCoord &&
		entityA->getSpeedVector().yCoord == entityB->getSpeedVector().yCoord
		) {
		return Collision();
	}

	// We place ourselves in the reference frame of u. u is therefore stationary and is at (0,0)
	float x = entityA->getPosition().xCoord - entityB->getPosition().xCoord;
	float y = entityA->getPosition().yCoord - entityB->getPosition().yCoord;
	Coords myp(x, y);
	float vx = entityA->getSpeedVector().xCoord - entityB->getSpeedVector().xCoord;
	float vy = entityA->getSpeedVector().yCoord - entityB->getSpeedVector().yCoord;
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
		p.xCoord = p.xCoord - backdist * (vx / length);
		p.yCoord = p.yCoord - backdist * (vy / length);

		// If the point is now further away it means we are not going the right way, therefore the collision won't happen
		if (myp.distanceSquare(p) > mypdist) {
			return Collision();
		}

		pdist = p.distance(myp);

		// The point of impact is further than what we can travel in one turn
		if (pdist > length) {
			return Collision();
		}

		// Time needed to reach the impact point
		float t = pdist / length;

		return Collision(entityA, entityB, t);
	}

	return Collision();
}

//*************************************************************************************************************
//*************************************************************************************************************

void State::movePods() {
	// This tracks the time during the turn. The goal is to reach 1.0
	float t = TURN_START_TIME;

	Collision previousCollision;
	Collision firstCollision;

	while (t < TURN_END_TIME) {
		// We look for all the collisions that are going to occur during the turn
		for (int i = 0; i < podsCount; ++i) {
			// Collision with another pod?
			for (int j = i + 1; j < podsCount; ++j) {
				Collision col = checkForCollision(pods[i], pods[j]);

				if (col.isValid() && TURN_START_TIME == col.getCollisinTurnTime() && compareCollisions(&previousCollision, &col)) {
					col = Collision();
				}

				// If the collision occurs earlier than the one we currently have we keep it
				if (col.isValid() && col.getCollisinTurnTime() + t < TURN_END_TIME &&
					(!firstCollision.isValid() || col.getCollisinTurnTime() < firstCollision.getCollisinTurnTime())) {
					firstCollision = col;
				}
			}

			// Collision with another checkpoint?
			// It is unnecessary to check all checkpoints here. We only test the pod's next checkpoint.
			// We could look for the collisions of the pod with all the checkpoints, but if such a collision happens it wouldn't impact the game in any way
			Collision col = checkForCollision(pods[i], checkPoints[pods[i]->getNextCheckPointId()]);

			if (col.isValid() && previousCollision.isValid() && /*TURN_START_TIME == col->getCollisinTurnTime() &&*/ compareCollisions(&previousCollision, &col)) {
				col = Collision();
			}

			// If the collision happens earlier than the current one we keep it
			if (col.isValid() && col.getCollisinTurnTime() + t < TURN_END_TIME &&
				(!firstCollision.isValid() || col.getCollisinTurnTime() < firstCollision.getCollisinTurnTime())) {
				firstCollision = col;
			}
		}

		if (!firstCollision.isValid()) {
			// No collision, we can move the pods until the end of the turn
			for (int i = 0; i < podsCount; ++i) {
				pods[i]->move(TURN_END_TIME - t);
			}

			// End of the turn
			t = TURN_END_TIME;
		}
		else {
			// Move the pods to reach the time t of the collision
			for (int i = 0; i < podsCount; ++i) {
				pods[i]->move(firstCollision.getCollisinTurnTime());
			}

			CheckPoint* checkPoint = dynamic_cast<CheckPoint*>(firstCollision.getEntityB());
			Pod* pod = dynamic_cast<Pod*>(firstCollision.getEntityA());
			if (pod && checkPoint) {
				computeCheckPointCollision(pod, checkPoint);
			}
			else {
				// Play out the collision
				firstCollision.getEntityA()->computeBounce(firstCollision.getEntityB());
			}

			t += firstCollision.getCollisinTurnTime();
		}

		previousCollision = firstCollision;
		firstCollision = Collision();
	}

}

//*************************************************************************************************************
//*************************************************************************************************************

bool State::compareCollisions(Collision* collisionA, Collision* collisoionB) const {
	const bool sameEntityA = collisionA->getEntityA() == collisoionB->getEntityA();
	const bool sameEntityB = collisionA->getEntityB() == collisoionB->getEntityB();

	return (sameEntityA && sameEntityB);
}

//*************************************************************************************************************
//*************************************************************************************************************

Pod* State::getClosestToCPHunter(PodRole role) const {
	Pod* podRunner = NULL;

	float minDistance = MAX_DIST;

	for (int podIdx = 0; podIdx < podsCount; ++podIdx) {
		Pod* pod = pods[podIdx];
		if (pod->getRole() == role) {
			Coords podPosition = pod->getPosition();
			Coords nextCPPositon = checkPoints[pod->getNextCheckPointId()]->getPosition();
			float distanceToNextCP = podPosition.distance(nextCPPositon);
			if (distanceToNextCP < minDistance) {
				minDistance = distanceToNextCP;
				podRunner = pod;
			}
		}
	}

	return podRunner;
}

//*************************************************************************************************************
//*************************************************************************************************************

int State::getRolePodIdx(PodRole role) const {
	int idx = INVALID_ID;

	for (int podIdx = 0; podIdx < podsCount; ++podIdx) {
		if (pods[podIdx]->getRole() == role) {
			idx = podIdx;
			break;
		}
	}

	return idx;
}

//*************************************************************************************************************
//*************************************************************************************************************

bool State::isTerminal() const {
	bool terminal = false;

	for (int podIdx = 0; podIdx < podsCount; ++podIdx) {
		if (pods[podIdx]->getPassedCheckPoints() >= checkPointsCount || pods[podIdx]->getTurnsLeft() <= 0) {
			terminal = true;
			break;
		}
	}

	return terminal;
}

//*************************************************************************************************************
//*************************************************************************************************************

void State::turnEnd() {
}

//*************************************************************************************************************
//*************************************************************************************************************

void State::computeCheckPointCollision(Pod* pod, CheckPoint* checkPoint) {
	const int nextCPId =  pod->getNextCheckPointId();
	const int collisionCPId = checkPoint->getId();

	// If the collision is not with the next checkpoint nothing happens
	if (nextCPId == collisionCPId) {
		pod->resetCPCounter();
		pod->incrementPassedCPCounter();
	}
}

//*************************************************************************************************************
//*************************************************************************************************************

void State::assignRunerRole(PodRole runnerRole, PodRole hunterRole) {
	Pod* runner = getHunterWithMostCPs(hunterRole);

	// If the hunters are with equal checkPoints passed
	if (!runner) {
		runner = getClosestToCPHunter(hunterRole);
	}

	runner->setRole(runnerRole);
}

//*************************************************************************************************************
//*************************************************************************************************************

void State::assignRoles() {
	assignRunerRole(PR_MY_RUNNER, PR_MY_HUNTER);
	assignRunerRole(PR_ENEMY_RUNNER, PR_ENEMY_HUNTER);
}

//*************************************************************************************************************
//*************************************************************************************************************

Pod* State::getPodByRole(PodRole role) const {
	Pod* pod = NULL;

	for (int podIdx = 0; podIdx < podsCount; ++podIdx) {
		if (pods[podIdx]->getRole() == role) {
			pod = pods[podIdx];
			break;
		}
	}

	return pod;
}

//*************************************************************************************************************
//*************************************************************************************************************

Pod* State::getHunterWithMostCPs(PodRole hunterRole) const {
	Pod* pod = NULL;
	int maxCPs = INT_MIN;

	for (int podIdx = 0; podIdx < podsCount; ++podIdx) {
		if (pods[podIdx]->getRole() == hunterRole) {
			int podsCPs = pods[podIdx]->getPassedCheckPoints();

			// Two pods have same CPs
			if (podsCPs == maxCPs) {
				pod = NULL;
			}
			else if (podsCPs > maxCPs) {
				maxCPs = podsCPs;
				pod = pods[podIdx];
			}
		}
	}

	return pod;
}

//*************************************************************************************************************
//*************************************************************************************************************

void State::debug() const {
	for (int podIdx = 0; podIdx < podsCount; ++podIdx) {
		pods[podIdx]->debug();
	}
}

//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------

class Node {
public:
	Node();
	Node(
		Action action,
		State* state,
		Node* parent,
		int childrenCount,
		Node** children,
		int nodeDepth,
		char label
	);
	~Node();

	Node** getChildren() const { return children; };
	int getChildrenCount() const { return childrenCount; }
	State* getState() const { return state; }
	Action getAction() const { return action; }
	Node* getParent() const { return parent; }
	int getNodeDepth() const { return nodeDepth; }
	string getPathToNode() const { return pathToNode; }
	int getEvalValue() const { return evalValue; }

	void setAction(Action action) { this->action = action; }
	void setState(State* state) { this->state = state; }
	void setParent(Node* parent) { this->parent = parent; }
	void setChildrenCount(int childrenCount) { this->childrenCount = childrenCount; }
	void setChildren(Node** children) { this->children = children; }
	void setNodeDepth(int nodeDepth) { this->nodeDepth = nodeDepth; }
	void setEvalValue(int evalValue) { this->evalValue = evalValue; }

	void addChild(Node* newChild);
	Node* createChild(MaximizeMinimize mm, Action actionForChild, int actionIdx);
	void deleteChildren();
	Node* getChildI(int i);
	void copyState(State* state);
	void createChildren(Action* allPossibleActions, MaximizeMinimize mm);

	void setPathToNode();

private:
	// Action to get to node
	Action action;

	// Game state for two pods, NULL if my turn
	State* state;

	Node* parent; // Needed to backTrack the best leave node to parent

	int childrenCount;
	Node** children;

	int nodeDepth;
	
	// for debuging the tree
	char label;
	string pathToNode;
	int evalValue;
};

//*************************************************************************************************************
//*************************************************************************************************************

Node::Node() :
	action(),
	state(NULL),
	parent(NULL),
	childrenCount(0),
	children(NULL),
	nodeDepth(0),
	pathToNode(PARENT_PATH),
	label(PARENT_LABEL),
	evalValue(0)
{
}

//*************************************************************************************************************
//*************************************************************************************************************

Node::Node(
	Action action,
	State* state,
	Node* parent,
	int childrenCount,
	Node** children,
	int nodeDepth,
	char label
) :
	action(action),
	state(state),
	parent(parent),
	childrenCount(childrenCount),
	children(children),
	nodeDepth(nodeDepth),
	label(label)
{
}

//*************************************************************************************************************
//*************************************************************************************************************

Node::~Node() {
	// Max Nodes with not even depth are reusing the state of the Min parent Node and it is already deleted
	if (state && (0 == nodeDepth % 2)) {
		delete state;
		state = NULL;
	}

	// Children and parent will be deleted when deleting the whole tree
	// Action is not dynamically allocated

	pathToNode.clear();
}

//*************************************************************************************************************
//*************************************************************************************************************

void Node::addChild(Node* newChild) {
	++childrenCount;

	Node** temp = new Node*[childrenCount];

	// Redirect all pointers of the current children
	for (int childIdx = 0; childIdx < childrenCount - 1; ++childIdx) {
		temp[childIdx] = children[childIdx];
	}

	temp[childrenCount - 1] = newChild;

	delete[] children;
	children = temp;
}

//*************************************************************************************************************
//*************************************************************************************************************

Node* Node::createChild(MaximizeMinimize mm, Action actionForChild, int actionIdx) {
	Node* child = new Node(actionForChild, NULL, this, 0, NULL, nodeDepth + 1, 'A' + actionIdx);
	child->setPathToNode();

	if (MM_MAXIMIZE == mm) {
		// No need to copy/change the state for MAX
		child->setState(state);
	}
	else if (MM_MINIMIZE == mm) {
		// If minimize I need to generate simulate state with action for the enemy pod and the action from the parent node for my pod
		// Use child action and node action to simulate state for MIN
		Action actionForSimulation[SUBSTATE_PODS_COUNT] = { action, actionForChild };
		child->copyState(state);
		child->getState()->simulateTurn(actionForSimulation);
	}

	return child;
}

//*************************************************************************************************************
//*************************************************************************************************************

void Node::deleteChildren() {
	if (children) {
		for (int childIdx = 0; childIdx < childrenCount; ++childIdx) {
			if (children[childIdx]) {
				delete children[childIdx];
				children[childIdx] = NULL;
			}
		}

		delete children;
		children = NULL;
	}
}

//*************************************************************************************************************
//*************************************************************************************************************

Node* Node::getChildI(int i) {
	return children[i];
}

//*************************************************************************************************************
//*************************************************************************************************************

void Node::copyState(State* state) {
	this->state = new State(state);
}

//*************************************************************************************************************
//*************************************************************************************************************

void Node::createChildren(Action* allPossibleActions, MaximizeMinimize mm) {
	// Apply all actions to the current state and choose the best of them using light eval function
	for (int actionIdx = 0; actionIdx < ALL_POSSIBLE_POD_ACTIONS_COUNT; ++actionIdx) {
		Action actionForChild = allPossibleActions[actionIdx];

		Pod pod = *(state->getPod(TSPI_MY_POD_IDX));
	
		if (MM_MINIMIZE == mm) {
			pod = *(state->getPod(TSPI_ENEMY_POD_IDX));
		}

		pod.heuristicSimulate(&actionForChild);

		// May be here check for the distance between runner pod and next CP or between hunter and runner
		Coords nextCPCoords = state->getCheckPoint(pod.getNextCheckPointId())->getPosition();
		int hValue = pod.heuristicEval(nextCPCoords, mm);

		// If the tested action is good create a child and add it
		Node* child = createChild(mm, actionForChild, actionIdx);
		addChild(child);
	}
}

//*************************************************************************************************************
//*************************************************************************************************************

void Node::setPathToNode() {
	Node* n = NULL;
	Node* p = getParent();

	pathToNode.push_back(label);

	while (p) {
		n = p;
		pathToNode.push_back(n->label);
		p = n->getParent();
	}

	reverse(pathToNode.begin(), pathToNode.end());
}

//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------

struct MinMaxResult {
	MinMaxResult(
		Node* bestLeaveNode,
		int evaluationValue
	) :
		bestLeaveNode(bestLeaveNode),
		evaluationValue(evaluationValue)
	{}

	Node* bestLeaveNode;
	int evaluationValue;
};

//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------

class Minimax {
public:
	Minimax();
	Minimax(Node* tree, int maxTreeDepth);
	~Minimax();

	Action run(State* state, PodRole role, Action* allPossibleActions);
	Action backtrack(Node* node) const;
	void deleteTree(Node* node);
	void initTree();
	void clear();

	MinMaxResult maximize(Node* node, PodRole podRole, int alpha, int beta, Action* allPossibleActions);
	MinMaxResult minimize(Node* node, PodRole podRole, int alpha, int beta, Action* allPossibleActions);

	int evaluateState(Node* node, PodRole podRole) const;
	int evaluateRunnerState(State* state) const;
	int evaluateHunterState(State* state) const;

	void printTreeToFile(const string& fileName);
	void printChildren(Node* node, ofstream& file);
	int debugEval(const string& nodePath) const;
private:
	Node* tree;

	int maxTreeDepth;
};

//*************************************************************************************************************
//*************************************************************************************************************

Minimax::Minimax() : 
	tree(NULL),
	maxTreeDepth(0)
{
}

//*************************************************************************************************************
//*************************************************************************************************************

Minimax::Minimax(
	Node* tree,
	int maxTreeDepth
) :
	tree(tree),
	maxTreeDepth(maxTreeDepth)
{
}

//*************************************************************************************************************
//*************************************************************************************************************

Minimax::~Minimax() {
	// MiniMax tree is deleted in the game class after choosing an action
	//clear();
}

//*************************************************************************************************************
//*************************************************************************************************************

Action Minimax::run(State* state, PodRole podRole, Action* allPossibleActions) {
	tree->copyState(state);
	MinMaxResult minimaxRes = maximize(tree, podRole, INT_MIN, INT_MAX, allPossibleActions);

	//cerr << minimaxRes.bestLeaveNode->getPathToNode() << endl;

	return backtrack(minimaxRes.bestLeaveNode);
}

//*************************************************************************************************************
//*************************************************************************************************************

Action Minimax::backtrack(Node* node) const {
	//cerr << "LEAVE NODE: " << node->getPathToNode() << endl;

	Node* n = node;
	Node* p = node->getParent();

	while (p->getParent()) {
		n = p;
		p = n->getParent();
	}

	return n->getAction();
}

//*************************************************************************************************************
//*************************************************************************************************************

void Minimax::deleteTree(Node* node) {
	if (node) {
		int childrenCount = node->getChildrenCount();
		Node** children = node->getChildren();

		delete node;
		node = NULL;

		for (int childIdx = 0; childIdx < childrenCount; ++childIdx) {
			deleteTree(children[childIdx]);
		}

		delete[] children;
	}
}

//*************************************************************************************************************
//*************************************************************************************************************

void Minimax::initTree() {
	tree = new Node();
}

//*************************************************************************************************************
//*************************************************************************************************************

void Minimax::clear() {
	if (tree) {
		deleteTree(tree);
	}
}

//*************************************************************************************************************
//*************************************************************************************************************

MinMaxResult Minimax::maximize(Node* node, PodRole podRole, int alpha, int beta, Action* allPossibleActions) {
	if (node->getNodeDepth() == maxTreeDepth || node->getState()->isTerminal()) {
		int eval = evaluateState(node, podRole);
		MinMaxResult res = MinMaxResult(node, eval);
		
		node->setEvalValue(eval);

		return res;
	}

	MinMaxResult res = MinMaxResult(NULL, INT_MIN);

	node->createChildren(allPossibleActions, MM_MAXIMIZE);

	for (int childIdx = 0; childIdx < node->getChildrenCount(); ++childIdx) {
		MinMaxResult minRes = minimize(node->getChildI(childIdx), podRole, alpha, beta, allPossibleActions);

		if (minRes.evaluationValue > res.evaluationValue) {
			res = minRes;
		}

		if (res.evaluationValue >= beta) {
			break;
		}

		if (res.evaluationValue > alpha) {
			alpha = res.evaluationValue;
		}
	}

	node->setEvalValue(res.evaluationValue);
	return res;
}

//*************************************************************************************************************
//*************************************************************************************************************

MinMaxResult Minimax::minimize(Node* node, PodRole podRole, int alpha, int beta, Action* allPossibleActions) {
	if (node->getNodeDepth() == maxTreeDepth || node->getState()->isTerminal()) {
		int eval = evaluateState(node, podRole);
		MinMaxResult res = MinMaxResult(node, eval);

		node->setEvalValue(eval);

		return res;
	}

	MinMaxResult res = MinMaxResult(NULL, INT_MAX);

	node->createChildren(allPossibleActions, MM_MINIMIZE);

	for (int childIdx = 0; childIdx < POD_ACTIONS_COUNT; ++childIdx) {
		MinMaxResult maxRes = maximize(node->getChildI(childIdx), podRole, alpha, beta, allPossibleActions);

		if (maxRes.evaluationValue < res.evaluationValue) {
			res = maxRes;
		}

		if (res.evaluationValue <= alpha) {
			break;
		}

		if (res.evaluationValue < beta) {
			beta = res.evaluationValue;
		}
	}

	node->setEvalValue(res.evaluationValue);
	return res;
}

//*************************************************************************************************************
//*************************************************************************************************************

int Minimax::evaluateState(Node* node, PodRole podRole) const {
	int evaluation = 0;

	if (PR_MY_RUNNER == podRole) {
		evaluation = evaluateRunnerState(node->getState());
	}
	else if (PR_MY_HUNTER == podRole) {
		evaluation = evaluateHunterState(node->getState());
	}
	else {
		evaluation = debugEval(node->getPathToNode());
	}

	return evaluation;
}

//*************************************************************************************************************
//*************************************************************************************************************

int Minimax::evaluateRunnerState(State* state) const {
	int evalValue = 0;

	Pod* runner = state->getPodByRole(PR_MY_RUNNER);
	Pod* hunter = state->getPodByRole(PR_ENEMY_HUNTER);
	int passedCheckPoints = runner->getPassedCheckPoints();
	int cpsToWin = LAPS_COUNT * state->getCheckPointsCount();

	if (passedCheckPoints >= cpsToWin) {
		evalValue = INT_MAX;
	}
	else if (runner->getTurnsLeft() <= 0) {
		evalValue = INT_MIN;
	}
	else {
		int nextCpId = runner->getNextCheckPointId();
		CheckPoint* nextCp = state->getCheckPoint(nextCpId);
		Coords nextCpPosition = nextCp->getPosition();
		int distToNextCp = (int)runner->getPosition().distance(nextCpPosition);
		int angleToNextCp = (int)runner->calcAngleToTarget(nextCpPosition);
		int distToHunter = (int)runner->getPosition().distance(hunter->getPosition());
		int angleToHunter = (int)runner->calcAngleToTarget(hunter->getPosition());

		evalValue =
			(PASSED_CPS_WEIGHT * passedCheckPoints) -
			(DIST_TO_NEXT_CP_WEIGHT * distToNextCp) -
			(ANGLE_TO_NEXT_CP_WEIGHT * angleToNextCp) +
			(DIST_TO_HUNTER_WEIGHT * distToHunter) +
			(ANGLE_HUNTER_WEIGHT * angleToHunter);
	}

	return evalValue;
}

//*************************************************************************************************************
//*************************************************************************************************************

int Minimax::evaluateHunterState(State* state) const {
	int evalValue = 0;

	Pod* hunter = state->getPodByRole(PR_MY_HUNTER);
	Pod* runner = state->getPodByRole(PR_ENEMY_RUNNER);
	int passedCheckPoints = runner->getPassedCheckPoints();
	int cpsToWin = LAPS_COUNT * state->getCheckPointsCount();

	if (passedCheckPoints >= cpsToWin) {
		evalValue = INT_MIN;
	}
	else if (runner->getTurnsLeft() <= 0) {
		evalValue = INT_MAX;
	}
	else {
		int nextCpId = runner->getNextCheckPointId();
		CheckPoint* nextCp = state->getCheckPoint(nextCpId);
		Coords nextCpPosition = nextCp->getPosition();
		int distToNextCp = (int)runner->getPosition().distance(nextCpPosition);
		int angleToNextCp = (int)runner->calcAngleToTarget(nextCpPosition);
		int distToHunter = (int)runner->getPosition().distance(hunter->getPosition());
		int angleToHunter = (int)runner->calcAngleToTarget(hunter->getPosition());

		evalValue =
			(ENEMY_DIST_TO_NEXT_CP_WEIGHT * distToNextCp) +
			(ENEMY_ANGLE_TO_NEXT_CP_WEIGHT * angleToNextCp) -
			(DIST_TO_HUNTER_WEIGHT * distToHunter) -
			(ANGLE_HUNTER_WEIGHT * angleToHunter);
	}

	return evalValue;
}

//*************************************************************************************************************
//*************************************************************************************************************

void Minimax::printChildren(Node* node, ofstream& file) {
	string nodePath = node->getPathToNode();

	file << nodePath << " [label=\"";
	file << nodePath << "\\n";
	file << node->getEvalValue();
	file << "\"]\n";

	for (int childIdx = 0; childIdx < node->getChildrenCount(); ++childIdx) {
		Node* child = node->getChildI(childIdx);
		string childPath = child->getPathToNode();

		file << nodePath << "->" << childPath;
		//file << " [label=\"";
		//child->getAction().debugPrint(file);
		//file << "\"]\n";
		file << endl;

		printChildren(child, file);
	}
}

//*************************************************************************************************************
//*************************************************************************************************************

void Minimax::printTreeToFile(const string& fileName) {
	ofstream file;
	file.open(fileName.c_str());
	file << "digraph mytree{\n";

	printChildren(tree, file);

	file << "}";
	file.close();
}

//*************************************************************************************************************
//*************************************************************************************************************

int Minimax::debugEval(const string& nodePath) const {
	int rInt = 0;

	if ("PAAAA" == nodePath) { rInt = 41; }
	else if ("PAAAB" == nodePath) { rInt = 67; }
	else if ("PAAAC" == nodePath) { rInt = 34; }
	else if ("PAABA" == nodePath) { rInt = 0; }
	else if ("PAABB" == nodePath) { rInt = 69; }
	else if ("PAABC" == nodePath) { rInt = 24; }
	else if ("PAACA" == nodePath) { rInt = 78; }
	else if ("PAACB" == nodePath) { rInt = 58; }
	else if ("PAACC" == nodePath) { rInt = 62; }
	else if ("PABAA" == nodePath) { rInt = 64; }
	else if ("PABAB" == nodePath) { rInt = 5; }
	else if ("PABAC" == nodePath) { rInt = 45; }
	else if ("PABBA" == nodePath) { rInt = 81; }
	else if ("PABBB" == nodePath) { rInt = 27; }
	else if ("PABBC" == nodePath) { rInt = 61; }
	else if ("PABCA" == nodePath) { rInt = 91; }
	else if ("PABCB" == nodePath) { rInt = 95; }
	else if ("PABCC" == nodePath) { rInt = 42; }
	else if ("PACAA" == nodePath) { rInt = 27; }
	else if ("PACAB" == nodePath) { rInt = 36; }
	else if ("PACAC" == nodePath) { rInt = 91; }
	else if ("PACBA" == nodePath) { rInt = 4; }
	else if ("PACBB" == nodePath) { rInt = 2; }
	else if ("PACBC" == nodePath) { rInt = 53; }
	else if ("PACCA" == nodePath) { rInt = 92; }
	else if ("PACCB" == nodePath) { rInt = 82; }
	else if ("PACCC" == nodePath) { rInt = 21; }
	else if ("PBAAA" == nodePath) { rInt = 16; }
	else if ("PBAAB" == nodePath) { rInt = 18; }
	else if ("PBAAC" == nodePath) { rInt = 95; }
	else if ("PBABA" == nodePath) { rInt = 47; }
	else if ("PBABB" == nodePath) { rInt = 26; }
	else if ("PBABC" == nodePath) { rInt = 71; }
	else if ("PBACA" == nodePath) { rInt = 38; }
	else if ("PBACB" == nodePath) { rInt = 69; }
	else if ("PBACC" == nodePath) { rInt = 12; }
	else if ("PBBAA" == nodePath) { rInt = 67; }
	else if ("PBBAB" == nodePath) { rInt = 99; }
	else if ("PBBAC" == nodePath) { rInt = 35; }
	else if ("PBBBA" == nodePath) { rInt = 94; }
	else if ("PBBBB" == nodePath) { rInt = 3; }
	else if ("PBBBC" == nodePath) { rInt = 11; }
	else if ("PBBCA" == nodePath) { rInt = 22; }
	else if ("PBBCB" == nodePath) { rInt = 33; }
	else if ("PBBCC" == nodePath) { rInt = 73; }
	else if ("PBCAA" == nodePath) { rInt = 64; }
	else if ("PBCAB" == nodePath) { rInt = 41; }
	else if ("PBCAC" == nodePath) { rInt = 11; }
	else if ("PBCBA" == nodePath) { rInt = 53; }
	else if ("PBCBB" == nodePath) { rInt = 68; }
	else if ("PBCBC" == nodePath) { rInt = 47; }
	else if ("PBCCA" == nodePath) { rInt = 44; }
	else if ("PBCCB" == nodePath) { rInt = 62; }
	else if ("PBCCC" == nodePath) { rInt = 57; }
	else if ("PCAAA" == nodePath) { rInt = 37; }
	else if ("PCAAB" == nodePath) { rInt = 59; }
	else if ("PCAAC" == nodePath) { rInt = 23; }
	else if ("PCABA" == nodePath) { rInt = 41; }
	else if ("PCABB" == nodePath) { rInt = 29; }
	else if ("PCABC" == nodePath) { rInt = 78; }
	else if ("PCACA" == nodePath) { rInt = 16; }
	else if ("PCACB" == nodePath) { rInt = 35; }
	else if ("PCACC" == nodePath) { rInt = 90; }
	else if ("PCBAA" == nodePath) { rInt = 42; }
	else if ("PCBAB" == nodePath) { rInt = 88; }
	else if ("PCBAC" == nodePath) { rInt = 6; }
	else if ("PCBBA" == nodePath) { rInt = 40; }
	else if ("PCBBB" == nodePath) { rInt = 42; }
	else if ("PCBBC" == nodePath) { rInt = 64; }
	else if ("PCBCA" == nodePath) { rInt = 48; }
	else if ("PCBCB" == nodePath) { rInt = 46; }
	else if ("PCBCC" == nodePath) { rInt = 5; }
	else if ("PCCAA" == nodePath) { rInt = 90; }
	else if ("PCCAB" == nodePath) { rInt = 29; }
	else if ("PCCAC" == nodePath) { rInt = 70; }
	else if ("PCCBA" == nodePath) { rInt = 50; }
	else if ("PCCBB" == nodePath) { rInt = 6; }
	else if ("PCCBC" == nodePath) { rInt = 1; }
	else if ("PCCCA" == nodePath) { rInt = 93; }
	else if ("PCCCB" == nodePath) { rInt = 48; }
	else if ("PCCCC" == nodePath) { rInt = 29; }

	return rInt;
}

//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------

class Game {
public:
	Game();
	~Game();

	void initGame();
	void gameLoop();
	void getGameInput();
	void getTurnInput();
	void turnBegin();
	void makeTurn();
	void turnEnd();
	void play();

	void makeFirstTurn() const;
	Action chooseAction(State* state, PodRole role);
	void makeSubStates();
	void clearSubStates();
	void resetMiniMax();
	void fillAllPossibleActions();

	void getHardCodedInput(int podIdx);
	void assignInput(
		int inPodXCoord,
		int inPodYCoord,
		int inPodVx,
		int inPodVy,
		int inPodAngle,
		int inPodNextCheckPointId
	);
	void debug() const;
	void debugCheckPoints() const;

private:
	int turnsCount;
	int lapsCount;
	int checkPointsCount;
	int lastTurnPodsGoalCPs[GAME_PODS_COUNT];
	CheckPoint** checkPoints;
	State* turnState;
	State* myRunnerSubState;
	State* myHunterSubState;
	Minimax* minimax;
	Action allPossibleActions[ALL_POSSIBLE_POD_ACTIONS_COUNT];

	// Turn inputs
	int podXCoord, podYCoord, podVx, podVy, podAngle, podNextCheckPointId;
};

//*************************************************************************************************************
//*************************************************************************************************************

Game::Game() :
	turnsCount(0),
	lapsCount(0),
	checkPointsCount(0),
	checkPoints(NULL),
	turnState(NULL),
	myRunnerSubState(NULL),
	myHunterSubState(NULL),
	minimax(NULL)
{
	for (int podIdx = 0; podIdx < GAME_PODS_COUNT; ++podIdx) {
		lastTurnPodsGoalCPs[podIdx] = FIRST_GOAL_CP_ID;
	}
}

//*************************************************************************************************************
//*************************************************************************************************************

Game::~Game() {
	if (turnState) {
		delete turnState;
		turnState = NULL;
	}

	if (minimax) {
		delete minimax;
		minimax = NULL;
	}

	clearSubStates();

	if (checkPoints) {
		for (int cpIdx = 0; cpIdx < checkPointsCount; ++cpIdx) {
			delete checkPoints[cpIdx];
			checkPoints[cpIdx] = NULL;
		}

		delete[] checkPoints;
		checkPoints = NULL;
	}
}

//*************************************************************************************************************
//*************************************************************************************************************

void Game::initGame() {
	turnState = new State(GAME_PODS_COUNT);
	fillAllPossibleActions();
}

//*************************************************************************************************************
//*************************************************************************************************************

void Game::gameLoop() {
	while (true) {
		clock_t begin = clock();

		getTurnInput();
		turnBegin();
		makeTurn();
		turnEnd();

		if (USE_HARDCODED_INPUT) {
			clock_t end = clock();
			double elapsedMilliSecs = double(end - begin);
		
			cerr << endl;
			cerr << "Turn " << turnsCount << " milliseconds: " << elapsedMilliSecs << endl;
			cerr << endl;

			// Profiling
			if (SIM_TURNS == turnsCount) {
				break;
			}
		}
	}
}

//*************************************************************************************************************
//*************************************************************************************************************

void Game::getGameInput() {
	if (USE_HARDCODED_INPUT) {
		lapsCount = 3; // Profiling
		checkPointsCount = 4; // Profiling
	}
	else {
		cin >> lapsCount;
		cin >> checkPointsCount;
		//cerr << lapsCount << endl;
		//cerr << checkPointsCount << endl;
	}

	checkPoints = new CheckPoint*[checkPointsCount];

	int checkPointXCoord, checkPointYCoord;
	for (int cpIdx = 0; cpIdx < checkPointsCount; ++cpIdx) {
		if (USE_HARDCODED_INPUT) {
			if (0 == cpIdx) { checkPointXCoord = 7982; checkPointYCoord = 7873; }
			if (1 == cpIdx) { checkPointXCoord = 13284; checkPointYCoord = 5513; }
			if (2 == cpIdx) { checkPointXCoord = 9539; checkPointYCoord = 1380; }
			if (3 == cpIdx) { checkPointXCoord = 3637; checkPointYCoord = 7873; }
		}
		else {
			cin >> checkPointXCoord >> checkPointYCoord;
			//cerr << checkPointXCoord << " " << checkPointYCoord << endl;
		}

		checkPoints[cpIdx] = new CheckPoint(Coords((float)checkPointXCoord, (float)checkPointYCoord), Coords(), CHECKPOINT_RADIUS, cpIdx);
	}

	turnState->setCheckPointsCount(checkPointsCount);
	turnState->setCheckPoints(checkPoints);
}

//*************************************************************************************************************
//*************************************************************************************************************

void Game::getHardCodedInput(int podIdx) {
	switch (turnsCount) {
	case FIRST_TURN:
		if (0 == podIdx) { assignInput(7779, 7416, 0, 0, -1, 1); }
		if (1 == podIdx) { assignInput(8185, 8330, 0, 0, -1, 1); }
		if (2 == podIdx) { assignInput(7372, 6503, 0, 0, -1, 1); }
		if (3 == podIdx) { assignInput(8592, 9243, 0, 0, -1, 1); }
		break;
	case 1:
		if (0 == podIdx) { assignInput(7874, 7383, 80, -27, 341, 1); }
		if (1 == podIdx) { assignInput(8754, 8016, 483, -267, 331, 1); }
		if (2 == podIdx) { assignInput(7471, 6486, 83, -14, 350, 1); }
		if (3 == podIdx) { assignInput(8670, 9181, 66, -52, 322, 1); }
		break;
	case 2:
		if (0 == podIdx) { assignInput(8049, 7324, 148, -50, 341, 1); }
		if (1 == podIdx) { assignInput(9305, 7676, 468, -289, 313, 1); }
		if (2 == podIdx) { assignInput(8195, 6365, 615, -103, 351, 1); }
		if (3 == podIdx) { assignInput(8814, 9067, 122, -97, 322, 1); }
		break;
	case 3:
		if (0 == podIdx) { assignInput(8279, 7216, 195, -91, 325, 1); }
		if (1 == podIdx) { assignInput(9860, 7337, 471, -287, 330, 1); }
		if (2 == podIdx) { assignInput(8909, 6245, 606, -101, 350, 1); }
		if (3 == podIdx) { assignInput(9014, 8908, 170, -135, 322, 1); }
		break;
	case 4:
		if (0 == podIdx) { assignInput(8567, 7088, 244, -109, 338, 1); }
		if (1 == podIdx) { assignInput(10398, 6976, 457, -306, 312, 1); }
		if (2 == podIdx) { assignInput(9614, 6127, 598, -99, 350, 1); }
		if (3 == podIdx) { assignInput(9262, 8711, 211, -167, 322, 1); }
		break;
	case 5:
		if (0 == podIdx) { assignInput(8891, 6920, 275, -143, 324, 1); }
		if (1 == podIdx) { assignInput(10940, 6617, 460, -305, 328, 1); }
		if (2 == podIdx) { assignInput(10311, 6011, 592, -98, 350, 1); }
		if (3 == podIdx) { assignInput(9551, 8482, 245, -194, 321, 1); }
		break;
	case 6:
		if (0 == podIdx) { assignInput(9257, 6734, 310, -157, 335, 1); }
		if (1 == podIdx) { assignInput(11613, 6373, 617, -165, 310, 1); }
		if (2 == podIdx) { assignInput(10843, 5729, 406, -281, 332, 1); }
		if (3 == podIdx) { assignInput(9874, 8226, 274, -217, 321, 1); }
		break;
	case 7:
		if (0 == podIdx) { assignInput(9647, 6517, 331, -184, 323, 1); }
		if (1 == podIdx) { assignInput(12308, 6145, 590, -193, 321, 1); }
		if (2 == podIdx) { assignInput(11348, 5431, 428, -253, 350, 1); }
		if (3 == podIdx) { assignInput(10226, 7947, 299, -237, 321, 1); }
		break;
	case 8:
		if (0 == podIdx) { assignInput(10054, 6268, 346, -211, 320, 1); }
		if (1 == podIdx) { assignInput(12953, 5868, 547, -235, 303, 2); }
		if (2 == podIdx) { assignInput(11875, 5192, 447, -202, 8, 1); }
		if (3 == podIdx) { assignInput(10603, 7648, 320, -254, 321, 1); }
		break;
	case 9:
		if (0 == podIdx) { assignInput(10477, 5994, 359, -233, 321, 1); }
		if (1 == podIdx) { assignInput(13531, 5538, 491, -280, 288, 2); }
		if (2 == podIdx) { assignInput(12412, 5034, 456, -134, 26, 1); }
		if (3 == podIdx) { assignInput(11001, 7332, 338, -268, 321, 1); }
		break;
	case 10:
		if (0 == podIdx) { assignInput(10902, 5686, 361, -261, 311, 1); }
		if (1 == podIdx) { assignInput(14052, 5162, 442, -319, 287, 2); }
		if (2 == podIdx) { assignInput(12940, 4970, 448, -54, 44, 1); }
		if (3 == podIdx) { assignInput(11417, 7001, 353, -280, 321, 1); }
		break;
	case 11:
		if (0 == podIdx) { assignInput(11329, 5350, 362, -286, 311, 1); }
		if (1 == podIdx) { assignInput(14498, 4743, 379, -356, 272, 2); }
		if (2 == podIdx) { assignInput(13434, 5005, 420, 29, 62, 2); }
		if (3 == podIdx) { assignInput(11825, 6637, 346, -309, 303, 1); }
		break;
	default:
		break;
	}
}

//*************************************************************************************************************
//*************************************************************************************************************

void Game::getTurnInput() {
	//cerr << "case " << turnsCount << ":" << endl;
	for (int podIdx = 0; podIdx < GAME_PODS_COUNT; ++podIdx) {

		if (USE_HARDCODED_INPUT) {
			getHardCodedInput(podIdx);
		}
		else {
			cin >> podXCoord >> podYCoord >> podVx >> podVy >> podAngle >> podNextCheckPointId;
			//cerr << "\tif (" << podIdx << "== podIdx) { assignInput(" << podXCoord << ", " << podYCoord << ", " << podVx << ", " << podVy << ", " << podAngle << ", " << podNextCheckPointId << "); }" << endl;
		}
		
		Pod** pods = turnState->getPods();
		pods[podIdx]->setPosition(Coords((float)podXCoord, (float)podYCoord));
		pods[podIdx]->setSpeedVector(Coords((float)podVx, (float)podVy));
		pods[podIdx]->setAngle((float)podAngle);
		pods[podIdx]->setNextCheckPointId(podNextCheckPointId);

		if (lastTurnPodsGoalCPs[podIdx] != podNextCheckPointId) {
			pods[podIdx]->incrementPassedCPCounter();
			pods[podIdx]->resetCPCounter();
			lastTurnPodsGoalCPs[podIdx] = podNextCheckPointId;
		}
		else if (FIRST_TURN != turnsCount){
			pods[podIdx]->decreaseTurnsLeft();
		}

		PodRole role = PR_MY_HUNTER;
		if (podIdx >= TEAM_PODS_COUNT) {
			role = PR_ENEMY_HUNTER;
		}

		pods[podIdx]->setRole(role);
	}
	//cerr << "break;" << endl;
}

//*************************************************************************************************************
//*************************************************************************************************************

void Game::turnBegin() {
	if (FIRST_TURN != turnsCount) {
		turnState->assignRoles();
		makeSubStates();
	}
}

//*************************************************************************************************************
//*************************************************************************************************************

void Game::makeTurn() {
	if (FIRST_TURN == turnsCount) {
		makeFirstTurn();
	}
	else {
		PodRole runnerRole = USE_INVALID_ROLES ? PR_INVALID : PR_MY_RUNNER;
		PodRole hunterRole = USE_INVALID_ROLES ? PR_INVALID : PR_MY_HUNTER;

		// MiniMax
		Action runnerAction = chooseAction(myRunnerSubState, runnerRole);
		resetMiniMax();

		Action hunterAction = chooseAction(myHunterSubState, hunterRole);
		resetMiniMax();

		runnerAction.printAction();
		hunterAction.printAction();
	}
}

//*************************************************************************************************************
//*************************************************************************************************************

void Game::turnEnd() {
	if (FIRST_TURN != turnsCount) {
		clearSubStates();
	}

	++turnsCount;
}

//*************************************************************************************************************
//*************************************************************************************************************

void Game::play() {
	initGame();
	getGameInput();
	gameLoop();
}

//*************************************************************************************************************
//*************************************************************************************************************

void Game::makeFirstTurn() const {
	// Aim the two pods to the first CheckPoint
	CheckPoint* firstCheckPoint = turnState->getCheckPoint(FIRST_GOAL_CP_ID);
	Coords firstCheckPointCoords = firstCheckPoint->getPosition();
	cout << firstCheckPointCoords.xCoord << ' ' << firstCheckPointCoords.yCoord << ' ' << MAX_THRUST << endl;
	cout << firstCheckPointCoords.xCoord << ' ' << firstCheckPointCoords.yCoord << ' ' << BOOST << endl;
}

//*************************************************************************************************************
//*************************************************************************************************************

Action Game::chooseAction(State* state, PodRole role) {
	minimax = new Minimax(NULL, MINIMAX_DEPTH);
	minimax->initTree();
	Action result = minimax->run(state, role, allPossibleActions);

	if (PRINT_MINIMAX_TREE_TO_FILE) {
		string fileForTree = RUNNER_TREE_FILE;

		if (PR_MY_HUNTER == role) {
			fileForTree = HUNTER_TREE_FILE;
		}

		minimax->printTreeToFile(fileForTree);
	}

	return result;
}

//*************************************************************************************************************
//*************************************************************************************************************

void Game::makeSubStates() {
	myRunnerSubState = new State(turnState, PR_MY_RUNNER);
	myHunterSubState = new State(turnState, PR_MY_HUNTER);
}

//*************************************************************************************************************
//*************************************************************************************************************

void Game::clearSubStates() {
	if (myRunnerSubState) {
		delete myRunnerSubState;
		myRunnerSubState = NULL;
	}

	if (myHunterSubState) {
		delete myHunterSubState;
		myHunterSubState = NULL;
	}
}

//*************************************************************************************************************
//*************************************************************************************************************

void Game::resetMiniMax(){
	if (minimax) {
		minimax->clear();
		delete minimax;
		minimax = NULL;
	}
}

//*************************************************************************************************************
//*************************************************************************************************************

void Game::fillAllPossibleActions() {
	int actionIdx = 0;

	for (int dirIdx = 0; dirIdx < POD_DIRECTIONS_COUNT; ++dirIdx) {
		for (int thrustIdx = 0; thrustIdx < POD_THRUSTS_COUNT; ++thrustIdx) {
			for (int sheildFlagIdx = 0; sheildFlagIdx < POD_SHEILD_FLAGS_COUNT; ++sheildFlagIdx) {
				allPossibleActions[actionIdx].setDirectionAngle(DIRECTION_ANGLES[dirIdx]);
				allPossibleActions[actionIdx].setThrust(THRUST_VALUES[thrustIdx]);
				allPossibleActions[actionIdx].setUseSheild(SHEILD_FLAGS[sheildFlagIdx]);

				++actionIdx;
			}
		}
	}
}

//*************************************************************************************************************
//*************************************************************************************************************

void Game::assignInput(
	int inPodXCoord,
	int inPodYCoord,
	int inPodVx,
	int inPodVy,
	int inPodAngle,
	int inPodNextCheckPointId
) {
	podXCoord = inPodXCoord;
	podYCoord = inPodYCoord;
	podVx = inPodVx;
	podVy = inPodVy;
	podAngle = inPodAngle;
	podNextCheckPointId = inPodNextCheckPointId;
}

//*************************************************************************************************************
//*************************************************************************************************************

void Game::debug() const {
	turnState->debug();
}

//*************************************************************************************************************
//*************************************************************************************************************

void Game::debugCheckPoints() const {
	for (int checkPointIdx = 0; checkPointIdx < checkPointsCount; ++checkPointIdx) {
		checkPoints[checkPointIdx]->debug();
	}
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
*/