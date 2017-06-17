#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <climits>

#define M_PI 3.14159265358979323846

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
//const int POD_ACTIONS_COUNT = 7;
const int POD_ACTIONS_COUNT = 3; // For debuging
const int FIRST_GOAL_CP_ID = 1;

const int MINIMAX_DEPTH = 4;
const int LAPS_COUNT = 3;

const int PASSED_CPS_WEIGHT = 5000;
const int DIST_TO_NEXT_CP_WEIGHT = 3;
const int ANGLE_TO_NEXT_CP_WEIGHT = 200;
const int DIST_TO_HUNTER_WEIGHT = 5;
const int ANGLE_HUNTER_WEIGHT = 100;

const int ENEMY_DIST_TO_NEXT_CP_WEIGHT = 200;
const int ENEMY_ANGLE_TO_NEXT_CP_WEIGHT = 1500;

const string SHEILD = "SHEILD";
const string BOOST = "BOOST";

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
	PD_LEFT = 0,
	PD_FORWARD,
	PD_RIGHT,
};

enum ActionType {
	AT_INVALID_ACTION = -1,
	AT_LEFT_MAX_SPEED = 0,
	AT_LEFT_MIN_SPEED,
	AT_LEFT_SHEILD,
	AT_RIGHT_MAX_SPEED,
	AT_RIGHT_MIN_SPEED,
	AT_RIGHT_SHEILD,
	AT_FORWARD_NAX_SPEED,
};

//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------

struct Coords {
	Coords() : xCoord(INVALID_COORD), yCoord(INVALID_COORD) {}
	Coords(float xCoord, float yCoord) : xCoord(xCoord), yCoord(yCoord) {}

	float distanceSquare(Coords p) const;
	float distance(Coords p) const;
	Coords closestPointOnLine(Coords linePointA, Coords linePointB) const;

	float xCoord;
	float yCoord;
};

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
	Action(Coords target, bool useSheild, int thrust, ActionType type);
	~Action();

	Coords getTarget() const { return target; }
	bool getUseSheild() const { return useSheild; }
	int getThrust() const { return thrust; }
	void fillAction(Coords target, bool useSheild, int thrust, ActionType type);

	void printAction() const;

	void debug() const;
private:
	Coords target;
	bool useSheild;
	int thrust;
	ActionType type;
};

//*************************************************************************************************************
//*************************************************************************************************************

Action::Action() : target(), useSheild(false), thrust(0), type(AT_INVALID_ACTION) {

}

//*************************************************************************************************************
//*************************************************************************************************************

Action::Action(Coords target, bool useSheild, int thrust, ActionType type) :
	target(target),
	useSheild(useSheild),
	thrust(thrust),
	type(type)
{
}

//*************************************************************************************************************
//*************************************************************************************************************

Action::~Action() {

}

//*************************************************************************************************************
//*************************************************************************************************************

void Action::fillAction(Coords target, bool useSheild, int thrust, ActionType type) {
	this->target = target;
	this->useSheild = useSheild;
	this->thrust = thrust;
	this->type = type;
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
	cerr << "ACTION_TYPE: " << type << endl;
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

	int getNextCheckPointId() const { return nextCheckPointId; }
	int getTurnsLeft() const { return turnsLeft; }
	int getSheildUp() const { return shieldUp; }
	PodRole getRole() const { return role; }
	int getPassedCheckPoints() const { return passedCheckPoints; }

	Action getTurnAction(int actionIdx) const;
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
	void generateTurnActions();
	float clampAngle(float angleToClamp) const;
	Coords calcPodTarget(PodDirection podDirection);
	void initActions();
	void incrementPassedCPCounter();
	void decreaseTuensLeft();

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
	Action turnActions[POD_ACTIONS_COUNT];
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
	initActions();
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

Action Pod::getTurnAction(int actionIdx) const {
	return turnActions[actionIdx];
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

void Pod::generateTurnActions() {
	Coords podLeftTarget = calcPodTarget(PD_LEFT);
	Coords podForwardTarget = calcPodTarget(PD_FORWARD);
	Coords podRightTarget = calcPodTarget(PD_RIGHT);

	turnActions[0].fillAction(podLeftTarget, false, MAX_THRUST, AT_LEFT_MAX_SPEED);
	turnActions[1].fillAction(podRightTarget, true, 0, AT_RIGHT_SHEILD);
	turnActions[2].fillAction(podForwardTarget, false, MAX_THRUST, AT_FORWARD_NAX_SPEED);

	//turnActions[0].fillAction(podLeftTarget, false, MAX_THRUST, AT_LEFT_MAX_SPEED);
	//turnActions[1].fillAction(podLeftTarget, false, 0, AT_LEFT_MIN_SPEED);
	//turnActions[2].fillAction(podLeftTarget, true, 0, AT_LEFT_SHEILD);
	//turnActions[3].fillAction(podRightTarget, false, MAX_THRUST, AT_RIGHT_MAX_SPEED);
	//turnActions[4].fillAction(podRightTarget, false, 0, AT_RIGHT_MIN_SPEED);
	//turnActions[5].fillAction(podRightTarget, true, 0, AT_RIGHT_SHEILD);
	//turnActions[6].fillAction(podForwardTarget, false, MAX_THRUST, AT_FORWARD_NAX_SPEED);
}

//*************************************************************************************************************
//*************************************************************************************************************

Coords Pod::calcPodTarget(PodDirection podDirection) {
	float podTargetAngle = 0.f;

	switch (podDirection)
	{
	case PD_LEFT:
		podTargetAngle = clampAngle(angle - MAX_ANGLE_PER_TURN);
		break;
	case PD_FORWARD:
		podTargetAngle = clampAngle(angle);
		break;
	case PD_RIGHT:
		podTargetAngle = clampAngle(angle + MAX_ANGLE_PER_TURN);
		break;
	default:
		break;
	}

	float angleRadians = podTargetAngle * ((float)M_PI / (MAX_ANGLE / 2));

	float sinLeftOffset = sin(angleRadians);
	float cosLeftOffset = cos(angleRadians);

	sinLeftOffset *= TARGET_OFFSET;
	cosLeftOffset *= TARGET_OFFSET;

	return Coords(cosLeftOffset + position.xCoord, sinLeftOffset + position.yCoord);
}

//*************************************************************************************************************
//*************************************************************************************************************

void Pod::initActions() {
	for (int actionIdx = 0; actionIdx < POD_ACTIONS_COUNT; ++actionIdx) {
		turnActions[actionIdx] = Action();
	}
}

//*************************************************************************************************************
//*************************************************************************************************************

void Pod::incrementPassedCPCounter() {
	++passedCheckPoints;
}

//*************************************************************************************************************
//*************************************************************************************************************

void Pod::decreaseTuensLeft() {
	--turnsLeft;
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

	float getCollisinTurnTime() const { return collisionTurnTime; }
	Entity* getEntityA() const { return entityA; }
	Entity* getEntityB() const { return entityB; }
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
	if (entityA) {
		delete entityA;
		entityA = NULL;
	}

	if (entityB) {
		delete entityB;
		entityB = NULL;
	}
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
	State(int podsCount, Pod** pods, CheckPoint** checkPoints);
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
	Collision* checkForCollision(Entity* entityA, Entity* entityB) const;
	void movePods();
	bool compareCollisions(Collision* collisionA, Collision* collisoionB) const;
	void assignRunerRole(PodRole runnerRole, PodRole hunterRole);
	void assignRoles();
	Pod* getPodByRole(PodRole role) const;
	Pod* getHunterWithMostCPs(PodRole hunterRole) const;
	Pod* getClosestToCPHunter(PodRole role) const;
	int getRolePodIdx(PodRole role) const;
	bool isTerminal() const;
	void generatePodsTurnActions();
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

State::State(int podsCount, Pod** pods, CheckPoint** checkPoints) :
	podsCount(podsCount),
	pods(pods),
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
		pods[podIdx]->rotate(podActions[podIdx].getTarget());

		if (podActions[podIdx].getUseSheild()) {
			pods[podIdx]->activateSheild();
		}

		int thrustToApply = podActions[podIdx].getThrust();
		if (pods[podIdx]->getSheildUp()) {
			thrustToApply = 0;
		}

		pods[podIdx]->applyThrust(thrustToApply);

		// After a pod is simulated we must compute new targets
		pods[podIdx]->generateTurnActions();
	}

	movePods();

	for (int podIdx = 0; podIdx < podsCount; ++podIdx) {
		pods[podIdx]->end();
	}
}

//*************************************************************************************************************
//*************************************************************************************************************

Collision* State::checkForCollision(Entity* entityA, Entity* entityB) const {
	// Square of the distance
	float dist = entityA->getPosition().distanceSquare(entityB->getPosition());

	// Sum of the radii squared
	float sr = (float)(entityA->getRadius() + entityB->getRadius()) * (entityA->getRadius() + entityB->getRadius());

	// We take everything squared to avoid calling sqrt uselessly. It is better for performances

	if (dist < sr) {
		// Objects are already touching each other. We have an immediate collision.
		return new Collision(entityA, entityB, 0.0);
	}

	// Optimisation. Objects with the same speed will never collide
	if (entityA->getSpeedVector().xCoord == entityB->getSpeedVector().xCoord &&
		entityA->getSpeedVector().yCoord == entityB->getSpeedVector().yCoord
		) {
		return NULL;
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
			return NULL;
		}

		pdist = p.distance(myp);

		// The point of impact is further than what we can travel in one turn
		if (pdist > length) {
			return NULL;
		}

		// Time needed to reach the impact point
		float t = pdist / length;

		return new Collision(entityA, entityB, t);
	}

	return NULL;
}

//*************************************************************************************************************
//*************************************************************************************************************

void State::movePods() {
	// This tracks the time during the turn. The goal is to reach 1.0
	float t = TURN_START_TIME;

	Collision* previousCollision = NULL;

	while (t < TURN_END_TIME) {
		Collision* firstCollision = NULL;

		// We look for all the collisions that are going to occur during the turn
		for (int i = 0; i < podsCount; ++i) {
			// Collision with another pod?
			for (int j = i + 1; j < podsCount; ++j) {
				Collision* col = checkForCollision(pods[i], pods[j]);

				if (col && TURN_START_TIME == col->getCollisinTurnTime() && compareCollisions(previousCollision, col)) {
					col = NULL;
				}

				// If the collision occurs earlier than the one we currently have we keep it
				if (col != NULL && col->getCollisinTurnTime() + t < TURN_END_TIME &&
					(firstCollision == NULL || col->getCollisinTurnTime() < firstCollision->getCollisinTurnTime())) {
					firstCollision = col;
				}
			}

			// Collision with another checkpoint?
			// It is unnecessary to check all checkpoints here. We only test the pod's next checkpoint.
			// We could look for the collisions of the pod with all the checkpoints, but if such a collision happens it wouldn't impact the game in any way
			Collision* col = checkForCollision(pods[i], checkPoints[pods[i]->getNextCheckPointId()]);

			if (col && TURN_START_TIME == col->getCollisinTurnTime() && compareCollisions(previousCollision, col)) {
				col = NULL;
			}

			// If the collision happens earlier than the current one we keep it
			if (col != NULL && col->getCollisinTurnTime() + t < TURN_END_TIME &&
				(firstCollision == NULL || col->getCollisinTurnTime() < firstCollision->getCollisinTurnTime())) {
				firstCollision = col;
			}
		}

		if (firstCollision == NULL) {
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
				pods[i]->move(firstCollision->getCollisinTurnTime());
			}

			CheckPoint* checkPoint = dynamic_cast<CheckPoint*>(firstCollision->getEntityB());
			Pod* pod = dynamic_cast<Pod*>(firstCollision->getEntityA());
			if (pod && checkPoint) {
				computeCheckPointCollision(pod, checkPoint);
			}
			else {
				// Play out the collision
				firstCollision->getEntityA()->computeBounce(firstCollision->getEntityB());
			}

			t += firstCollision->getCollisinTurnTime();
		}

		previousCollision = firstCollision;
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

void State::generatePodsTurnActions() {
	for (int podIdx = 0; podIdx < podsCount; ++podIdx) {
		pods[podIdx]->generateTurnActions();
	}
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

	void setAction(Action action) { this->action = action; }
	void setState(State* state) { this->state = state; }
	void setParent(Node* parent) { this->parent = parent; }
	void setChildrenCount(int childrenCount) { this->childrenCount = childrenCount; }
	void setChildren(Node** children) { this->children = children; }
	void setNodeDepth(int nodeDepth) { this->nodeDepth = nodeDepth; }

	void addChild(Node* newChild);
	Node* createChild(MaximizeMinimize mm, int actionIdx);
	void deleteChildren();
	Node* getChildI(int i);
	void copyState(State* state);

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
	label('%'),
	pathToNode("")
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
	if (state) {
		delete state;
		state = NULL;
	}

	if (children) {
		delete[] children;
		children = NULL;
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

Node* Node::createChild(MaximizeMinimize mm, int actionIdx) {
	Node* child = NULL;
	Pod* pod = NULL;

	if (MM_MAXIMIZE == mm) {
		pod = state->getPod(TSPI_MY_POD_IDX);
	}
	else if (MM_MINIMIZE == mm) {
		pod = state->getPod(TSPI_ENEMY_POD_IDX);
	}

	Action actionForChild = pod->getTurnAction(actionIdx);

	child = new Node(actionForChild, NULL, this, 0, NULL, nodeDepth + 1, 'A' + actionIdx);
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

	Action run(State* state, PodRole role);
	Action backtrack(Node* node) const;
	void deleteTree(Node* node);
	void initTree();

	MinMaxResult maximize(Node* node, PodRole podRole, int alpha, int beta);
	MinMaxResult minimize(Node* node, PodRole podRole, int alpha, int beta);

	int evaluateState(State* state, PodRole podRole) const;
	int evaluateRunnerState(State* state) const;
	int evaluateHunterrState(State* state) const;

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
	deleteTree(tree);

	if (tree) {
		delete tree;
		tree = NULL;
	}
}

//*************************************************************************************************************
//*************************************************************************************************************

Action Minimax::run(State* state, PodRole podRole) {
	tree->copyState(state);
	MinMaxResult minimaxRes = maximize(tree, podRole, INT_MIN, INT_MAX);

	cout << minimaxRes.bestLeaveNode->getPathToNode() << endl;

	return backtrack(minimaxRes.bestLeaveNode);
}

//*************************************************************************************************************
//*************************************************************************************************************

Action Minimax::backtrack(Node* node) const {
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
		if (NULL == node->getChildren()) {
			delete node;
			node = NULL;
		}
		else {
			for (int childIdx = 0; childIdx < node->getChildrenCount(); ++childIdx) {
				deleteTree(node->getChildI(childIdx));
			}
		}
	}
}

//*************************************************************************************************************
//*************************************************************************************************************

void Minimax::initTree() {
	tree = new Node();
}

//*************************************************************************************************************
//*************************************************************************************************************

MinMaxResult Minimax::maximize(Node* node, PodRole podRole, int alpha, int beta) {
	if (node->getNodeDepth() == maxTreeDepth || node->getState()->isTerminal()) {
		//int eval = evaluateState(node->getState(), podRole);
		//MinMaxResult res = MinMaxResult(node, eval);
		//return res;

		int rInt = debugEval(node->getPathToNode());

		MinMaxResult res = MinMaxResult(node, rInt); // For debugging the tree
		return res;
	}

	MinMaxResult res = MinMaxResult(NULL, INT_MIN);

	for (int actionIdx = 0; actionIdx < POD_ACTIONS_COUNT; ++actionIdx) {
		Node* child = node->createChild(MM_MAXIMIZE, actionIdx);
		node->addChild(child);

		MinMaxResult minRes = minimize(child, podRole, alpha, beta);

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

	return res;
}

//*************************************************************************************************************
//*************************************************************************************************************

MinMaxResult Minimax::minimize(Node* node, PodRole podRole, int alpha, int beta) {
	MinMaxResult res = MinMaxResult(NULL, INT_MAX);

	for (int actionIdx = 0; actionIdx < POD_ACTIONS_COUNT; ++actionIdx) {
		Node* child = node->createChild(MM_MINIMIZE, actionIdx);
		node->addChild(child);

		MinMaxResult maxRes = maximize(child, podRole, alpha, beta);

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

	return res;
}

//*************************************************************************************************************
//*************************************************************************************************************

int Minimax::evaluateState(State* state, PodRole podRole) const {
	int evaluation = 0;

	if (PR_MY_RUNNER == podRole) {
		evaluation = evaluateRunnerState(state);
	}
	else if (PR_MY_HUNTER == podRole) {
		evaluation = evaluateHunterrState(state);
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

int Minimax::evaluateHunterrState(State* state) const {
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

int Minimax::debugEval(const string& nodePath) const {
	int rInt = 0;

	if ("%AAAA" == nodePath) { rInt = 41; }
	else if ("%AAAB" == nodePath) { rInt = 67; }
	else if ("%AAAC" == nodePath) { rInt = 34; }
	else if ("%AABA" == nodePath) { rInt = 0; }
	else if ("%AABB" == nodePath) { rInt = 69; }
	else if ("%AABC" == nodePath) { rInt = 24; }
	else if ("%AACA" == nodePath) { rInt = 78; }
	else if ("%AACB" == nodePath) { rInt = 58; }
	else if ("%AACC" == nodePath) { rInt = 62; }
	else if ("%ABAA" == nodePath) { rInt = 64; }
	else if ("%ABAB" == nodePath) { rInt = 5; }
	else if ("%ABAC" == nodePath) { rInt = 45; }
	else if ("%ABBA" == nodePath) { rInt = 81; }
	else if ("%ABBB" == nodePath) { rInt = 27; }
	else if ("%ABBC" == nodePath) { rInt = 61; }
	else if ("%ABCA" == nodePath) { rInt = 91; }
	else if ("%ABCB" == nodePath) { rInt = 95; }
	else if ("%ABCC" == nodePath) { rInt = 42; }
	else if ("%ACAA" == nodePath) { rInt = 27; }
	else if ("%ACAB" == nodePath) { rInt = 36; }
	else if ("%ACAC" == nodePath) { rInt = 91; }
	else if ("%ACBA" == nodePath) { rInt = 4; }
	else if ("%ACBB" == nodePath) { rInt = 2; }
	else if ("%ACBC" == nodePath) { rInt = 53; }
	else if ("%ACCA" == nodePath) { rInt = 92; }
	else if ("%ACCB" == nodePath) { rInt = 82; }
	else if ("%ACCC" == nodePath) { rInt = 21; }
	else if ("%BAAA" == nodePath) { rInt = 16; }
	else if ("%BAAB" == nodePath) { rInt = 18; }
	else if ("%BAAC" == nodePath) { rInt = 95; }
	else if ("%BABA" == nodePath) { rInt = 47; }
	else if ("%BABB" == nodePath) { rInt = 26; }
	else if ("%BABC" == nodePath) { rInt = 71; }
	else if ("%BACA" == nodePath) { rInt = 38; }
	else if ("%BACB" == nodePath) { rInt = 69; }
	else if ("%BACC" == nodePath) { rInt = 12; }
	else if ("%BBAA" == nodePath) { rInt = 67; }
	else if ("%BBAB" == nodePath) { rInt = 99; }
	else if ("%BBAC" == nodePath) { rInt = 35; }
	else if ("%BBBA" == nodePath) { rInt = 94; }
	else if ("%BBBB" == nodePath) { rInt = 3; }
	else if ("%BBBC" == nodePath) { rInt = 11; }
	else if ("%BBCA" == nodePath) { rInt = 22; }
	else if ("%BBCB" == nodePath) { rInt = 33; }
	else if ("%BBCC" == nodePath) { rInt = 73; }
	else if ("%BCAA" == nodePath) { rInt = 64; }
	else if ("%BCAB" == nodePath) { rInt = 41; }
	else if ("%BCAC" == nodePath) { rInt = 11; }
	else if ("%BCBA" == nodePath) { rInt = 53; }
	else if ("%BCBB" == nodePath) { rInt = 68; }
	else if ("%BCBC" == nodePath) { rInt = 47; }
	else if ("%BCCA" == nodePath) { rInt = 44; }
	else if ("%BCCB" == nodePath) { rInt = 62; }
	else if ("%BCCC" == nodePath) { rInt = 57; }
	else if ("%CAAA" == nodePath) { rInt = 37; }
	else if ("%CAAB" == nodePath) { rInt = 59; }
	else if ("%CAAC" == nodePath) { rInt = 23; }
	else if ("%CABA" == nodePath) { rInt = 41; }
	else if ("%CABB" == nodePath) { rInt = 29; }
	else if ("%CABC" == nodePath) { rInt = 78; }
	else if ("%CACA" == nodePath) { rInt = 16; }
	else if ("%CACB" == nodePath) { rInt = 35; }
	else if ("%CACC" == nodePath) { rInt = 90; }
	else if ("%CBAA" == nodePath) { rInt = 42; }
	else if ("%CBAB" == nodePath) { rInt = 88; }
	else if ("%CBAC" == nodePath) { rInt = 6; }
	else if ("%CBBA" == nodePath) { rInt = 40; }
	else if ("%CBBB" == nodePath) { rInt = 42; }
	else if ("%CBBC" == nodePath) { rInt = 64; }
	else if ("%CBCA" == nodePath) { rInt = 48; }
	else if ("%CBCB" == nodePath) { rInt = 46; }
	else if ("%CBCC" == nodePath) { rInt = 5; }
	else if ("%CCAA" == nodePath) { rInt = 90; }
	else if ("%CCAB" == nodePath) { rInt = 29; }
	else if ("%CCAC" == nodePath) { rInt = 70; }
	else if ("%CCBA" == nodePath) { rInt = 50; }
	else if ("%CCBB" == nodePath) { rInt = 6; }
	else if ("%CCBC" == nodePath) { rInt = 1; }
	else if ("%CCCA" == nodePath) { rInt = 93; }
	else if ("%CCCB" == nodePath) { rInt = 48; }
	else if ("%CCCC" == nodePath) { rInt = 29; }

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
}

//*************************************************************************************************************
//*************************************************************************************************************

void Game::gameLoop() {
	while (true) {
		getTurnInput();
		turnBegin();
		makeTurn();
		turnEnd();

		// Profiling
		if (2 == turnsCount) {
			break;
		}
	}
}

//*************************************************************************************************************
//*************************************************************************************************************

void Game::getGameInput() {
	//cin >> lapsCount;
	lapsCount = 3; // Profiling
	//cerr << lapsCount << endl;

	//cin >> checkPointsCount;
	checkPointsCount = 4; // Profiling
	//cerr << checkPointsCount << endl;

	checkPoints = new CheckPoint*[checkPointsCount];

	int checkPointXCoord;
	int checkPointYCoord;
	for (int cpIdx = 0; cpIdx < checkPointsCount; ++cpIdx) {
		if (0 == cpIdx) { checkPointXCoord = 7982; checkPointYCoord = 7873; }
		if (1 == cpIdx) { checkPointXCoord = 13284; checkPointYCoord = 5513; }
		if (2 == cpIdx) { checkPointXCoord = 9539; checkPointYCoord = 1380; }
		if (3 == cpIdx) { checkPointXCoord = 3637; checkPointYCoord = 7873; }
		//cin >> checkPointXCoord;
		//cin >> checkPointYCoord;
		//cerr << checkPointXCoord << " " << checkPointYCoord << endl;

		checkPoints[cpIdx] = new CheckPoint(Coords((float)checkPointXCoord, (float)checkPointYCoord), Coords(), CHECKPOINT_RADIUS, cpIdx);
	}

	turnState->setCheckPointsCount(checkPointsCount);
	turnState->setCheckPoints(checkPoints);
}

//*************************************************************************************************************
//*************************************************************************************************************

void Game::getTurnInput() {
	int podXCoord, podYCoord, podVx, podVy, podAngle, podNextCheckPointId;

	for (int podIdx = 0; podIdx < GAME_PODS_COUNT; ++podIdx) {
		if (FIRST_TURN == turnsCount) {
			if (0 == podIdx) { podXCoord = 7779; podYCoord = 7416; podVx = 0; podVy = 0; podAngle = -1; podNextCheckPointId = 1; }
			if (1 == podIdx) { podXCoord = 8185; podYCoord = 8330; podVx = 0; podVy = 0; podAngle = -1; podNextCheckPointId = 1; }
			if (2 == podIdx) { podXCoord = 7372; podYCoord = 6503; podVx = 0; podVy = 0; podAngle = -1; podNextCheckPointId = 1; }
			if (3 == podIdx) { podXCoord = 8592; podYCoord = 9243; podVx = 0; podVy = 0; podAngle = -1; podNextCheckPointId = 1; }
		}
		else {
			if (0 == podIdx) { podXCoord = 7874; podYCoord = 7383; podVx = 80; podVy = -27; podAngle = 341; podNextCheckPointId = 1; }
			if (1 == podIdx) { podXCoord = 8754; podYCoord = 8016; podVx = 483; podVy = -267; podAngle = 331; podNextCheckPointId = 1; }
			if (2 == podIdx) { podXCoord = 7471; podYCoord = 6486; podVx = 83; podVy = -14; podAngle = 350; podNextCheckPointId = 1; }
			if (3 == podIdx) { podXCoord = 8670; podYCoord = 9181; podVx = 66; podVy = -52; podAngle = 322; podNextCheckPointId = 1; }
		}
		//cin >> podXCoord >> podYCoord >> podVx >> podVy >> podAngle >> podNextCheckPointId;
		//cerr << podXCoord << " " << podYCoord << " " << podVx << " " << podVy << " " << podAngle << " " << podNextCheckPointId << endl;

		Pod** pods = turnState->getPods();
		pods[podIdx]->setPosition(Coords((float)podXCoord, (float)podYCoord));
		pods[podIdx]->setSpeedVector(Coords((float)podVx, (float)podVy));
		pods[podIdx]->setAngle((float)podAngle);
		pods[podIdx]->setNextCheckPointId(podNextCheckPointId);

		if (lastTurnPodsGoalCPs[podIdx] != podNextCheckPointId) {
			pods[podIdx]->incrementPassedCPCounter();
			pods[podIdx]->resetCPCounter();
		}
		else if (FIRST_TURN != turnsCount){
			pods[podIdx]->decreaseTuensLeft();
		}

		PodRole role = PR_MY_HUNTER;
		if (podIdx >= TEAM_PODS_COUNT) {
			role = PR_ENEMY_HUNTER;
		}

		pods[podIdx]->setRole(role);
	}
}

//*************************************************************************************************************
//*************************************************************************************************************

void Game::turnBegin() {
	if (FIRST_TURN != turnsCount) {
		turnState->assignRoles();
		turnState->generatePodsTurnActions();
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
		// MiniMax
		Action runnerAction = chooseAction(myRunnerSubState, PR_MY_RUNNER);
		Action hunterAction = chooseAction(myHunterSubState, PR_MY_HUNTER);

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
	cout << firstCheckPointCoords.xCoord << " " << firstCheckPointCoords.yCoord << " " << MAX_THRUST << endl;
	cout << firstCheckPointCoords.xCoord << " " << firstCheckPointCoords.yCoord << " " << BOOST << endl;
}

//*************************************************************************************************************
//*************************************************************************************************************

Action Game::chooseAction(State* state, PodRole role) {
	minimax = new Minimax(NULL, MINIMAX_DEPTH);
	minimax->initTree();
	return minimax->run(state, role);
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

int main() {
	Game game;
	game.play();

	return 0;
}

// Game simulation parameters
/*
seed = 681000254
pod_per_player = 2
pod_timeout = 100
map = 7982 7873 13284 5513 9539 1380 3637 4405
*/

// Inpit
/*
3
4
7982 7873
13284 5513
9539 1380
3637 4405
7779 7416 0 0 -1 1
8185 8330 0 0 -1 1
7372 6503 0 0 -1 1
8592 9243 0 0 -1 1
7874 7383 80 -27 341 1
8754 8016 483 -267 331 1
7471 6486 83 -14 350 1
8670 9181 66 -52 322 1

*/