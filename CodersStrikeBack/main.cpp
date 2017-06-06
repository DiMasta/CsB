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

const float INVALID_COORD = -1.f;
const float MAX_ANGLE_PER_TURN = 18.f;
const float DEFAULT_ANGLE = -1.f;
const float FRICTION = .85f;
const float HALF_MOMENTUM = 120.f;
const float MASS_WITH_SHEILD = 10.f;
const float MASS_WITHOUT_SHEILD = 1.f;
const float TURN_START_TIME = 0.f;
const float TURN_END_TIME = 1.f;
const float MAX_DIST = 99999.f;

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

const string SHEILD = "SHEILD";
const string BOOST = "BOOST";

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

struct SimulationParams {
	SimulationParams();

	Coords targets[GAME_PODS_COUNT];
	int podsThrusts[GAME_PODS_COUNT];
	bool sheilds[GAME_PODS_COUNT];
};

//*************************************************************************************************************
//*************************************************************************************************************

SimulationParams::SimulationParams() {
	for (int i = 0; i < GAME_PODS_COUNT; ++i) {
		targets[i] = Coords();
		podsThrusts[i] = 0;
		sheilds[i] = false;
	}
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
		PodRole role
	);
	~Pod();

	void setAngle(float angle) { this->angle = angle; }
	void setNextCheckPointId(int nextCheckPointId) { this->nextCheckPointId = nextCheckPointId; }
	void setTurnsLeft(int turnsLeft) { this->turnsLeft = turnsLeft; }
	void setRole(PodRole role) { this->role = role; }

	int getNextCheckPointId() const { return nextCheckPointId; }
	int getSheildUp() const { return shieldUp; }
	PodRole getRole() const { return role; }

	float calcAngleToTarget(Coords target) const;
	float calcDircetionToTurn(Coords target) const;
	void rotate(Coords target, int turnsCount);
	void applyThrust(int thrust);
	void move(float time);
	void end();
	float truncate(float toTruncate);
	void resetCPCounter();
	void activateSheild();
	void manageSheild();

	void computeBounce(Entity* entity) override;
	bool sheildOn() const override;
	void debug() const override;

private:
	float angle;
	int nextCheckPointId;
	int turnsLeft;
	string belongs;
	bool shieldUp;
	int sheildTurnsLeft;
	PodRole role;
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
	role(PR_INVALID)
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
	PodRole role
) :
	Entity(position, speedVector, radius),
	angle(angle),
	nextCheckPointId(nextCheckPointId),
	turnsLeft(turnsLeft),
	shieldUp(shieldUp),
	sheildTurnsLeft(sheildTurnsLeft),
	role(role)
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

void Pod::rotate(Coords target, int turnsCount) {
	float angleToTurn = calcDircetionToTurn(target);

	if (turnsCount > 0) {
		// Can't turn by more than 18 in one turn
		if (angleToTurn > MAX_ANGLE_PER_TURN) {
			angleToTurn = MAX_ANGLE_PER_TURN;
		}
		else if (angleToTurn < -MAX_ANGLE_PER_TURN) {
			angleToTurn = -MAX_ANGLE_PER_TURN;
		}
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
	if (dynamic_cast<CheckPoint*>(entity) != NULL) {
		// Collision with a checkpoint
		resetCPCounter();
	}
	else {
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

bool Pod::sheildOn() const {
	return shieldUp;
}

//*************************************************************************************************************
//*************************************************************************************************************

void Pod::debug() const {
	cerr << belongs << ":" << " ";
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
	State(int checkPointsCount, int podsCount);
	State(State* state);
	State(State* state, PodRole role);
	State(int podsCount, Pod** pods, int checkPointsCount, CheckPoint** checkPoints);
	~State();

	Pod** getPods() const { return pods; }
	int getCheckPointsCount() const { return checkPointsCount; }
	CheckPoint** getCheckPoints() const { return checkPoints; }
	Pod* getPod(int id) const { return pods[id]; }
	CheckPoint* getCheckPoint(int id) const { return checkPoints[id]; }
	int getPodsCount() const { return podsCount; }

	void initState(int checkPointsCount, int podsCount);
	void setCheckPointData(Coords postion, int id);
	void simulateTurn(const SimulationParams& simulationParams, int turnsCount);
	Collision* checkForCollision(Entity* entityA, Entity* entityB) const;
	void movePods();
	bool compareCollisions(Collision* collisionA, Collision* collisoionB) const;
	void assignRoles();
	Pod* getPodByRole(PodRole role) const;
	Pod* getClosestToCPHunter(PodRole role) const;
	int getRolePodIdx(PodRole role) const;

	void debug() const;
	void debugCheckPoints() const;

private:
	int podsCount;
	Pod** pods;

	int checkPointsCount;
	CheckPoint** checkPoints;
};

//*************************************************************************************************************
//*************************************************************************************************************

State::State() : podsCount(0), pods(NULL), checkPointsCount(0), checkPoints(NULL) {

}

//*************************************************************************************************************
//*************************************************************************************************************

State::State(int checkPointsCount, int podsCount) :
	checkPointsCount(checkPointsCount),
	podsCount(podsCount)
{
	initState(checkPointsCount, podsCount);
}

//*************************************************************************************************************
//*************************************************************************************************************

State::State(State* state) {
	int sourceCheckPointsCount = state->getCheckPointsCount();
	int sourcePodsCount = state->getPodsCount();
	initState(sourceCheckPointsCount, sourcePodsCount);

	for (int podIdx = 0; podIdx < sourcePodsCount; ++podIdx) {
		memcpy(this->pods[podIdx], state->pods[podIdx], sizeof(Pod));
	}

	for (int cpIdx = 0; cpIdx < sourceCheckPointsCount; ++cpIdx) {
		memcpy(this->checkPoints[cpIdx], state->checkPoints[cpIdx], sizeof(CheckPoint));
	}

	this->checkPointsCount = sourceCheckPointsCount;
	this->podsCount = podsCount;
}

//*************************************************************************************************************
//*************************************************************************************************************

State::State(State* state, PodRole role) {
	int sourceCheckPointsCount = state->getCheckPointsCount();
	int sourcePodsCount = state->getPodsCount();

	initState(sourceCheckPointsCount, SUBSTATE_PODS_COUNT);

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

	this->checkPointsCount = sourceCheckPointsCount;
	this->podsCount = SUBSTATE_PODS_COUNT;
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

void State::initState(int checkPointsCount, int podsCount) {
	pods = new Pod*[podsCount];
	for (int podIdx = 0; podIdx < podsCount; ++podIdx) {
		pods[podIdx] = new Pod();
	}

	checkPoints = new CheckPoint*[checkPointsCount];
	for (int cpIdx = 0; cpIdx < checkPointsCount; ++cpIdx) {
		checkPoints[cpIdx] = new CheckPoint();
	}
}

//*************************************************************************************************************
//*************************************************************************************************************

void State::setCheckPointData(Coords postion, int id) {
	checkPoints[id]->setPosition(postion);
	checkPoints[id]->setId(id);
	checkPoints[id]->setSpeedVector(Coords(0.f, 0.f));
}

//*************************************************************************************************************
//*************************************************************************************************************

void State::simulateTurn(const SimulationParams& simulationParams, int turnsCount) {
	for (int podIdx = 0; podIdx < podsCount; ++podIdx) {
		pods[podIdx]->rotate(simulationParams.targets[podIdx], turnsCount);

		if (simulationParams.sheilds[podIdx]) {
			pods[podIdx]->activateSheild();
		}

		int thrustToApply = simulationParams.podsThrusts[podIdx];
		if (pods[podIdx]->getSheildUp()) {
			thrustToApply = 0;
		}

		pods[podIdx]->applyThrust(thrustToApply);
	}

	movePods();

	for (int podIdx = 0; podIdx < GAME_PODS_COUNT; ++podIdx) {
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

			// Play out the collision
			firstCollision->getEntityA()->computeBounce(firstCollision->getEntityB());

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

	return INVALID_ID;
}

//*************************************************************************************************************
//*************************************************************************************************************

void State::assignRoles() {
	Pod* myRunner = getClosestToCPHunter(PR_MY_HUNTER);
	Pod* enemyRunner = getClosestToCPHunter(PR_ENEMY_HUNTER);

	myRunner->setRole(PR_MY_RUNNER);
	enemyRunner->setRole(PR_ENEMY_HUNTER);
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

void State::debug() const {
	for (int podIdx = 0; podIdx < podsCount; ++podIdx) {
		pods[podIdx]->debug();
	}
}

//*************************************************************************************************************
//*************************************************************************************************************

void State::debugCheckPoints() const {
	for (int checkPointIdx = 0; checkPointIdx < checkPointsCount; ++checkPointIdx) {
		checkPoints[checkPointIdx]->debug();
	}
}

//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------

class Action {
public:
	Action();
	~Action();

	void printAction() const;
private:
	Coords coords;
	bool useSheild;
	int thrust;
};

//*************************************************************************************************************
//*************************************************************************************************************

Action::Action() : coords(), useSheild(false), thrust(0) {

}

//*************************************************************************************************************
//*************************************************************************************************************

Action::~Action() {

}

//*************************************************************************************************************
//*************************************************************************************************************

void Action::printAction() const {
	cout << coords.xCoord << " " << coords.yCoord << " ";

	if (useSheild) {
		cout << SHEILD;
	}
	else {
		cout << thrust;
	}

	cout << endl;
}

//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------

class Node {
public:
	Node();
	Node(Action* action, State* state, Node* parent);
	~Node();
private:
	// Action to get to node
	Action* action;

	// Game state for two pods, NULL if my turn
	State* state;

	Node* parent;
};

//*************************************************************************************************************
//*************************************************************************************************************

Node::Node() {
}

//*************************************************************************************************************
//*************************************************************************************************************

Node::Node(Action * action, State * state, Node * parent) {
}

//*************************************************************************************************************
//*************************************************************************************************************

Node::~Node() {
}

//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------

class Minimax {
public:
	Minimax();
	~Minimax();

	Node* run(State* state);

	Node* maximize(Node* node);
	Node* minimize(Node* node);

	// Evaluation functions

private:
};

//*************************************************************************************************************
//*************************************************************************************************************

Minimax::Minimax() {
}

//*************************************************************************************************************
//*************************************************************************************************************

Minimax::~Minimax() {
}

//*************************************************************************************************************
//*************************************************************************************************************

Node * Minimax::run(State * state) {
	return nullptr;
}

//*************************************************************************************************************
//*************************************************************************************************************

Node * Minimax::maximize(Node * node) {
	return nullptr;
}

//*************************************************************************************************************
//*************************************************************************************************************

Node * Minimax::minimize(Node * node) {
	return nullptr;
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
	void chooseMove(State* state);
	void makeSubStates();
	void clearSubStates();

	void debug() const;

private:
	int turnsCount;
	int lapsCount;
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
	turnState(NULL),
	myRunnerSubState(NULL),
	myHunterSubState(NULL),
	minimax(NULL)
{
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
}

//*************************************************************************************************************
//*************************************************************************************************************

void Game::initGame() {
}

//*************************************************************************************************************
//*************************************************************************************************************

void Game::gameLoop() {
	while (true) {
		getTurnInput();
		turnBegin();
		makeTurn();
		turnEnd();
	}
}

//*************************************************************************************************************
//*************************************************************************************************************

void Game::getGameInput() {
	cin >> lapsCount;

	int checkPointsCount;
	cin >> checkPointsCount;

	turnState = new State(checkPointsCount, GAME_PODS_COUNT);

	int checkPointXCoord;
	int checkPointYCoord;
	for (int cpIdx = 0; cpIdx < checkPointsCount; ++cpIdx) {
		cin >> checkPointXCoord;
		cin >> checkPointYCoord;

		turnState->setCheckPointData(Coords((float)checkPointXCoord, (float)checkPointYCoord), cpIdx);
	}
}

//*************************************************************************************************************
//*************************************************************************************************************

void Game::getTurnInput() {
	int podXCoord;
	int podYCoord;

	int podVx;
	int podVy;

	int podAngle;

	int podNextCheckPointId;

	for (int podIdx = 0; podIdx < GAME_PODS_COUNT; ++podIdx) {
		cin >> podXCoord >> podYCoord >> podVx >> podVy >> podAngle >> podNextCheckPointId;

		Pod** pods = turnState->getPods();
		pods[podIdx]->setPosition(Coords((float)podXCoord, (float)podYCoord));
		pods[podIdx]->setSpeedVector(Coords((float)podVx, (float)podVy));
		pods[podIdx]->setAngle((float)podAngle);
		pods[podIdx]->setNextCheckPointId(podNextCheckPointId);

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
	turnState->assignRoles();
	makeSubStates();
}

//*************************************************************************************************************
//*************************************************************************************************************

void Game::makeTurn() {
	if (FIRST_TURN == turnsCount) {
		makeFirstTurn();
	}
	else {
		// MiniMax
		chooseMove(myRunnerSubState);
		chooseMove(myHunterSubState);
	}
}

//*************************************************************************************************************
//*************************************************************************************************************

void Game::turnEnd() {
	++turnsCount;
	clearSubStates();
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
	CheckPoint* firstCheckPoint = turnState->getCheckPoint(1);
	Coords firstCheckPointCoords = firstCheckPoint->getPosition();
	cout << firstCheckPointCoords.xCoord << " " << firstCheckPointCoords.yCoord << " " << MAX_THRUST << endl;
	cout << firstCheckPointCoords.xCoord << " " << firstCheckPointCoords.yCoord << " " << BOOST << endl;
}

//*************************************************************************************************************
//*************************************************************************************************************

void Game::chooseMove(State* state) {
	minimax = new Minimax();
	minimax->run(state);
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

//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------

// Test code Sync1
int main() {
	Game game;
	game.play();

	// seed = 681000254
	// pod_per_player = 2
	// pod_timeout = 100
	// map = 7982 7873 13284 5513 9539 1380 3637 4405

	return 0;
}