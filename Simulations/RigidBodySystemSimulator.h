#ifndef RIGIDBODYSYSTEMSIMULATOR_h
#define RIGIDBODYSYSTEMSIMULATOR_h
#include "Simulator.h"
#include "util/timer.h"
//add your header for your rigid body system, for e.g.,
//#include "rigidBodySystem.h" 

#define TESTCASEUSEDTORUNTEST 2

struct Force
{
	Vec3 Position;
	Vec3 Direction;
};


struct RigidBox
{
	RigidBox() {}
	RigidBox(const Vec3& pos, const Vec3& size, const Quat& orient, const Vec3& vel, const Vec3& ang, float mass);

	Vec3 Position;
	Vec3 Size;
	Quat Orientation;

	Vec3 Velocity;
	Vec3 AngularVelocity;

	float Mass;
	float Restitution = 0.5f;

	Mat4 Iref;

	// Local position
	std::vector<Vec3> RigidPoints;
	// World 
	std::vector<Force> ExternalForces;
	std::vector<Force> CollisionForces;

	bool EnableGravity = false;
	bool IsOnGround = false;
	bool IsGround = false;
};

struct RigidBodySystem
{
	void InitSystemWithOneBox();
	void InitSystemWithTwoBoxes();
	void InitSystemWithFourBoxes();

	std::vector<RigidBox> Boxes;
};

class RigidBodySystemSimulator :public Simulator {
public:
	// Construtors
	RigidBodySystemSimulator();

	// Functions
	const char* getTestCasesStr();
	void initUI(DrawingUtilitiesClass* DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// ExtraFunctions
	int getNumberOfRigidBodies();
	Vec3 getPositionOfRigidBody(int i);
	Vec3 getLinearVelocityOfRigidBody(int i);
	Vec3 getAngularVelocityOfRigidBody(int i);
	void applyForceOnBody(int i, Vec3 loc, Vec3 force);
	void addRigidBody(Vec3 position, Vec3 size, int mass);
	void setOrientationOf(int i, Quat orientation);
	void setVelocityOf(int i, Vec3 velocity);

	void DrawRigidBodySystem();
	void SimulateRigidMotion(float ts);
	void UpdateCollisionForce(float ts);

private:
	// Attributes
	// add your RigidBodySystem data members, for e.g.,
	// RigidBodySystem * m_pRigidBodySystem; 
	RigidBodySystem* m_pRigidBodySystem;
	Vec3 m_externalForce;

	// UI Attributes
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;

	MuTime myTimer;
	float m_accumFrameTime = 0.0f;
};
#endif