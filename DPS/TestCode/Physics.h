#pragma once

#include <map>
#include<vector>
#include <btBulletDynamicsCommon.h>

class Physics : public BaseApplication
{

	btBroadphaseInterface* overlappingPairCache;

	// Set up the collision configuration and dispatcher
	btDefaultCollisionConfiguration* collisionConfiguration;
	btCollisionDispatcher* dispatcher;

	// The actual physics solver
	btSequentialImpulseConstraintSolver* solver;

	// The world.
    btDiscreteDynamicsWorld* dynamicsWorld;

	std::vector<btCollisionShape *> collisionShapes;
	std::map<std::string, btRigidBody *> physicsAccessors;


	public:
	Physics(void);
	~Physics(void);
	void initObjects();

	//<<irrelevant code here...>
};