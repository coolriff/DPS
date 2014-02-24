#ifndef __DPS_h_
#define __DPS_h_

#include "BaseApplication.h"
#include "DPSHelper.h"
#include "BtOgrePG.h"
#include "BtOgreGP.h"
#include "BtOgreExtras.h"
#include "ExampleApplication.h"
#include <btBulletDynamicsCommon.h>
#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>
#include <BulletSoftBody/btDefaultSoftBodySolver.h>
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h>


class DPS;

namespace Globals
{
	DPS* app;
	btSoftRigidDynamicsWorld* phyWorld;
    BtOgre::DebugDrawer* dbgdraw;
}

class DPS : public BaseApplication
{
	public:
		DPS(void);
		~DPS(void);

	void updateLiquidBody(void);

	protected:
		//Bullet things.
		//btAxisSweep3 *mBroadphase;
		//btDefaultCollisionConfiguration *mCollisionConfig;
		//btCollisionDispatcher *mDispatcher;
		//btDefaultSoftBodySolver* softSolver;
		//btSequentialImpulseConstraintSolver *mSolver;

		//btSoftRigidDynamicsWorld* world;
		btDispatcher* dispatcher;
		btCollisionConfiguration* collisionConfig;
		btBroadphaseInterface* broadphase;
		btConstraintSolver* solver;
		btSoftBodySolver* softbodySolver;

		Ogre::SceneNode* mClothNode;

		btSoftBodyWorldInfo* m_SoftBodyWorldInfo;
		btSoftBody* m_LiquidBody;


		//manual object replaces the liquid body's entity
		//Ogre::Entity* m_BlobEntity;
		Ogre::ManualObject* m_ManualObject;

		//DPSHelper* dpsHelper;
		std::shared_ptr<DPSHelper> dpsHelper;

		virtual void createScene(void);
		virtual void createFrameListenerBtOgre(void);
		void setColor(Ogre::Entity* ent ,Ogre::Vector3 v);
		void throwSphere(void);
		void createLiquidBody(const btVector3& startPos);
		void initLiquidBody(void);
		//static void updateLiquidBody(void);
		//void createGround(void);
		//void createBoxShape(float width, float height, float depth, Ogre::Entity* entity, Ogre::Vector3 position, bool bStatic);
		bool keyPressed(const OIS::KeyEvent &arg);
		//void createCamera(void);
		//void createViewports(void);
		//void destroyScene(void);
		//void createPhysicalGround(float size);
		//void createBase(void);
		//void createLight(void);
		//void resetCamera(void);
		//void setFlymode(const Ogre::FrameEvent& evt, bool flag);
		//virtual bool frameRenderingQueued(const Ogre::FrameEvent& evt);
		//virtual bool gameLoop(const Ogre::FrameEvent& evt);
};



#endif