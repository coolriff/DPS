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

	protected:

		btDispatcher* dispatcher;
		btCollisionConfiguration* collisionConfig;
		btBroadphaseInterface* broadphase;
		btConstraintSolver* solver;
		btSoftBodySolver* softbodySolver;

		Ogre::SceneNode* mClothNode;

		btSoftBodyWorldInfo* m_SoftBodyWorldInfo;
		btSoftBody* m_SoftBody;
		btSoftBody* m_cloth;

		//manual object replaces the liquid body's entity
		//Ogre::Entity* m_BlobEntity;
		Ogre::ManualObject* m_ManualObject;

		//DPSHelper* dpsHelper;
		std::shared_ptr<DPSHelper> dpsHelper;

		virtual void createScene(void);
		
		virtual bool frameRenderingQueued(const Ogre::FrameEvent& evt);

		void setColor(Ogre::Entity* ent ,Ogre::Vector3 v);
		btSoftBody* createSoftBody(const btVector3& startPos);
		btSoftBody* createCloth(void);
		void initSoftBody(btSoftBody* body);
		void updateSoftBody(btSoftBody* body);
		bool keyPressed(const OIS::KeyEvent &arg);
};



#endif