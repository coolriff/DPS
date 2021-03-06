#ifndef __DPS_h_
#define __DPS_h_

#include "BaseApplication.h"
#include "DPSHelper.h"
#include "BtOgrePG.h"
#include "BtOgreGP.h"
#include "BtOgreExtras.h"
#include "ExampleApplication.h"

namespace Globals
{
	btDynamicsWorld* phyWorld;
    BtOgre::DebugDrawer* dbgdraw;
}

class DPS : public BaseApplication
{
	public:
		DPS(void);
		~DPS(void);

	protected:
		//Bullet things.
		btAxisSweep3 *mBroadphase;
		btDefaultCollisionConfiguration *mCollisionConfig;
		btCollisionDispatcher *mDispatcher;
		btSequentialImpulseConstraintSolver *mSolver;

		//DPSHelper* dpsHelper;
		std::shared_ptr<DPSHelper> dpsHelper;

		virtual void createScene(void);
		virtual void createFrameListenerBtOgre(void);
		void setColor(Ogre::Entity* ent ,Ogre::Vector3 v);
		void throwSphere(void);
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