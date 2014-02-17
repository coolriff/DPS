
#ifndef __DPS_h_
#define __DPS_h_

#include "BaseApplication.h"
#include "btBulletDynamicsCommon.h"
#include "ExampleApplication.h"

class DPS : public BaseApplication, public ExampleFrameListener
{
	public:
		DPS(void);
		~DPS(void);

	protected:

		//btDynamicsWorld * phyWorld;
		btAxisSweep3 *mBroadphase;
		btDefaultCollisionConfiguration *mCollisionConfig;
		btCollisionDispatcher *mDispatcher;
		btSequentialImpulseConstraintSolver *mSolver;

		Ogre::SceneNode * mNinjaNode;
		Ogre::Entity * mNinjaEntity;
		btRigidBody * mNinjaBody;
		btCollisionShape * mNinjaShape;

		Ogre::Entity * mGroundEntity;
		btRigidBody * mGroundBody;
		btBvhTriangleMeshShape * mGroundShape;

		virtual void createScene(void);
		void createFrameListener(void);
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

#endif // #ifndef __DPS_h_
