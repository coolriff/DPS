#include "ExampleApplication.h"
#include "BtOgrePG.h"
#include "BtOgreGP.h"
#include "BtOgreExtras.h"

namespace Globals
{
    btDynamicsWorld *phyWorld;
    BtOgre::DebugDrawer *dbgdraw;
}

class DPS : public ExampleApplication
{
	public:
		DPS(void);
		~DPS(void);

    protected:
		btAxisSweep3 *mBroadphase;
		btDefaultCollisionConfiguration *mCollisionConfig;
		btCollisionDispatcher *mDispatcher;
		btSequentialImpulseConstraintSolver *mSolver;

		Ogre::SceneNode *mNinjaNode;
		Ogre::Entity *mNinjaEntity;
		btRigidBody *mNinjaBody;
		btCollisionShape *mNinjaShape;

		Ogre::Entity *mGroundEntity;
		btRigidBody *mGroundBody;
		btBvhTriangleMeshShape *mGroundShape;

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