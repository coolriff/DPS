#ifndef __DPS_h_
#define __DPS_h_

#include <Terrain/OgreTerrain.h>
#include <Terrain/OgreTerrainGroup.h>
#include "BaseApplication.h"
#include "DPSHelper.h"
#include "DPSSoftBodyHelper.h"
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
class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;

///collisions between two btSoftBody's
class btSoftSoftCollisionAlgorithm;

///collisions between a btSoftBody and a btRigidBody
class btSoftRididCollisionAlgorithm;
class btSoftRigidDynamicsWorld;


namespace Globals
{
	DPS* app;
	btSoftRigidDynamicsWorld* phyWorld;
    BtOgre::DebugDrawer* dbgdraw;
}

class DPS : public BaseApplication
{
	public:

		void initPhysics();
		void exitPhysics();

		DPS(void) :
			mTerrainGlobals(0),
			mTerrainGroup(0),
			mTerrainsImported(false),
			mInfoLabel(0),
			GRASS_WIDTH(40),
			GRASS_HEIGHT(40)
		{
			initPhysics();
		}

		~DPS(void)
		{
			exitPhysics();
		}

		btAlignedObjectArray<btSoftSoftCollisionAlgorithm*> m_SoftSoftCollisionAlgorithms;
		btAlignedObjectArray<btSoftRididCollisionAlgorithm*> m_SoftRigidCollisionAlgorithms;
		btSoftBodyWorldInfo	m_softBodyWorldInfo;
		//keep the collision shapes, for deletion/cleanup
		btAlignedObjectArray<btCollisionShape*>		m_collisionShapes;
		btBroadphaseInterface*	m_broadphase;
		btCollisionDispatcher*	m_dispatcher;
		btConstraintSolver*	m_solver;
		btCollisionAlgorithmCreateFunc*	m_boxBoxCF;
		btDefaultCollisionConfiguration* m_collisionConfiguration;

		ParticleSystem* fireOnCube_1;
		ParticleSystem* fireOnCube_2;
		ParticleSystem* ps;
		ParticleSystem* ps2;
		ParticleSystem* ps3;

		Ogre::TerrainGlobalOptions* mTerrainGlobals;
		Ogre::TerrainGroup* mTerrainGroup;
		bool mTerrainsImported;
		OgreBites::Label* mInfoLabel;

		const Real GRASS_WIDTH;
		const Real GRASS_HEIGHT;
		StaticGeometry* mField;
		AnimationState* mLightAnimState;
		Controller<Real>* mLightController;

		void defineTerrain(long x, long y);
		void initBlendMaps(Ogre::Terrain* terrain);
		void configureTerrainDefaults(Ogre::Light* light);
		void getTerrainImage(bool flipX, bool flipY, Ogre::Image& img);

		void waveGrass(Real timeElapsed);
		void createGrassMesh();

		struct GrassVertex
		{
			float x, y, z;
			float nx, ny, nz;
			float u, v;
		};

	protected:

		Ogre::ManualObject* m_ManualObject;

		std::shared_ptr<DPSHelper> dpsHelper;
		std::shared_ptr<DPSSoftBodyHelper> dpsSoftbodyHelper;

		virtual void createScene(void);
		virtual void createFrameListener(void);
		virtual void destroyScene(void);
		virtual bool frameRenderingQueued(const Ogre::FrameEvent& evt);

		void initSoftBody(btSoftBody* body);
		void updateSoftBody(btSoftBody* body);
		bool keyPressed(const OIS::KeyEvent &arg);
};



#endif