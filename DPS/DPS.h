#ifndef __DPS_h_
#define __DPS_h_

#include "BaseApplication.h"
#include "DPSHelper.h"
#include "DPSSoftBodyHelper.h"
#include "BtOgrePG.h"
#include "BtOgreGP.h"
#include "BtOgreExtras.h"
#include <btBulletDynamicsCommon.h>
#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>
#include <BulletSoftBody/btDefaultSoftBodySolver.h>
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h>
#include "Leap.h"
#include "LeapListener.h"

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

		DPS(void);
		~DPS(void);

		// No CCD (continuous collision detection) for Gimpact shapes
		// base on Bullet engine forum Example
		struct MyClosestRayResultCallback : public btCollisionWorld::ClosestRayResultCallback
		{
			const btCollisionShape * m_hitTriangleShape;
			int					  m_hitTriangleIndex;
			int					  m_hitShapePart;

			MyClosestRayResultCallback (const btVector3 & rayFrom,const btVector3 & rayTo)
				: btCollisionWorld::ClosestRayResultCallback(rayFrom, rayTo),
				m_hitTriangleShape(NULL),
				m_hitTriangleIndex(0),
				m_hitShapePart(0)
			{
			}

			virtual ~MyClosestRayResultCallback()
			{
			}

			virtual btScalar addSingleResult(btCollisionWorld::LocalRayResult & rayResult, bool normalInWorldSpace)
			{
				if (rayResult.m_localShapeInfo)
				{
					m_hitTriangleShape = rayResult.m_collisionObject->getCollisionShape();
					m_hitTriangleIndex = rayResult.m_localShapeInfo->m_triangleIndex;
					m_hitShapePart = rayResult.m_localShapeInfo->m_shapePart;
				} else 
				{
					m_hitTriangleShape = NULL;
					m_hitTriangleIndex = 0;
					m_hitShapePart = 0;
				}
				return ClosestRayResultCallback::addSingleResult(rayResult,normalInWorldSpace);
			}
		};

		btAlignedObjectArray<btSoftSoftCollisionAlgorithm*> m_SoftSoftCollisionAlgorithms;
		btAlignedObjectArray<btSoftRididCollisionAlgorithm*> m_SoftRigidCollisionAlgorithms;
		btSoftBodyWorldInfo	m_softBodyWorldInfo;

		//keep the collision shapes, for deletion/cleanup
		btAlignedObjectArray<btCollisionShape*> m_collisionShapes;
		btBroadphaseInterface*	m_broadphase;
		btCollisionDispatcher*	m_dispatcher;
		btConstraintSolver*	m_solver;
		btCollisionAlgorithmCreateFunc*	m_boxBoxCF;
		btDefaultCollisionConfiguration* m_collisionConfiguration;

		void initPhysics(void);
		void exitPhysics(void);
		void resetCamera(void);
		void resetCamera(Ogre::Vector3 camPos);
		void deleteOgreEntities(void);
		void deletePhysicsShapes(void);
		void GimpactRayCallBack(void);
		bool process_triangle(btCollisionShape * shape, int hitTriangleIndex);
		void GUIeventHandler(void);
		void setMiniCamPosition(Ogre::Vector3 camPos);
		std::string convertInt(int number);

		void bulletDebugScreen(void);
		void solidScreen(void);
		void demoController(void);
		void clearScreen(void);

		Ogre::FrameEvent evt;
		double dt;

		int leapMotionCounter;
		std::string fingerName_0;
		std::string fingerName_1;

		LeapListener leapMotionListener;
		Leap::Controller leapMotionController;
		bool leapMotionRunning;

		Ogre::MovableObject* rayObject;
		Ogre::SceneNode* rayNode;

		bool leapMotionInit(void);
		void leapMotionUpdate(void);
		void leapMotionCleanup(void);

		bool runClothDome_1;
		bool runClothDome_2;
		bool runClothDome_3;
		bool runClothDome_4;
		bool runClothDome_5;
		bool runClothDome_6;
		bool runClothDome_7;

		bool runDeformDome_1;
		bool runDeformDome_2;
		bool runDeformDome_3;
		bool runDeformDome_4;
		bool runDeformDome_5;
		bool runDeformDome_6;
		bool runDeformDome_7;

		bool runPlaygroud_1;
		bool runPlaygroud_2;
		bool runPlaygroud_3;
		bool runPlaygroud_4;
		bool runPlaygroud_5;

	protected:

		std::shared_ptr<DPSHelper> dpsHelper;
		std::shared_ptr<DPSSoftBodyHelper> dpsSoftbodyHelper;

		virtual void createScene(void);
		virtual bool frameRenderingQueued(const Ogre::FrameEvent& evt);

		// OIS::MouseListener
		virtual bool mouseMoved(const OIS::MouseEvent &arg);
		virtual bool mousePressed(const OIS::MouseEvent &arg, OIS::MouseButtonID id);
		virtual bool mouseReleased(const OIS::MouseEvent &arg, OIS::MouseButtonID id);

		bool keyPressed(const OIS::KeyEvent &arg);
};



#endif