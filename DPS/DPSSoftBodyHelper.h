#ifndef __DPSSOFTBODYHELPER_h_
#define __DPSSOFTBODYHELPER_h_

#include "BaseApplication.h"
#include <btBulletDynamicsCommon.h>
#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>
#include <BulletSoftBody/btDefaultSoftBodySolver.h>
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h>

class DPSSoftBodyHelper
{
public:
	DPSSoftBodyHelper(btSoftRigidDynamicsWorld* phyWorld, Ogre::Camera* mCamera, Ogre::SceneManager* mSceneMgr);
	~DPSSoftBodyHelper(void);

	btSoftBody* createDeformableModel(void);
	btSoftBody* createSoftBody(const btVector3& startPos);
	btSoftBody* createCloth(void);

	btSoftBody* m_SoftBody;
	btSoftBody* m_cloth;
	btSoftBody* m_deformableModel;
	
	btSoftRigidDynamicsWorld* phyWorld;
	Ogre::Camera* mCamera;
	Ogre::SceneManager* mSceneMgr;
};

#endif