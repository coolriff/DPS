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
	btSoftBody* createMesh(void);
	btSoftBody* createSoftBody(const btVector3& startPos);
	void createCloth(void);
	btSoftBody* createBunny(void);

	void initSoftBody(Ogre::ManualObject* m_ManualObject, btSoftBody* body);
	void updateSoftBody(Ogre::ManualObject* m_ManualObject, btSoftBody* body);

	btSoftBody* m_deformableModel;
	btSoftBody* m_mesh;
	btSoftBody* m_SoftBody;
	btSoftBody* m_cloth;
	btSoftBody* m_bunny;

	Ogre::ManualObject* m_clothManualObject;
	/*Ogre::ManualObject* m_ManualObject;*/

// 	struct RenderBufferVertexElement
// 	{
// 		std::vector<float> triangles;
// 		std::vector<int> indicies;
// 		std::vector<float> texture;
// 	};

	btSoftRigidDynamicsWorld* phyWorld;
	Ogre::Camera* mCamera;
	Ogre::SceneManager* mSceneMgr;

	std::vector<float> triangles;
	std::vector<int> indicies;
	std::vector<float> texCoord;

};

#endif