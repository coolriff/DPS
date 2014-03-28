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


	btSoftBody* createBunny(void);

	btScalar DPSSoftBodyHelper::UnitRand();
	btScalar DPSSoftBodyHelper::SignedUnitRand();
	btVector3 DPSSoftBodyHelper::Vector3Rand();

	void initSoftBody(Ogre::ManualObject*& m_ManualObject, btSoftBody* body);
	void updateSoftBody(Ogre::ManualObject*& m_ManualObject, btSoftBody* body);


	//cloth demos
	Ogre::ManualObject* m_clothManualObject_1;
	btSoftBody* m_clothBody_1;
	void createClothDemo_1(void);

	Ogre::ManualObject* m_clothManualObject_2;
	btSoftBody* m_clothBody_2;
	void createClothDemo_2(void);

	Ogre::ManualObject* m_clothManualObject_3;
	btSoftBody* m_clothBody_3;
	void createClothDemo_3(void);

	Ogre::ManualObject* m_clothManualObject_4;
	btSoftBody* m_clothBody_4;
	void createClothDemo_4(btRigidBody* body);

	Ogre::ManualObject* m_clothManualObject_5_0;
	btSoftBody* m_clothBody_5_0;
	Ogre::ManualObject* m_clothManualObject_5_1;
	btSoftBody* m_clothBody_5_1;
	void createClothDemo_5(void);

	Ogre::ManualObject* m_clothManualObject_6_0;
	btSoftBody* m_clothBody_6_0;
	Ogre::ManualObject* m_clothManualObject_6_1;
	btSoftBody* m_clothBody_6_1;
	Ogre::ManualObject* m_clothManualObject_6_2;
	btSoftBody* m_clothBody_6_2;
	void createClothDemo_6(void);

	Ogre::ManualObject* m_clothManualObject_7_0;
	btSoftBody* m_clothBody_7_0;
	Ogre::ManualObject* m_clothManualObject_7_1;
	btSoftBody* m_clothBody_7_1;
	Ogre::ManualObject* m_clothManualObject_7_2;
	btSoftBody* m_clothBody_7_2;
	void createClothDemo_7(void);


	//softbody demos
	Ogre::ManualObject* m_softManualObject_1;
	btSoftBody* m_softBody_1;
	void createSoftDemo_1(const btVector3& startPos);

	Ogre::ManualObject* m_softManualObject_2;
	btSoftBody* m_softBody_2;
	void createSoftDemo_2(const btVector3& startPos);

	Ogre::ManualObject* m_softManualObject_3;
	btSoftBody* m_softBody_3;
	void createSoftDemo_3(const btVector3& startPos);



	//deformation demos
	Ogre::ManualObject* m_deformManualObject_1;
	btSoftBody* m_deformBody_1;
	void createDeformDemo_1(const btVector3& startPos);

	Ogre::ManualObject* m_deformManualObject_2;
	btSoftBody* m_deformBody_2;
	void createDeformDemo_2(const btVector3& startPos);

	Ogre::ManualObject* m_deformManualObject_3;
	btSoftBody* m_deformBody_3;
	void createDeformDemo_3(const btVector3& startPos);

	Ogre::ManualObject* m_BuunyManualObject_3;
	btSoftBody* m_BuunyBody_3;
	void createGimpactBuuny(void);


	void createGimpactBarrel(void);

	void createGimpactTorus(void);


	btSoftBody* m_deformableModel;
	btSoftBody* m_mesh;

	btSoftBody* m_bunny;

	
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