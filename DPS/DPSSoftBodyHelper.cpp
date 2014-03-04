#include "DPSSoftBodyHelper.h"
#include "objloader.h"
#include "Mesh/BunnyMesh.h"
#include "Mesh/TorusMesh.h"
#include "Mesh/barrel.h"
#include <btBulletDynamicsCommon.h>
#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>
#include <BulletSoftBody/btDefaultSoftBodySolver.h>
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h>

DPSSoftBodyHelper::DPSSoftBodyHelper(btSoftRigidDynamicsWorld* phyWorld, Ogre::Camera* mCamera, Ogre::SceneManager* mSceneMgr)
{
	this->phyWorld = phyWorld;
	this->mCamera = mCamera;
	this->mSceneMgr = mSceneMgr;
}


DPSSoftBodyHelper::~DPSSoftBodyHelper(void)
{
}


btSoftBody* DPSSoftBodyHelper::createSoftBody(const btVector3& startPos)
{
	m_SoftBody = btSoftBodyHelpers::CreateEllipsoid(phyWorld->getWorldInfo(), startPos, btVector3(2,2,2), 100);
	//m_SoftBody->m_cfg.viterations=50;
	//m_SoftBody->m_cfg.piterations=50;
	//set the liquid body properties
	//m_SoftBody->m_cfg.kPR = 3500.f;
	//m_SoftBody->m_cfg.kDP = 0.001f;
	//m_SoftBody->m_cfg.kDF = 0.1f;
	//m_SoftBody->m_cfg.kKHR = 1.f; //we hardcode this parameter, since any value below 1.0 means the soft body does less than full correction on penetration
	//m_SoftBody->m_cfg.kCHR  = 1.f;
	//m_SoftBody->setTotalMass(50.0);
	//m_SoftBody->setMass(0,0);
	//m_LiquidBody->generateClusters(100);
	//m_SoftBody->m_materials[0]->m_kLST = 0.1f;

// 	m_SoftBody->m_materials[0]->m_kLST	=	0.1;
// 	m_SoftBody->m_cfg.kDF				=	1;
// 	m_SoftBody->m_cfg.kDP				=	0.001;
// 	m_SoftBody->m_cfg.kPR				=	2500;
// 	m_SoftBody->setTotalMass(30,true);
// 	m_SoftBody->setMass(0,0);

	m_SoftBody->m_cfg.kPR = 3500.f;
	m_SoftBody->m_cfg.kDP = 0.001f;
	m_SoftBody->m_cfg.kDF = 0.1f;
	m_SoftBody->m_cfg.kKHR = 1.f; //we hardcode this parameter, since any value below 1.0 means the soft body does less than full correction on penetration
	m_SoftBody->m_cfg.kCHR  = 1.f;

	m_SoftBody->generateClusters(100);
	m_SoftBody->m_materials[0]->m_kLST = 0.1f;

	phyWorld->addSoftBody(m_SoftBody);

	return m_SoftBody;
}


btSoftBody* DPSSoftBodyHelper::createDeformableModel(void)
{
	std::vector<float> triangles;
	std::vector<int> indicies;
	Objloader* obj = new Objloader;
	obj->LoadModel("monkey",&triangles,&indicies);
	//load("monkey.obj",&triangles,&indicies);

	m_deformableModel = btSoftBodyHelpers::CreateFromTriMesh(phyWorld->getWorldInfo(),&(triangles[0]),&(indicies[0]),indicies.size()/3,true);
	m_deformableModel->setTotalMass(20.0,true);
	//m_deformableModel->generateClusters(1000);
	m_deformableModel->m_cfg.kSRHR_CL=1.0;	
	//m_deformableModel->m_cfg.collisions =	btSoftBody::fCollision::CL_RS;
	m_deformableModel->m_cfg.viterations=500;
	m_deformableModel->m_cfg.piterations=500;
	m_deformableModel->m_cfg.citerations=500;
	m_deformableModel->m_cfg.diterations=500;
	m_deformableModel->m_cfg.kPR=500;
	//m_deformableModel->translate(btVector3(0,5,0));
// 	//softMonkey->setMass(0,0);


	btSoftBody::Material* pm = m_deformableModel->appendMaterial();
	pm->m_kLST				=	0.5;
	pm->m_flags				-=	btSoftBody::fMaterial::DebugDraw;
	m_deformableModel->generateBendingConstraints(2,pm);
	m_deformableModel->m_cfg.piterations	=	2;
	m_deformableModel->m_cfg.kDF			=	0.5;
	m_deformableModel->m_cfg.collisions	|=	btSoftBody::fCollision::VF_SS;
	m_deformableModel->translate(btVector3(0,5,0));
	m_deformableModel->scale(btVector3(3,3,3));
	//m_deformableModel->setTotalMass(1,true);
	phyWorld->addSoftBody(m_deformableModel);

	return m_deformableModel;
}


btSoftBody* DPSSoftBodyHelper::createCloth(void)
{
	float s=4;
	float h=20;
	m_cloth = btSoftBodyHelpers::CreatePatch(phyWorld->getWorldInfo(),btVector3(-s,h,-s),btVector3(s,h,-s),btVector3(-s,h,s),btVector3(s,h,s),20,20,8,true);
	//m_cloth->m_cfg.viterations=5;
	m_cloth->m_cfg.piterations=2;
	m_cloth->m_cfg.aeromodel	=	btSoftBody::eAeroModel::V_TwoSided;


	m_cloth->setWindVelocity(btVector3(2060,10,1400));

	m_cloth->setTotalMass(3.0);
	m_cloth->translate(btVector3(2060,5,1420));
	//m_cloth->setMass(100,100);
	//m_cloth->setCollisionFlags(m_cloth->getCollisionFlags() | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
	phyWorld->addSoftBody(m_cloth);

	return m_cloth;
}


btSoftBody* DPSSoftBodyHelper::createBunny(void)
{
	m_bunny = btSoftBodyHelpers::CreateFromTriMesh(phyWorld->getWorldInfo(),gVerticesBunny,&gIndicesBunny[0][0],BUNNY_NUM_TRIANGLES);
// 	btSoftBody::Material* pm = m_bunny->appendMaterial();
// 	pm->m_kLST				=	0.5;
// 	pm->m_flags				-=	btSoftBody::fMaterial::DebugDraw;
// 	m_bunny->generateBendingConstraints(2,pm);
// 	m_bunny->m_cfg.piterations	=	2;
// 	m_bunny->m_cfg.kDF			=	0.5;
// 	m_bunny->m_cfg.collisions	|=	btSoftBody::fCollision::VF_SS;
// 	m_bunny->translate(btVector3(0,5,0));
// 	m_bunny->scale(btVector3(3,3,3));
// 	m_bunny->setTotalMass(1,true);


	m_bunny->m_cfg.kSRHR_CL=1.0;	
	m_bunny->m_cfg.kCHR=1.0;
	m_bunny->m_cfg.kSHR=1.0;
	m_bunny->m_cfg.kAHR=1.0;
	m_bunny->m_cfg.kPR=50;
	m_bunny->m_cfg.piterations=50;

	m_bunny->translate(btVector3(0,5,0));
	m_bunny->setTotalMass(20.0,true);
	m_bunny->scale(btVector3(3,3,3));

	phyWorld->addSoftBody(m_bunny);
	return(m_bunny);
}


