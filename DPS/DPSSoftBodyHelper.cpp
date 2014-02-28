#include "DPSSoftBodyHelper.h"
#include "objloader.h"
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
	m_SoftBody = btSoftBodyHelpers::CreateEllipsoid(phyWorld->getWorldInfo(), startPos, btVector3(1,1,1)*3, 512);
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

	m_SoftBody->m_materials[0]->m_kLST	=	0.1;
	m_SoftBody->m_cfg.kDF				=	1;
	m_SoftBody->m_cfg.kDP				=	0.001;
	m_SoftBody->m_cfg.kPR				=	2500;
	m_SoftBody->setTotalMass(30,true);
	m_SoftBody->setMass(0,0);
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
	m_deformableModel->translate(btVector3(0,5,0));
	//softMonkey->setMass(0,0);
	phyWorld->addSoftBody(m_deformableModel);

	return m_deformableModel;
}


btSoftBody* DPSSoftBodyHelper::createCloth(void)
{
	float s=4;
	float h=20;
	m_cloth = btSoftBodyHelpers::CreatePatch(phyWorld->getWorldInfo(),btVector3(-s,h,-s),btVector3(s,h,-s),btVector3(-s,h,s),btVector3(s,h,s),50,50,4+8,true);
	m_cloth->m_cfg.viterations=50;
	m_cloth->m_cfg.piterations=50;
	m_cloth->setTotalMass(3.0);
	//m_cloth->setMass(100,100);
	phyWorld->addSoftBody(m_cloth);

	return m_cloth;
}