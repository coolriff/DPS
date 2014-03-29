#include "DPSSoftBodyHelper.h"
#include "objloader.h"
#include "Mesh/BunnyMesh.h"
#include "Mesh/TorusMesh.h"
#include "Mesh/barrel.h"
#include "Mesh/barrelg.h"
#include <btBulletDynamicsCommon.h>
#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>
#include <BulletSoftBody/btDefaultSoftBodySolver.h>
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h>
#include "BulletCollision/Gimpact/btGImpactShape.h"
#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"

#define RAND_MAX 0x7fff
#define SIMD_PI btScalar(3.1415926535897932384626433832795029)
#define SIMD_HALF_PI (SIMD_PI * btScalar(0.5))


DPSSoftBodyHelper::DPSSoftBodyHelper(btSoftRigidDynamicsWorld* phyWorld, Ogre::Camera* mCamera, Ogre::SceneManager* mSceneMgr, btAlignedObjectArray<btCollisionShape*> m_collisionShapes)
{
	this->phyWorld = phyWorld;
	this->mCamera = mCamera;
	this->mSceneMgr = mSceneMgr;
	this->m_collisionShapes = m_collisionShapes;
}


DPSSoftBodyHelper::~DPSSoftBodyHelper(void)
{
}


btSoftBody* DPSSoftBodyHelper::createDeformableModel(void)
{
	Objloader* obj = new Objloader;
	obj->LoadModel("suzanne",&triangles,&indicies,&texCoord);
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
	//m_deformableModel->scale(btVector3(0.1,0.1,0.1));
// 	//m_deformableModel->scale(btVector3(3,3,3));
// 	//m_deformableModel->setTotalMass(1,true);
	phyWorld->addSoftBody(m_deformableModel);

	return m_deformableModel;
}

btSoftBody* DPSSoftBodyHelper::createMesh(void)
{
	Objloader* obj = new Objloader;
	obj->LoadModel("mycloth",&triangles,&indicies,&texCoord);
	//load("monkey.obj",&triangles,&indicies);

	m_mesh = btSoftBodyHelpers::CreateFromTriMesh(phyWorld->getWorldInfo(),&(triangles[0]),&(indicies[0]),indicies.size()/3,true);
	m_mesh->setTotalMass(30.0,true);
	phyWorld->addSoftBody(m_mesh);

	return m_mesh;
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

// void DPSSoftBodyHelper::initSoftBody(Ogre::ManualObject*& m_ManualObject, btSoftBody* body)
// {
// 	//manual objects are used to generate new meshes based on raw vertex data
// 	//this is used for the liquid form
// 	m_ManualObject = mSceneMgr->createManualObject("liquidBody");
// 	m_ManualObject->setDynamic(true);
// 	m_ManualObject->setCastShadows(true);
// 
// 	btSoftBody::tNodeArray& nodes(body->m_nodes);
// 	btSoftBody::tFaceArray& faces(body->m_faces);
// 
// 	m_ManualObject->estimateVertexCount(faces.size()*3);
// 	m_ManualObject->estimateIndexCount(faces.size()*3);
// 
// 	//m_ManualObject->begin("CharacterMaterials/LiquidBody", Ogre::RenderOperation::OT_TRIANGLE_LIST);
// 	//m_ManualObject->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);
// 	//m_ManualObject->begin("FlatVertexColour", Ogre::RenderOperation::OT_TRIANGLE_LIST);
// 	m_ManualObject->begin("softbody", Ogre::RenderOperation::OT_TRIANGLE_LIST);
// 
// 	//btSoftBody::Node *node0 = 0, *node1 = 0, *node2 = 0;
// 
// 	//http://www.ogre3d.org/tikiwiki/ManualObject
// 	for (int i = 0; i < faces.size(); ++i)
// 	{
// 		//node0 = faces[i].m_n[0];
// 		//node1 = faces[i].m_n[1];
// 		//node2 = faces[i].m_n[2];
// 
// 		//problem of rendering texture - DO NOT USE NORMALS IN UPDATE FUNCTION FOR NOW! 
// 		m_ManualObject->position(body->m_faces[i].m_n[0]->m_x.x(),body->m_faces[i].m_n[0]->m_x.y(),body->m_faces[i].m_n[0]->m_x.z());
// 		m_ManualObject->colour(Ogre::ColourValue(0.7f,0.7f,0.7f,1.0f));
// 		//m_ManualObject->colour(0.5f,0.5f,0.5f);
// 		//m_ManualObject->textureCoord(dpsSoftbodyHelper->texCoord[i],dpsSoftbodyHelper->texCoord[i+1]);
// 		m_ManualObject->position(body->m_faces[i].m_n[1]->m_x.x(),body->m_faces[i].m_n[1]->m_x.y(),body->m_faces[i].m_n[1]->m_x.z());
// 		m_ManualObject->colour(Ogre::ColourValue(0.7f,0.7f,0.7f,1.0f));
// 		//m_ManualObject->colour(0.5f,0.5f,0.5f);
// 		//m_ManualObject->textureCoord(dpsSoftbodyHelper->texCoord[i+1],dpsSoftbodyHelper->texCoord[i+2]);
// 		m_ManualObject->position(body->m_faces[i].m_n[2]->m_x.x(),body->m_faces[i].m_n[2]->m_x.y(),body->m_faces[i].m_n[2]->m_x.z());
// 		m_ManualObject->colour(Ogre::ColourValue(0.7f,0.7f,0.7f,1.0f));
// 		//m_ManualObject->colour(0.5f,0.5f,0.5f);
// 		//m_ManualObject->textureCoord(dpsSoftbodyHelper->texCoord[i+2],dpsSoftbodyHelper->texCoord[i+3]);
// 		// 
// 		// 		m_ManualObject->position(body->m_faces[i].m_n[0]->m_x[0],body->m_faces[i].m_n[0]->m_x[1],body->m_faces[i].m_n[0]->m_x[2]);
// 		// 		m_ManualObject->position(body->m_faces[i].m_n[1]->m_x[0],body->m_faces[i].m_n[1]->m_x[1],body->m_faces[i].m_n[1]->m_x[2]);
// 		// 		m_ManualObject->position(body->m_faces[i].m_n[2]->m_x[0],body->m_faces[i].m_n[2]->m_x[1],body->m_faces[i].m_n[2]->m_x[2]);
// 		// 
// 		m_ManualObject->normal(body->m_faces[i].m_n[0]->m_n[0], body->m_faces[i].m_n[0]->m_n[1], body->m_faces[i].m_n[0]->m_n[2]);
// 		m_ManualObject->normal(body->m_faces[i].m_n[1]->m_n[0], body->m_faces[i].m_n[1]->m_n[1], body->m_faces[i].m_n[1]->m_n[2]);
// 		m_ManualObject->normal(body->m_faces[i].m_n[2]->m_n[0], body->m_faces[i].m_n[2]->m_n[1], body->m_faces[i].m_n[2]->m_n[2]);
// 
// 		//m_ManualObject->position(node0->m_x[0], node0->m_x[1], node0->m_x[2]);
// 		//m_ManualObject->textureCoord(1,0);
// 		//m_ManualObject->normal(node0->m_n[0], node0->m_n[1], node0->m_n[2]);
// 
// 		//m_ManualObject->position(node1->m_x[0], node1->m_x[1], node1->m_x[2]);
// 		//m_ManualObject->textureCoord(0,1);
// 		//m_ManualObject->normal(node1->m_n[0], node1->m_n[1], node1->m_n[2]);
// 
// 		//m_ManualObject->position(node2->m_x[0], node2->m_x[1], node2->m_x[2]);
// 		//m_ManualObject->textureCoord(1,1);
// 		//m_ManualObject->normal(node2->m_n[0], node2->m_n[1], node2->m_n[2]);
// 
// 		m_ManualObject->triangle(i*3,i*3+1,i*3+2);
// 		// 		m_ManualObject->triangle(i*3+1);
// 		// 		m_ManualObject->triangle(i*3+2);
// 	}
// 	// 	m_ManualObject->textureCoord(1,0);
// 	// 	m_ManualObject->textureCoord(0,0);
// 	// 	m_ManualObject->textureCoord(0,1);
// 	// 	m_ManualObject->textureCoord(1,1);
// 	m_ManualObject->end();
// }
// 
// 
// void DPSSoftBodyHelper::updateSoftBody(Ogre::ManualObject*& m_ManualObject, btSoftBody* body)
// {
// 	//grab the calculated mesh data from the physics body
// 	btSoftBody::tNodeArray& nodes(body->m_nodes);
// 	btSoftBody::tFaceArray& faces(body->m_faces);
// 
// 	m_ManualObject->beginUpdate(0);
// 	/*	btSoftBody::Node *node0 = 0, *node1 = 0, *node2 = 0;*/
// 	for (int i = 0; i < faces.size(); i++)
// 	{
// 		m_ManualObject->position(body->m_faces[i].m_n[0]->m_x.x(),body->m_faces[i].m_n[0]->m_x.y(),body->m_faces[i].m_n[0]->m_x.z());
// 		//m_ManualObject->colour(0.5f,0.5f,0.5f);
// 		m_ManualObject->position(body->m_faces[i].m_n[1]->m_x.x(),body->m_faces[i].m_n[1]->m_x.y(),body->m_faces[i].m_n[1]->m_x.z());
// 		//m_ManualObject->colour(0.5f,0.5f,0.5f);
// 		m_ManualObject->position(body->m_faces[i].m_n[2]->m_x.x(),body->m_faces[i].m_n[2]->m_x.y(),body->m_faces[i].m_n[2]->m_x.z());
// 		//m_ManualObject->colour(0.5f,0.5f,0.5f);
// 		// 
// 		// 		m_ManualObject->textureCoord(body->m_faces[i].m_n[1]->m_x.x(), body->m_faces[i].m_n[1]->m_x.y(), body->m_faces[i].m_n[1]->m_x.z());
// 		// 		m_ManualObject->textureCoord(body->m_faces[i].m_n[2]->m_x.x(), body->m_faces[i].m_n[2]->m_x.y(), body->m_faces[i].m_n[2]->m_x.z());
// 
// 		// 		m_ManualObject->position(body->m_faces[i].m_n[0]->m_x[0],body->m_faces[i].m_n[0]->m_x[1],body->m_faces[i].m_n[0]->m_x[2]);
// 		// 		m_ManualObject->position(body->m_faces[i].m_n[1]->m_x[0],body->m_faces[i].m_n[1]->m_x[1],body->m_faces[i].m_n[1]->m_x[2]);
// 		// 		m_ManualObject->position(body->m_faces[i].m_n[2]->m_x[0],body->m_faces[i].m_n[2]->m_x[1],body->m_faces[i].m_n[2]->m_x[2]);
// 		// 
// 		m_ManualObject->normal(body->m_faces[i].m_n[0]->m_n[0], body->m_faces[i].m_n[0]->m_n[1], body->m_faces[i].m_n[0]->m_n[2]);
// 		m_ManualObject->normal(body->m_faces[i].m_n[1]->m_n[0], body->m_faces[i].m_n[1]->m_n[1], body->m_faces[i].m_n[1]->m_n[2]);
// 		m_ManualObject->normal(body->m_faces[i].m_n[2]->m_n[0], body->m_faces[i].m_n[2]->m_n[1], body->m_faces[i].m_n[2]->m_n[2]);
// 
// 		// 		node0 = faces[i].m_n[0];
// 		// 		node1 = faces[i].m_n[1];
// 		// 		node2 = faces[i].m_n[2];
// 
// 		// 		m_ManualObject->position(node0->m_x[0], node0->m_x[1], node0->m_x[2]);
// 		// 		m_ManualObject->normal(node0->m_n[0], node0->m_n[1], node0->m_n[2]);
// 		// 				
// 		// 		m_ManualObject->position(node1->m_x[0], node1->m_x[1], node1->m_x[2]);
// 		// 		m_ManualObject->normal(node1->m_n[0], node1->m_n[1], node1->m_n[2]);
// 		// 				
// 		// 		m_ManualObject->position(node2->m_x[0], node2->m_x[1], node2->m_x[2]);
// 		// 		m_ManualObject->normal(node2->m_n[0], node2->m_n[1], node2->m_n[2]);
// 
// 		m_ManualObject->index(i*3);
// 		m_ManualObject->index(i*3+1);
// 		m_ManualObject->index(i*3+2);
// 	}
// 
// 	m_ManualObject->end();
// }



void DPSSoftBodyHelper::createClothDemo_1(void)
{
	float s=4;
	float h=10;
	m_clothBody_1 = btSoftBodyHelpers::CreatePatch(phyWorld->getWorldInfo(),btVector3(-s,h,-s),btVector3(s,h,-s),btVector3(-s,h,s),btVector3(s,h,s),20,20,4+8,true);
	m_clothBody_1->setTotalMass(3.0);
	phyWorld->addSoftBody(m_clothBody_1);

	initSoftBody(m_clothManualObject_1, m_clothBody_1);

	Ogre::SceneNode* m_clothNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	m_clothNode->attachObject(m_clothManualObject_1);
}


void DPSSoftBodyHelper::createClothDemo_2(void)
{
	float s=4;
	float h=10;
	m_clothBody_2 = btSoftBodyHelpers::CreatePatch(phyWorld->getWorldInfo(),btVector3(-s,h,-s),btVector3(s,h,-s),btVector3(-s,h,s),btVector3(s,h,s),20,20,1+2+4+8,true);
	m_clothBody_2->setTotalMass(3.0);
	phyWorld->addSoftBody(m_clothBody_2);

	initSoftBody(m_clothManualObject_2, m_clothBody_2);

	Ogre::SceneNode* m_clothNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	m_clothNode->attachObject(m_clothManualObject_2);
}


void DPSSoftBodyHelper::createClothDemo_3(void)
{
	float s=4;
	float h=10;
	//btQuaternion orientation(-SIMD_PI/2,0,0);
	m_clothBody_3 = btSoftBodyHelpers::CreatePatch(phyWorld->getWorldInfo(),btVector3(-s,h,-s),btVector3(s,h,-s),btVector3(-s,h,s),btVector3(s,h,s),50,50,0,true);
	m_clothBody_3->setTotalMass(0.1);
	m_clothBody_3->m_cfg.piterations = 10;
	m_clothBody_3->m_cfg.citerations = 10;
	m_clothBody_3->m_cfg.diterations = 10;
	//m_clothBody_3->rotate(orientation);
	phyWorld->addSoftBody(m_clothBody_3);

	initSoftBody(m_clothManualObject_3, m_clothBody_3);

	Ogre::SceneNode* m_clothNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	m_clothNode->attachObject(m_clothManualObject_3);
	//m_clothNode->pitch(Ogre::Degree(180));
}


void DPSSoftBodyHelper::createClothDemo_4(btRigidBody* body)
{
	float s=4;
	float h=15;
	m_clothBody_4 = btSoftBodyHelpers::CreatePatch(phyWorld->getWorldInfo(),btVector3(-s,h,-s),btVector3(s,h,-s),btVector3(-s,h,s),btVector3(s,h,s),20,20,4+8,true);
	phyWorld->addSoftBody(m_clothBody_4);

	initSoftBody(m_clothManualObject_4, m_clothBody_4);

	Ogre::SceneNode* m_clothNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	m_clothNode->attachObject(m_clothManualObject_4);

	
	m_clothBody_4->appendAnchor(0,body);
	m_clothBody_4->appendAnchor(19,body);

}


void DPSSoftBodyHelper::createClothDemo_5(void)
{
	float s=8;
	float h=10;
	m_clothBody_5_0 = btSoftBodyHelpers::CreatePatch(phyWorld->getWorldInfo(),btVector3(-s,h,-s),btVector3(s,h,-s),btVector3(-s,h,s),btVector3(s,h,s),15,15,1+2+4+8,true);
	m_clothBody_5_0->m_materials[0]->m_kLST	= 0.4;
	m_clothBody_5_0->m_cfg.collisions |= btSoftBody::fCollision::VF_SS;
	m_clothBody_5_0->setTotalMass(10);
	phyWorld->addSoftBody(m_clothBody_5_0);
	initSoftBody(m_clothManualObject_5_0, m_clothBody_5_0);

	Ogre::SceneNode* m_clothNode_0 = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	m_clothNode_0->attachObject(m_clothManualObject_5_0);

	float s1=4;
	float h1=10;
	btVector3 p = btVector3(5,10,0);
	m_clothBody_5_1 = btSoftBodyHelpers::CreatePatch(phyWorld->getWorldInfo(),btVector3(-s1,h1,-s1)+p,btVector3(s1,h1,-s1)+p,btVector3(-s1,h1,s1)+p,btVector3(s1,h1,s1)+p,7,7,0,true);
	btSoftBody::Material* pm = m_clothBody_5_1->appendMaterial();
	pm->m_kLST = 0.1;
	pm->m_flags -= btSoftBody::fMaterial::DebugDraw;
	m_clothBody_5_1->generateBendingConstraints(2,pm);
	m_clothBody_5_1->m_materials[0]->m_kLST	= 0.5;
	m_clothBody_5_1->m_cfg.collisions |= btSoftBody::fCollision::VF_SS;
	m_clothBody_5_1->setTotalMass(10);
	phyWorld->addSoftBody(m_clothBody_5_1);
	initSoftBody(m_clothManualObject_5_1, m_clothBody_5_1);

	Ogre::SceneNode* m_clothNode_1 = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	m_clothNode_1->attachObject(m_clothManualObject_5_1);
}


void DPSSoftBodyHelper::createClothDemo_6(void)
{
	float s = 5;
	float h = 10;

	//1
	m_clothBody_6_0 = btSoftBodyHelpers::CreatePatch(phyWorld->getWorldInfo(),btVector3(-s,h,-s),btVector3(s,h,-s),btVector3(-s,h,s),btVector3(s,h,s),6,6,0,true);
	btSoftBody::Material* pm = m_clothBody_6_0->appendMaterial();
	pm->m_flags -= btSoftBody::fMaterial::DebugDraw;
	m_clothBody_6_0->generateBendingConstraints(2,pm);
	m_clothBody_6_0->m_cfg.kLF = 0.004;
	m_clothBody_6_0->m_cfg.kDG = 0.0003;
	m_clothBody_6_0->m_cfg.aeromodel = btSoftBody::eAeroModel::V_TwoSided;
	btTransform trs;
	btQuaternion rot;
	btVector3 ra = Vector3Rand()*0.1;
	btVector3 rp = Vector3Rand()*15+btVector3(0,20,80);
	rot.setEuler(SIMD_PI/8+ra.x(),-SIMD_PI/7+ra.y(),ra.z());
	trs.setIdentity();
	trs.setOrigin(rp);
	trs.setRotation(rot);
	m_clothBody_6_0->transform(trs);
	m_clothBody_6_0->setTotalMass(0.1);
	m_clothBody_6_0->addForce(btVector3(0,2,0),0);
	phyWorld->addSoftBody(m_clothBody_6_0);

	initSoftBody(m_clothManualObject_6_0, m_clothBody_6_0);

	Ogre::SceneNode* m_clothNode_0 = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	m_clothNode_0->attachObject(m_clothManualObject_6_0);


	//2
	m_clothBody_6_1 = btSoftBodyHelpers::CreatePatch(phyWorld->getWorldInfo(),btVector3(-s,h,-s),btVector3(s,h,-s),btVector3(-s,h,s),btVector3(s,h,s),6,6,0,true);
	btSoftBody::Material* pm1 = m_clothBody_6_1->appendMaterial();
	pm1->m_flags -= btSoftBody::fMaterial::DebugDraw;
	m_clothBody_6_1->generateBendingConstraints(2,pm);
	m_clothBody_6_1->m_cfg.kLF = 0.004;
	m_clothBody_6_1->m_cfg.kDG = 0.0003;
	m_clothBody_6_1->m_cfg.aeromodel = btSoftBody::eAeroModel::V_TwoSided;
	btTransform trs1;
	btQuaternion rot1;
	btVector3 ra1 = Vector3Rand()*0.1;
	btVector3 rp1 = Vector3Rand()*15+btVector3(0,20,80);
	rot1.setEuler(SIMD_PI/8+ra1.x(),-SIMD_PI/7+ra1.y(),ra1.z());
	trs1.setIdentity();
	trs1.setOrigin(rp1);
	trs1.setRotation(rot1);
	m_clothBody_6_1->transform(trs1);
	m_clothBody_6_1->setTotalMass(0.1);
	m_clothBody_6_1->addForce(btVector3(0,2,0),0);
	phyWorld->addSoftBody(m_clothBody_6_1);

	initSoftBody(m_clothManualObject_6_1, m_clothBody_6_1);

	Ogre::SceneNode* m_clothNode_1 = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	m_clothNode_1->attachObject(m_clothManualObject_6_1);


	//3
	m_clothBody_6_2 = btSoftBodyHelpers::CreatePatch(phyWorld->getWorldInfo(),btVector3(-s,h,-s),btVector3(s,h,-s),btVector3(-s,h,s),btVector3(s,h,s),6,6,0,true);
	btSoftBody::Material* pm2 = m_clothBody_6_2->appendMaterial();
	pm2->m_flags -= btSoftBody::fMaterial::DebugDraw;
	m_clothBody_6_2->generateBendingConstraints(2,pm);
	m_clothBody_6_2->m_cfg.kLF = 0.004;
	m_clothBody_6_2->m_cfg.kDG = 0.0003;
	m_clothBody_6_2->m_cfg.aeromodel = btSoftBody::eAeroModel::V_TwoSided;
	btTransform trs2;
	btQuaternion rot2;
	btVector3 ra2 = Vector3Rand()*0.1;
	btVector3 rp2 = Vector3Rand()*15+btVector3(0,20,80);
	rot2.setEuler(SIMD_PI/8+ra2.x(),-SIMD_PI/7+ra2.y(),ra2.z());
	trs2.setIdentity();
	trs2.setOrigin(rp2);
	trs2.setRotation(rot2);
	m_clothBody_6_2->transform(trs2);
	m_clothBody_6_2->setTotalMass(0.1);
	m_clothBody_6_2->addForce(btVector3(0,2,0),0);
	phyWorld->addSoftBody(m_clothBody_6_2);

	initSoftBody(m_clothManualObject_6_2, m_clothBody_6_2);

	Ogre::SceneNode* m_clothNode_2 = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	m_clothNode_2->attachObject(m_clothManualObject_6_2);
}


void DPSSoftBodyHelper::createClothDemo_7(void)
{
	float s = 5;
	float h = 0;

	//1
	m_clothBody_7_0 = btSoftBodyHelpers::CreatePatch(phyWorld->getWorldInfo(),btVector3(-s,h,-s*3),btVector3(s,h,-s*3),btVector3(-s,h,s),btVector3(s,h,s),10,30,1+2,true);
	m_clothBody_7_0->getCollisionShape()->setMargin(0.5);
	btSoftBody::Material* pm = m_clothBody_7_0->appendMaterial();
	pm->m_kLST = 0.0004;
	pm->m_flags -= btSoftBody::fMaterial::DebugDraw;
	m_clothBody_7_0->generateBendingConstraints(2,pm);
	m_clothBody_7_0->m_cfg.kLF = 0.05;
	m_clothBody_7_0->m_cfg.kDG = 0.01;
	m_clothBody_7_0->m_cfg.piterations = 2;
	m_clothBody_7_0->m_cfg.aeromodel = btSoftBody::eAeroModel::V_TwoSidedLiftDrag;
	m_clothBody_7_0->setWindVelocity(btVector3(4, -12.0, -25.0));

	btTransform trs;
	btQuaternion rot;
	rot.setRotation(btVector3(1, 0, 0), btScalar(SIMD_PI/2));
	trs.setIdentity();
	trs.setOrigin(btVector3(7, 40, -10));
	trs.setRotation(rot);
	m_clothBody_7_0->transform(trs);
	m_clothBody_7_0->setTotalMass(2.0);
	phyWorld->addSoftBody(m_clothBody_7_0);

	initSoftBody(m_clothManualObject_7_0, m_clothBody_7_0);

	Ogre::SceneNode* m_clothNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	m_clothNode->attachObject(m_clothManualObject_7_0);


	//2
	m_clothBody_7_1 = btSoftBodyHelpers::CreatePatch(phyWorld->getWorldInfo(),btVector3(-s,h,-s*3),btVector3(s,h,-s*3),btVector3(-s,h,s),btVector3(s,h,s),10,30,1+2,true);
	m_clothBody_7_1->getCollisionShape()->setMargin(0.5);
	btSoftBody::Material* pm1 = m_clothBody_7_1->appendMaterial();
	pm1->m_kLST = 0.0004;
	pm1->m_flags -= btSoftBody::fMaterial::DebugDraw;
	m_clothBody_7_1->generateBendingConstraints(2,pm);
	m_clothBody_7_1->m_cfg.kLF = 0.05;
	m_clothBody_7_1->m_cfg.kDG = 0.01;
	m_clothBody_7_1->m_cfg.piterations = 2;
	m_clothBody_7_1->m_cfg.aeromodel = btSoftBody::eAeroModel::V_TwoSidedLiftDrag;
	m_clothBody_7_1->setWindVelocity(btVector3(4, -12.0, -25.0));

	btTransform trs1;
	btQuaternion rot1;
	rot1.setRotation(btVector3(1, 0, 0), btScalar(SIMD_PI/2));
	trs1.setIdentity();
	trs1.setOrigin(btVector3(-5, 40, 0));
	trs1.setRotation(rot);
	m_clothBody_7_1->transform(trs1);
	m_clothBody_7_1->setTotalMass(2.0);
	phyWorld->addSoftBody(m_clothBody_7_1);

	initSoftBody(m_clothManualObject_7_1, m_clothBody_7_1);

	Ogre::SceneNode* m_clothNode1 = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	m_clothNode1->attachObject(m_clothManualObject_7_1);


	//3
	m_clothBody_7_2 = btSoftBodyHelpers::CreatePatch(phyWorld->getWorldInfo(),btVector3(-s,h,-s*3),btVector3(s,h,-s*3),btVector3(-s,h,s),btVector3(s,h,s),10,30,1+2,true);
	m_clothBody_7_2->getCollisionShape()->setMargin(0.5);
	btSoftBody::Material* pm2 = m_clothBody_7_2->appendMaterial();
	pm2->m_kLST = 0.0004;
	pm2->m_flags -= btSoftBody::fMaterial::DebugDraw;
	m_clothBody_7_2->generateBendingConstraints(2,pm);
	m_clothBody_7_2->m_cfg.kLF = 0.05;
	m_clothBody_7_2->m_cfg.kDG = 0.01;
	m_clothBody_7_2->m_cfg.piterations = 2;
	m_clothBody_7_2->m_cfg.aeromodel = btSoftBody::eAeroModel::V_TwoSidedLiftDrag;
	m_clothBody_7_2->setWindVelocity(btVector3(4, -12.0, -25.0));

	btTransform trs2;
	btQuaternion rot2;
	rot2.setRotation(btVector3(1, 0, 0), btScalar(SIMD_PI/2));
	trs2.setIdentity();
	trs2.setOrigin(btVector3(-16, 40, 10));
	trs2.setRotation(rot2);
	m_clothBody_7_2->transform(trs2);
	m_clothBody_7_2->setTotalMass(2.0);
	phyWorld->addSoftBody(m_clothBody_7_2);

	initSoftBody(m_clothManualObject_7_2, m_clothBody_7_2);

	Ogre::SceneNode* m_clothNode2 = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	m_clothNode2->attachObject(m_clothManualObject_7_2);
}


//btSoftBodyHelpers::CreateFromTriMesh(phyWorld->getWorldInfo(), gbarrel_va, &gbarrel_ia[0][0], gfaces_size);
void DPSSoftBodyHelper::createDeformDemo_1(const btVector3& startPos)
{
	triangles.clear();
	indicies.clear();
	texCoord.clear();

	Objloader* obj = new Objloader;
	obj->LoadModel("softcube8",&triangles,&indicies,&texCoord);
	//load("monkey.obj",&triangles,&indicies);

	m_deformBody_1 = btSoftBodyHelpers::CreateFromTriMesh(phyWorld->getWorldInfo(),&(triangles[0]),&(indicies[0]),indicies.size()/3,true);
	m_deformBody_1->setTotalMass(20.0,true);
	//m_deformableModel->generateClusters(1000);
	m_deformBody_1->m_cfg.kSRHR_CL=1.0;	
	//m_deformableModel->m_cfg.collisions =	btSoftBody::fCollision::CL_RS;
	m_deformBody_1->m_cfg.viterations=500;
	m_deformBody_1->m_cfg.piterations=500;
	m_deformBody_1->m_cfg.citerations=500;
	m_deformBody_1->m_cfg.diterations=500;
	m_deformBody_1->m_cfg.kPR=500;


	// 	btSoftBody::Material* pm = m_softBody_3->appendMaterial();
	// 	pm->m_kLST = 0.5;
	// 	pm->m_flags -= btSoftBody::fMaterial::DebugDraw;
	// 	m_softBody_3->generateBendingConstraints(2,pm);
	// 	m_softBody_3->m_cfg.piterations	=	2;
	// 	m_softBody_3->m_cfg.kDF			=	0.5;
	m_deformBody_1->m_cfg.collisions = btSoftBody::fCollision::SDF_RS + btSoftBody::fCollision::CL_SS + btSoftBody::fCollision::CL_SELF;
	m_deformBody_1->translate(startPos);
	m_deformBody_1->scale(btVector3(2,2,2));
	phyWorld->addSoftBody(m_deformBody_1);

	initSoftBody(m_deformManualObject_1, m_deformBody_1);

	Ogre::SceneNode* m_deformNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	m_deformNode->attachObject(m_deformManualObject_1);
}


void DPSSoftBodyHelper::createDeformDemo_2(const btVector3& startPos)
{
	triangles.clear();
	indicies.clear();
	texCoord.clear();

	Objloader* obj = new Objloader;
	obj->LoadModel("b8",&triangles,&indicies,&texCoord);
	//load("monkey.obj",&triangles,&indicies);

	m_deformBody_2 = btSoftBodyHelpers::CreateFromTriMesh(phyWorld->getWorldInfo(),&(triangles[0]),&(indicies[0]),indicies.size()/3,true);
	m_deformBody_2->setTotalMass(50.0,true);
	//m_deformableModel->generateClusters(1000);
	m_deformBody_2->m_cfg.kSRHR_CL=1.0;	
	//m_deformableModel->m_cfg.collisions =	btSoftBody::fCollision::CL_RS;
	m_deformBody_2->m_cfg.viterations=500;
	m_deformBody_2->m_cfg.piterations=500;
	m_deformBody_2->m_cfg.citerations=500;
	m_deformBody_2->m_cfg.diterations=500;
	m_deformBody_2->m_cfg.kPR=500;


	// 	btSoftBody::Material* pm = m_softBody_3->appendMaterial();
	// 	pm->m_kLST = 0.5;
	// 	pm->m_flags -= btSoftBody::fMaterial::DebugDraw;
	// 	m_softBody_3->generateBendingConstraints(2,pm);
	// 	m_softBody_3->m_cfg.piterations	=	2;
	// 	m_softBody_3->m_cfg.kDF			=	0.5;
	m_deformBody_2->m_cfg.collisions	= btSoftBody::fCollision::SDF_RS + btSoftBody::fCollision::CL_SS + btSoftBody::fCollision::CL_SELF;
	m_deformBody_2->translate(startPos);
	m_deformBody_2->scale(btVector3(2,2,2));
	phyWorld->addSoftBody(m_deformBody_2);

	initSoftBody(m_deformManualObject_2, m_deformBody_2);

	Ogre::SceneNode* m_deformNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	m_deformNode->attachObject(m_deformManualObject_2);
}


void DPSSoftBodyHelper::createDeformDemo_3(const btVector3& startPos)
{
	triangles.clear();
	indicies.clear();
	texCoord.clear();

	Objloader* obj = new Objloader;
	obj->LoadModel("m845",&triangles,&indicies,&texCoord);
	//load("monkey.obj",&triangles,&indicies);

	m_deformBody_3 = btSoftBodyHelpers::CreateFromTriMesh(phyWorld->getWorldInfo(),&(triangles[0]),&(indicies[0]),indicies.size()/3,true);
	m_deformBody_3->setTotalMass(50.0,true);
	//m_deformableModel->generateClusters(1000);
	m_deformBody_3->m_cfg.kSRHR_CL=1.0;	
	//m_deformableModel->m_cfg.collisions =	btSoftBody::fCollision::CL_RS;
	m_deformBody_3->m_cfg.viterations=500;
	m_deformBody_3->m_cfg.piterations=500;
	m_deformBody_3->m_cfg.citerations=500;
	m_deformBody_3->m_cfg.diterations=500;
	m_deformBody_3->m_cfg.kPR=500;


	// 	btSoftBody::Material* pm = m_softBody_3->appendMaterial();
	// 	pm->m_kLST = 0.5;
	// 	pm->m_flags -= btSoftBody::fMaterial::DebugDraw;
	// 	m_softBody_3->generateBendingConstraints(2,pm);
	// 	m_softBody_3->m_cfg.piterations	=	2;
	// 	m_softBody_3->m_cfg.kDF			=	0.5;
	m_deformBody_3->m_cfg.collisions	= btSoftBody::fCollision::SDF_RS + btSoftBody::fCollision::CL_SS + btSoftBody::fCollision::CL_SELF;
	m_deformBody_3->translate(startPos);
	m_deformBody_3->scale(btVector3(2,2,2));
	phyWorld->addSoftBody(m_deformBody_3);

	initSoftBody(m_deformManualObject_3, m_deformBody_3);

	Ogre::SceneNode* m_deformNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	m_deformNode->attachObject(m_deformManualObject_3);
}


void DPSSoftBodyHelper::createDeformDemo_4(const btVector3& startPos)
{
	m_deformBody_4 = btSoftBodyHelpers::CreateFromTriMesh(phyWorld->getWorldInfo(), gVertices, &gIndices[0][0], NUM_TRIANGLES);
	// 	btSoftBody::Material* pm = m_softBody_2->appendMaterial();
	// 	pm->m_kLST = 0.5;
	// 	pm->m_flags -= btSoftBody::fMaterial::DebugDraw;
	// 	m_softBody_2->generateBendingConstraints(2,pm);
	// 	m_softBody_2->m_cfg.piterations = 2;
	// 	m_softBody_2->m_cfg.kDF = 0.5;
	// 	m_softBody_2->m_cfg.collisions = btSoftBody::fCollision::SDF_RS + btSoftBody::fCollision::CL_SS + btSoftBody::fCollision::CL_SELF;
	// 
	// 	m_softBody_2->randomizeConstraints();
	// 	m_softBody_2->translate(startPos);
	// 	m_softBody_2->scale(btVector3(6,6,6));
	// 	m_softBody_2->setTotalMass(50);


	m_deformBody_4->m_cfg.kSRHR_CL=1.0;	
	m_deformBody_4->m_cfg.kCHR=1.0;
	m_deformBody_4->m_cfg.kSHR=1.0;
	m_deformBody_4->m_cfg.kAHR=1.0;
	m_deformBody_4->m_cfg.kPR=50;
	m_deformBody_4->m_cfg.piterations=50;

	m_deformBody_4->translate(startPos);
	m_deformBody_4->setTotalMass(20.0,true);
	m_deformBody_4->scale(btVector3(3,3,3));
	phyWorld->addSoftBody(m_deformBody_4);

	initSoftBody(m_deformManualObject_4, m_deformBody_4);

	Ogre::SceneNode* m_softNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	m_softNode->attachObject(m_deformManualObject_4);
}


void DPSSoftBodyHelper::createDeformDemo_5(const btVector3& startPos)
{
	m_deformBody_5 = btSoftBodyHelpers::CreateEllipsoid(phyWorld->getWorldInfo(), startPos, btVector3(2,2,2), 100);
	//m_SoftBody->m_cfg.viterations=50;
	//m_SoftBody->m_cfg.piterations=50;
	//set the liquid body properties
	m_deformBody_5->m_cfg.kPR = 3500.f;
	m_deformBody_5->m_cfg.kDP = 0.001f;
	m_deformBody_5->m_cfg.kDF = 0.1f;
	m_deformBody_5->m_cfg.kKHR = 1.f; //we hardcode this parameter, since any value below 1.0 means the soft body does less than full correction on penetration
	m_deformBody_5->m_cfg.kCHR  = 1.f;
	m_deformBody_5->setTotalMass(150.0);
	m_deformBody_5->setMass(0,0);
	//m_LiquidBody->generateClusters(100);
	m_deformBody_5->m_materials[0]->m_kLST = 0.1f;

	phyWorld->addSoftBody(m_deformBody_5);

	initSoftBody(m_deformManualObject_5, m_deformBody_5);

	Ogre::SceneNode* m_softNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	m_softNode->attachObject(m_deformManualObject_5);

}


void DPSSoftBodyHelper::createDeformDemo_6(const btVector3& startPos)
{
	m_deformBody_6 = btSoftBodyHelpers::CreateFromTriMesh(phyWorld->getWorldInfo(), gVerticesBunny, &gIndicesBunny[0][0], BUNNY_NUM_TRIANGLES);
	// 	btSoftBody::Material* pm = m_softBody_2->appendMaterial();
	// 	pm->m_kLST = 0.5;
	// 	pm->m_flags -= btSoftBody::fMaterial::DebugDraw;
	// 	m_softBody_2->generateBendingConstraints(2,pm);
	// 	m_softBody_2->m_cfg.piterations = 2;
	// 	m_softBody_2->m_cfg.kDF = 0.5;
	// 	m_softBody_2->m_cfg.collisions = btSoftBody::fCollision::SDF_RS + btSoftBody::fCollision::CL_SS + btSoftBody::fCollision::CL_SELF;
	// 
	// 	m_softBody_2->randomizeConstraints();
	// 	m_softBody_2->translate(startPos);
	// 	m_softBody_2->scale(btVector3(6,6,6));
	// 	m_softBody_2->setTotalMass(50);


	m_deformBody_6->m_cfg.kSRHR_CL=1.0;	
	m_deformBody_6->m_cfg.kCHR=1.0;
	m_deformBody_6->m_cfg.kSHR=1.0;
	m_deformBody_6->m_cfg.kAHR=1.0;
	m_deformBody_6->m_cfg.kPR=50;
	m_deformBody_6->m_cfg.piterations=50;

	m_deformBody_6->translate(startPos);
	m_deformBody_6->setTotalMass(20.0,true);
	m_deformBody_6->scale(btVector3(3,3,3));

	phyWorld->addSoftBody(m_deformBody_6);

	initSoftBody(m_deformManualObject_6, m_deformBody_6);

	Ogre::SceneNode* m_softNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	m_softNode->attachObject(m_deformManualObject_6);
}


//playground
void DPSSoftBodyHelper::createCarWheels(Ogre::ManualObject*& ManualObject, btSoftBody*& body, btVector3& startPos, btVector3& rotation, btVector3& scaler)
{
	body = btSoftBodyHelpers::CreateFromTriMesh(phyWorld->getWorldInfo(),gVertices,&gIndices[0][0],NUM_TRIANGLES);
	btSoftBody::Material* pm = body->appendMaterial();
	pm->m_kLST = 1;
	pm->m_flags -= btSoftBody::fMaterial::DebugDraw;			
	body->generateBendingConstraints(2,pm);
	body->m_cfg.piterations = 2;
	body->m_cfg.kDF = 1;
	body->m_cfg.collisions = btSoftBody::fCollision::CL_SS + btSoftBody::fCollision::CL_RS;
	body->randomizeConstraints();
	body->scale(scaler);
	body->rotate(btQuaternion(rotation[0],rotation[1],rotation[2]));
	body->translate(startPos);
	body->setTotalMass(50,true);
	body->generateClusters(64);			
	phyWorld->addSoftBody(body);

	initCar(ManualObject, body);

	Ogre::SceneNode* m_CarWheelNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	m_CarWheelNode->attachObject(ManualObject);
}


void DPSSoftBodyHelper::createTorus(Ogre::ManualObject*& ManualObject, btSoftBody*& body, btVector3& startPos, btVector3& rotation, btVector3& scaler)
{
	body = btSoftBodyHelpers::CreateFromTriMesh(phyWorld->getWorldInfo(), gVertices, &gIndices[0][0], NUM_TRIANGLES);
	body->m_cfg.kSRHR_CL=1.0;	
	body->m_cfg.kCHR=1.0;
	body->m_cfg.kSHR=1.0;
	body->m_cfg.kAHR=1.0;
	body->m_cfg.kPR=50;
	body->m_cfg.piterations=50;
	body->m_cfg.collisions |= btSoftBody::fCollision::CL_SS;
	body->translate(startPos);
	body->setTotalMass(20.0,true);
	body->scale(scaler);
	phyWorld->addSoftBody(body);

	initCar(ManualObject, body);

	Ogre::SceneNode* m_CarWheelNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	m_CarWheelNode->attachObject(ManualObject);
}


void DPSSoftBodyHelper::createCarBody(Ogre::ManualObject*& ManualObject, btSoftBody*& body, btVector3& startPos, btVector3& rotation, btVector3& scaler)
{

	body=btSoftBodyHelpers::CreateFromTriMesh(phyWorld->getWorldInfo(),gVerticesBunny,&gIndicesBunny[0][0],BUNNY_NUM_TRIANGLES);
	btSoftBody::Material* pm = body->appendMaterial();
	pm->m_kLST = 1;
	pm->m_flags -= btSoftBody::fMaterial::DebugDraw;
 	body->generateBendingConstraints(2,pm);
	body->m_cfg.piterations	= 2;
	body->m_cfg.kDF = 1;
	body->m_cfg.collisions = btSoftBody::fCollision::CL_SS + btSoftBody::fCollision::CL_RS;
 	body->randomizeConstraints();
	btMatrix3x3	m;
	m.setEulerZYX(rotation.x(),rotation.y(),rotation.z());
	body->transform(btTransform(m,startPos));
	body->scale(scaler);
	body->setTotalMass(150,true);
 	body->generateClusters(1);
	phyWorld->addSoftBody(body);

	initCar(ManualObject, body);

	Ogre::SceneNode* m_CarNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	m_CarNode->attachObject(ManualObject);

}


//base on Bullet demo.
void DPSSoftBodyHelper::createPlayground_1(const btVector3& starPosition)
{
	btScalar	widthf=8;
	btScalar	widthr=9;
	btScalar	length=8;
	btScalar	height=4;
	btVector3	wheels[]=	{
		btVector3(+widthf,-height,+length),	// Front left
		btVector3(-widthf,-height,+length),	// Front right
		btVector3(+widthr,-height,-length),	// Rear left
		btVector3(-widthr,-height,-length),	// Rear right
	};

	btQuaternion orientation(-SIMD_PI/2,0,0);

	createCarWheels(m_playgroundManualObject_fl, m_playgroundBody_fl, wheels[0], btVector3(0,0,SIMD_HALF_PI), btVector3(2,4,2));
	createCarWheels(m_playgroundManualObject_fr, m_playgroundBody_fr, wheels[1], btVector3(0,0,SIMD_HALF_PI), btVector3(2,4,2));
	createCarWheels(m_playgroundManualObject_rl, m_playgroundBody_rl, wheels[2], btVector3(0,0,SIMD_HALF_PI), btVector3(2,4,2));
	createCarWheels(m_playgroundManualObject_rr, m_playgroundBody_rr, wheels[3], btVector3(0,0,SIMD_HALF_PI), btVector3(2,4,2));
	createCarBody(m_playgroundManualObject_carBody, m_playgroundBody_carBody, btVector3(0,0,0), btVector3(0,0,0), btVector3(8,8,8));

	btSoftBody::LJoint::Specs lspecs;
	lspecs.cfm = 1;
	lspecs.erp = 1;
	lspecs.position	= btVector3(0,0,0);

	lspecs.position = wheels[0];
	m_playgroundBody_carBody->appendLinearJoint(lspecs,m_playgroundBody_fl);

	lspecs.position = wheels[1];
	m_playgroundBody_carBody->appendLinearJoint(lspecs,m_playgroundBody_fr);

	lspecs.position = wheels[2];
	m_playgroundBody_carBody->appendLinearJoint(lspecs,m_playgroundBody_rl);

	lspecs.position = wheels[3];
	m_playgroundBody_carBody->appendLinearJoint(lspecs,m_playgroundBody_rr);

	btSoftBody::AJoint::Specs aspecs;
	aspecs.cfm = 1;
	aspecs.erp = 1;
	aspecs.axis = btVector3(1,0,0);

	//aspecs.icontrol	= &steercontrol_f;
	m_playgroundBody_carBody->appendAngularJoint(aspecs,m_playgroundBody_fl);
	m_playgroundBody_carBody->appendAngularJoint(aspecs,m_playgroundBody_fr);

	//aspecs.icontrol	= &motorcontrol;
	m_playgroundBody_carBody->appendAngularJoint(aspecs,m_playgroundBody_rl);
	m_playgroundBody_carBody->appendAngularJoint(aspecs,m_playgroundBody_rr);

	m_playgroundBody_carBody->translate(starPosition);
	m_playgroundBody_fl->translate(starPosition);
	m_playgroundBody_fr->translate(starPosition);
	m_playgroundBody_rl->translate(starPosition);
	m_playgroundBody_rr->translate(starPosition);

	m_playgroundBody_carBody->rotate(orientation);
	m_playgroundBody_fl->rotate(orientation);
	m_playgroundBody_fr->rotate(orientation);
	m_playgroundBody_rl->rotate(orientation);
	m_playgroundBody_rr->rotate(orientation);

	m_playgroundBody_fl->m_cfg.piterations	= 1;
	m_playgroundBody_fr->m_cfg.piterations	= 1;
	m_playgroundBody_rl->m_cfg.piterations	= 1;
	m_playgroundBody_rr->m_cfg.piterations	= 1;

	m_playgroundBody_fl->m_clusters[0]->m_matching	= 0.05; 
	m_playgroundBody_fr->m_clusters[0]->m_matching	= 0.05;
	m_playgroundBody_rl->m_clusters[0]->m_matching	= 0.05;
	m_playgroundBody_rr->m_clusters[0]->m_matching	= 0.05;

	m_playgroundBody_fl->m_clusters[0]->m_ndamping	= 0.05;
	m_playgroundBody_fr->m_clusters[0]->m_ndamping	= 0.05;
	m_playgroundBody_rl->m_clusters[0]->m_ndamping	= 0.05;
	m_playgroundBody_rr->m_clusters[0]->m_ndamping	= 0.05;
}


void DPSSoftBodyHelper::createPlayground_2(void)
{
	createTorus(m_playgroundManualObject_2_1, m_playgroundBody_2_1, btVector3(0,3,0), btVector3(0,0,0), btVector3(2,2,2));
 	createTorus(m_playgroundManualObject_2_2, m_playgroundBody_2_2, btVector3(0,7,0), btVector3(0,0,0), btVector3(2,2,2));
// 	createTorus(m_playgroundManualObject_2_3, m_playgroundBody_2_3, btVector3(0,11,0), btVector3(0,0,0), btVector3(2,2,2));
}


void DPSSoftBodyHelper::createPlayground_3(const btVector3& starPosition)
{
	triangles.clear();
	indicies.clear();
	texCoord.clear();

	Objloader* obj = new Objloader;
	obj->LoadModel("car",&triangles,&indicies,&texCoord);

	m_playgroundBody_3 = btSoftBodyHelpers::CreateFromTriMesh(phyWorld->getWorldInfo(),&(triangles[0]),&(indicies[0]),indicies.size()/3,true);
	m_playgroundBody_3->setTotalMass(50.0,true);
	m_playgroundBody_3->m_cfg.kSRHR_CL=1.0;	
	m_playgroundBody_3->m_cfg.viterations=500;
	m_playgroundBody_3->m_cfg.piterations=500;
	m_playgroundBody_3->m_cfg.citerations=500;
	m_playgroundBody_3->m_cfg.diterations=500;
	m_playgroundBody_3->m_cfg.kPR=500;
	m_playgroundBody_3->m_cfg.collisions	= btSoftBody::fCollision::SDF_RS + btSoftBody::fCollision::CL_SS + btSoftBody::fCollision::CL_SELF;
	m_playgroundBody_3->translate(starPosition);
	m_playgroundBody_3->scale(btVector3(1,1,1));
	phyWorld->addSoftBody(m_playgroundBody_3);

	initSoftBody(m_playgroundManualObject_3, m_playgroundBody_3);

	Ogre::SceneNode* m_PlaygroundNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	m_PlaygroundNode->attachObject(m_playgroundManualObject_3);
}


void DPSSoftBodyHelper::createPlayground_4(const btVector3& starPosition)
{
	triangles.clear();
	indicies.clear();
	texCoord.clear();

	Objloader* obj = new Objloader;
	obj->LoadModel("cylinder_high",&triangles,&indicies,&texCoord);

	m_playgroundBody_4 = btSoftBodyHelpers::CreateFromTriMesh(phyWorld->getWorldInfo(),&(triangles[0]),&(indicies[0]),indicies.size()/3,true);
	m_playgroundBody_4->setTotalMass(200.0,true);
	m_playgroundBody_4->m_cfg.kSRHR_CL=1.0;	
	m_playgroundBody_4->m_cfg.viterations=500;
	m_playgroundBody_4->m_cfg.piterations=500;
	m_playgroundBody_4->m_cfg.citerations=500;
	m_playgroundBody_4->m_cfg.diterations=500;
	m_playgroundBody_4->m_cfg.kPR=500;
	m_playgroundBody_4->m_cfg.collisions	= btSoftBody::fCollision::SDF_RS + btSoftBody::fCollision::CL_SS + btSoftBody::fCollision::CL_SELF;
	m_playgroundBody_4->translate(starPosition);
	m_playgroundBody_4->scale(btVector3(1,1,1));
	phyWorld->addSoftBody(m_playgroundBody_4);

	initSoftBody(m_playgroundManualObject_4, m_playgroundBody_4);

	Ogre::SceneNode* m_PlaygroundNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	m_PlaygroundNode->attachObject(m_playgroundManualObject_4);
}


btScalar DPSSoftBodyHelper::UnitRand()
{
	return (rand()/(btScalar)RAND_MAX);
}


btScalar DPSSoftBodyHelper::SignedUnitRand()
{
	return (UnitRand()*2-1);
}


btVector3 DPSSoftBodyHelper::Vector3Rand()
{
	const btVector3	p=btVector3(SignedUnitRand(),SignedUnitRand(),SignedUnitRand());
	return (p.normalized());
}



void DPSSoftBodyHelper::createGimpactBarrel(void)
{
	btTransform tr;
	tr.setIdentity();
	tr.setOrigin(btVector3(0,0,0));

	btTriangleIndexVertexArray * mesh_interface = new btTriangleIndexVertexArray(
		faces_size,
		barrel_ia,
		3*sizeof(int),
		vertices_size,
		(btScalar *) &barrel_va[0][0],
		sizeof(btVector3));

	btGImpactMeshShape * shape = new btGImpactMeshShape(mesh_interface);
	shape->setMargin(btScalar(0.1));
	shape->updateBound();
	m_collisionShapes.push_back(shape);
	/*	btRigidBody * body = localCreateRigidBody(btScalar(20.0),tr,shape);*/
	btVector3 localInertia(0,0,0);
	//btScalar m_defaultContactProcessingThreshold;
	shape->calculateLocalInertia(btScalar(20.0),localInertia);
	btDefaultMotionState* myMotionState = new btDefaultMotionState(tr);
	btRigidBody::btRigidBodyConstructionInfo cInfo(btScalar(20.0),myMotionState,shape,localInertia);
	btRigidBody* m_BuunyBody_3 = new btRigidBody(cInfo);
	m_BuunyBody_3->setContactProcessingThreshold(btScalar(BT_LARGE_FLOAT));
	//body->translate(btVector3(0,50,0));
	phyWorld->addRigidBody(m_BuunyBody_3);

// 	m_ManualObject = mSceneMgr->createManualObject("barrel");
// 	m_ManualObject->setDynamic(true);
// 
// 	//m_ManualObject->estimateVertexCount(BUNNY_NUM_VERTICES);
// 	//m_ManualObject->estimateIndexCount(BUNNY_NUM_INDICES);
// 
// 	m_ManualObject->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);
// 
// 	for (int i = 0; i < 83; i++)
// 	{
// 		btVector3 face = barrel_va[i];
// 
// 		m_ManualObject->position(face.x(),face.y(),face.z());
// 
// 		// 		m_ManualObject->normal(Ogre::Vector3(gIndicesBunny[i][0],gIndicesBunny[i][1],gIndicesBunny[i][2]));
// 		// 		m_ManualObject->normal(Ogre::Vector3(gIndicesBunny[i+1][0],gIndicesBunny[i+1][1],gIndicesBunny[i+1][2]));
// 		// 		m_ManualObject->normal(Ogre::Vector3(gIndicesBunny[i+2][0],gIndicesBunny[i+2][1],gIndicesBunny[i+2][2]));
// 
// 		//m_ManualObject->position(node0->m_x[0], node0->m_x[1], node0->m_x[2]);
// 		//m_ManualObject->textureCoord(1,0);
// 		//m_ManualObject->normal(node0->m_n[0], node0->m_n[1], node0->m_n[2]);
// 
// 		//m_ManualObject->position(node1->m_x[0], node1->m_x[1], node1->m_x[2]);
// 		//m_ManualObject->textureCoord(0,1);
// 		//m_ManualObject->normal(node1->m_n[0], node1->m_n[1], node1->m_n[2]);
// 
// 		//m_ManualObject->position(node2->m_x[0], node2->m_x[1], node2->m_x[2]);
// 		//m_ManualObject->textureCoord(1,1);
// 		//m_ManualObject->normal(node2->m_n[0], node2->m_n[1], node2->m_n[2]);
// 		// 
// 		// 		m_ManualObject->index(i*3);
// 		// 		m_ManualObject->index(i*3+1);
// 		// 		m_ManualObject->index(i*3+2);
// 	}
// 	// 	m_ManualObject->textureCoord(1,0);
// 	// 	m_ManualObject->textureCoord(0,0);
// 	// 	m_ManualObject->textureCoord(0,1);
// 	// 	m_ManualObject->textureCoord(1,1);
// 	m_ManualObject->end();
// 
// 	Ogre::SceneNode* mLiquidBodyNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
// 	mLiquidBodyNode->attachObject(m_ManualObject);

	//initSoftBody(m_BuunyManualObject_3, m_BuunyBody_3);
// 
// 	Ogre::SceneNode* m_deformNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
// 	m_deformNode->attachObject(m_BuunyManualObject_3);


}


void DPSSoftBodyHelper::createGimpactBuuny(void)
{
	btTransform tr;
	tr.setIdentity();
	tr.setOrigin(btVector3(0,0,0));

	btTriangleIndexVertexArray * mesh_interface = new btTriangleIndexVertexArray(
		BUNNY_NUM_TRIANGLES,
		&gIndicesBunny[0][0],
		3*sizeof(int),
		BUNNY_NUM_VERTICES,
		(btScalar *) &gVerticesBunny[0],
		sizeof(btScalar)*3);

	btGImpactMeshShape * shape = new btGImpactMeshShape(mesh_interface);
	shape->setMargin(btScalar(0.1));
	shape->setLocalScaling(btVector3(3,3,3));
	shape->updateBound();
	m_collisionShapes.push_back(shape);
	/*	btRigidBody * body = localCreateRigidBody(btScalar(20.0),tr,shape);*/
	btVector3 localInertia(0,0,0);
	//btScalar m_defaultContactProcessingThreshold;
	shape->calculateLocalInertia(btScalar(20.0),localInertia);
	btDefaultMotionState* myMotionState = new btDefaultMotionState(tr);
	btRigidBody::btRigidBodyConstructionInfo cInfo(btScalar(20.0),myMotionState,shape,localInertia);
	btRigidBody* body = new btRigidBody(cInfo);
	body->setContactProcessingThreshold(btScalar(BT_LARGE_FLOAT));
	body->translate(btVector3(0,10,0));
	phyWorld->addRigidBody(body);

	m_BuunyManualObject = mSceneMgr->createManualObject("bunny");
	m_BuunyManualObject->setDynamic(true);

	//m_ManualObject->estimateVertexCount(BUNNY_NUM_VERTICES);
	//m_ManualObject->estimateIndexCount(BUNNY_NUM_INDICES);

	m_BuunyManualObject->begin("softbody", Ogre::RenderOperation::OT_TRIANGLE_LIST);

	for (int i = 0; i < 1358; i+=9)
	{
		//node0 = faces[i].m_n[0];
		//node1 = faces[i].m_n[1];
		//node2 = faces[i].m_n[2];

		m_BuunyManualObject->position(Ogre::Vector3(gVerticesBunny[i],gVerticesBunny[i+1],gVerticesBunny[i+2]));
		m_BuunyManualObject->position(Ogre::Vector3(gVerticesBunny[i+3],gVerticesBunny[i+4],gVerticesBunny[i+5]));
		m_BuunyManualObject->position(Ogre::Vector3(gVerticesBunny[i+6],gVerticesBunny[i+7],gVerticesBunny[i+8]));


		m_BuunyManualObject->normal(Ogre::Vector3(gIndicesBunny[i][0],gIndicesBunny[i][1],gIndicesBunny[i][2]));
		m_BuunyManualObject->normal(Ogre::Vector3(gIndicesBunny[i+1][0],gIndicesBunny[i+1][1],gIndicesBunny[i+1][2]));
		m_BuunyManualObject->normal(Ogre::Vector3(gIndicesBunny[i+2][0],gIndicesBunny[i+2][1],gIndicesBunny[i+2][2]));

		//m_ManualObject->position(node0->m_x[0], node0->m_x[1], node0->m_x[2]);
		//m_ManualObject->textureCoord(1,0);
		//m_ManualObject->normal(node0->m_n[0], node0->m_n[1], node0->m_n[2]);

		//m_ManualObject->position(node1->m_x[0], node1->m_x[1], node1->m_x[2]);
		//m_ManualObject->textureCoord(0,1);
		//m_ManualObject->normal(node1->m_n[0], node1->m_n[1], node1->m_n[2]);

		//m_ManualObject->position(node2->m_x[0], node2->m_x[1], node2->m_x[2]);
		//m_ManualObject->textureCoord(1,1);
		//m_ManualObject->normal(node2->m_n[0], node2->m_n[1], node2->m_n[2]);

		m_BuunyManualObject->index(i*3);
		m_BuunyManualObject->index(i*3+1);
		m_BuunyManualObject->index(i*3+2);
	}
	// 	m_ManualObject->textureCoord(1,0);
	// 	m_ManualObject->textureCoord(0,0);
	// 	m_ManualObject->textureCoord(0,1);
	// 	m_ManualObject->textureCoord(1,1);
	m_BuunyManualObject->end();

	Ogre::SceneNode* m_BuunyManualNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	m_BuunyManualNode->attachObject(m_BuunyManualObject);
}


void DPSSoftBodyHelper::initSoftBody(Ogre::ManualObject*& m_ManualObject, btSoftBody* body)
{
		//manual objects are used to generate new meshes based on raw vertex data
	//this is used for the liquid form
	m_ManualObject = mSceneMgr->createManualObject();

	/*
		The following code needs to be run once to setup the vertex buffer with data based on
		the bullet soft body information.
	*/
	btSoftBody::tNodeArray& nodes(body->m_nodes);
	btSoftBody::tFaceArray& faces(body->m_faces);

	m_ManualObject->estimateVertexCount(faces.size()*3);
	m_ManualObject->estimateIndexCount(faces.size()*3);

	//http://www.ogre3d.org/tikiwiki/ManualObject
	//m_ManualObject->begin("FlatVertexColour", Ogre::RenderOperation::OT_TRIANGLE_LIST);
	m_ManualObject->begin("softbody", Ogre::RenderOperation::OT_TRIANGLE_LIST);
	for (int i = 0; i < faces.size(); ++i)
	{
		btSoftBody::Node *node0 = 0, *node1 = 0, *node2 = 0;
		node0 = faces[i].m_n[0];
		node1 = faces[i].m_n[1];
		node2 = faces[i].m_n[2];

		m_ManualObject->position(node0->m_x[0], node0->m_x[1], node0->m_x[2]);
		m_ManualObject->colour(Ogre::ColourValue(0.3021f,0.3308f,0.3671f,1.0f));
		m_ManualObject->normal(node0->m_n[0], node0->m_n[1], node0->m_n[2]);

		m_ManualObject->position(node1->m_x[0], node1->m_x[1], node1->m_x[2]);
		m_ManualObject->colour(Ogre::ColourValue(0.3021f,0.3308f,0.3671f,1.0f));
		m_ManualObject->normal(node1->m_n[0], node1->m_n[1], node1->m_n[2]);

		m_ManualObject->position(node2->m_x[0], node2->m_x[1], node2->m_x[2]);
		m_ManualObject->colour(Ogre::ColourValue(0.3021f,0.3308f,0.3671f,1.0f));
		m_ManualObject->normal(node2->m_n[0], node2->m_n[1], node2->m_n[2]);



		m_ManualObject->index(i*3);
		m_ManualObject->index(i*3+1);
		m_ManualObject->index(i*3+2);
	}
	m_ManualObject->end();

	m_ManualObject->setDynamic(true);
	m_ManualObject->setCastShadows(true);
}


void DPSSoftBodyHelper::updateSoftBody(Ogre::ManualObject*& m_ManualObject, btSoftBody* body)
{
	//grab the calculated mesh data from the physics body
	btSoftBody::tNodeArray& nodes(body->m_nodes);
	btSoftBody::tFaceArray& faces(body->m_faces);

	float minx, miny, minz, maxx, maxy, maxz;
	minx = maxx = nodes[0].m_x[0];
	miny = maxy = nodes[0].m_x[1];
	minz = maxz = nodes[0].m_x[2];

	m_ManualObject->beginUpdate(0);
	for (int i = 0; i < faces.size(); i++)
	{
		btSoftBody::Node *node0 = 0, *node1 = 0, *node2 = 0;
		node0 = faces[i].m_n[0];
		node1 = faces[i].m_n[1];
		node2 = faces[i].m_n[2];

		m_ManualObject->position(node0->m_x[0], node0->m_x[1], node0->m_x[2]);
		m_ManualObject->normal(node0->m_n[0], node0->m_n[1], node0->m_n[2]);

		m_ManualObject->position(node1->m_x[0], node1->m_x[1], node1->m_x[2]);
		m_ManualObject->normal(node1->m_n[0], node1->m_n[1], node1->m_n[2]);

		m_ManualObject->position(node2->m_x[0], node2->m_x[1], node2->m_x[2]);
		m_ManualObject->normal(node2->m_n[0], node2->m_n[1], node2->m_n[2]);
		m_ManualObject->index(i*3);
		m_ManualObject->index(i*3+1);
		m_ManualObject->index(i*3+2);
	}
	m_ManualObject->end();
}

void DPSSoftBodyHelper::initCar(Ogre::ManualObject*& m_ManualObject, btSoftBody*& body)
{
			//manual objects are used to generate new meshes based on raw vertex data
	//this is used for the liquid form
	m_ManualObject = mSceneMgr->createManualObject();

	/*
		The following code needs to be run once to setup the vertex buffer with data based on
		the bullet soft body information.
	*/
	btSoftBody::tNodeArray& nodes(body->m_nodes);
	btSoftBody::tFaceArray& faces(body->m_faces);

	m_ManualObject->estimateVertexCount(faces.size()*3);
	m_ManualObject->estimateIndexCount(faces.size()*3);

	//http://www.ogre3d.org/tikiwiki/ManualObject
	//m_ManualObject->begin("FlatVertexColour", Ogre::RenderOperation::OT_TRIANGLE_LIST);
	m_ManualObject->begin("softbody", Ogre::RenderOperation::OT_TRIANGLE_LIST);
	for (int i = 0; i < faces.size(); ++i)
	{
		btSoftBody::Node *node0 = 0, *node1 = 0, *node2 = 0;
		node0 = faces[i].m_n[0];
		node1 = faces[i].m_n[1];
		node2 = faces[i].m_n[2];

		m_ManualObject->position(node0->m_x[0], node0->m_x[1], node0->m_x[2]);
		m_ManualObject->colour(Ogre::ColourValue(0.3021f,0.3308f,0.3671f,1.0f));
		m_ManualObject->normal(node0->m_n[0], node0->m_n[1], node0->m_n[2]);

		m_ManualObject->position(node1->m_x[0], node1->m_x[1], node1->m_x[2]);
		m_ManualObject->colour(Ogre::ColourValue(0.3021f,0.3308f,0.3671f,1.0f));
		m_ManualObject->normal(node1->m_n[0], node1->m_n[1], node1->m_n[2]);

		m_ManualObject->position(node2->m_x[0], node2->m_x[1], node2->m_x[2]);
		m_ManualObject->colour(Ogre::ColourValue(0.3021f,0.3308f,0.3671f,1.0f));
		m_ManualObject->normal(node2->m_n[0], node2->m_n[1], node2->m_n[2]);



		m_ManualObject->index(i*3);
		m_ManualObject->index(i*3+1);
		m_ManualObject->index(i*3+2);
	}
	m_ManualObject->end();

	m_ManualObject->setDynamic(true);
	m_ManualObject->setCastShadows(true);
}

void DPSSoftBodyHelper::updateCar(Ogre::ManualObject*& m_ManualObject, btSoftBody*& body)
{
	//grab the calculated mesh data from the physics body
	btSoftBody::tNodeArray& nodes(body->m_nodes);
	btSoftBody::tFaceArray& faces(body->m_faces);

	float minx, miny, minz, maxx, maxy, maxz;
	minx = maxx = nodes[0].m_x[0];
	miny = maxy = nodes[0].m_x[1];
	minz = maxz = nodes[0].m_x[2];

	m_ManualObject->beginUpdate(0);
	for (int i = 0; i < faces.size(); i++)
	{
		btSoftBody::Node *node0 = 0, *node1 = 0, *node2 = 0;
		node0 = faces[i].m_n[0];
		node1 = faces[i].m_n[1];
		node2 = faces[i].m_n[2];

		m_ManualObject->position(node0->m_x[0], node0->m_x[1], node0->m_x[2]);
		m_ManualObject->normal(node0->m_n[0], node0->m_n[1], node0->m_n[2]);

		m_ManualObject->position(node1->m_x[0], node1->m_x[1], node1->m_x[2]);
		m_ManualObject->normal(node1->m_n[0], node1->m_n[1], node1->m_n[2]);

		m_ManualObject->position(node2->m_x[0], node2->m_x[1], node2->m_x[2]);
		m_ManualObject->normal(node2->m_n[0], node2->m_n[1], node2->m_n[2]);
		m_ManualObject->index(i*3);
		m_ManualObject->index(i*3+1);
		m_ManualObject->index(i*3+2);
	}
	m_ManualObject->end();
}




 