#include "DPSSoftBodyHelper.h"
#include "objloader.h"
#include "Mesh/BunnyMesh.h"
#include "Mesh/TorusMesh.h"
/*#include "Mesh/barrel.h"*/
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


void DPSSoftBodyHelper::createClothDemo_1(void)
{
	float s=4;
	float h=20;
	m_clothBody_1 = btSoftBodyHelpers::CreatePatch(phyWorld->getWorldInfo(),btVector3(-s,h,-s),btVector3(s,h,-s),btVector3(-s,h,s),btVector3(s,h,s),5,5,4+8,true);
	m_clothBody_1->m_cfg.viterations=500;
	m_clothBody_1->m_cfg.piterations=500;
	m_clothBody_1->setTotalMass(3.0);
	//m_cloth->setMass(100,100);
	//m_cloth->setCollisionFlags(m_cloth->getCollisionFlags() | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
	phyWorld->addSoftBody(m_clothBody_1);

	initSoftBody(m_clothManualObject_1, m_clothBody_1);

	Ogre::SceneNode* m_clothNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	m_clothNode->attachObject(m_clothManualObject_1);
}


void DPSSoftBodyHelper::createClothDemo_2(void)
{
	float s=4;
	float h=20;
	m_clothBody_2 = btSoftBodyHelpers::CreatePatch(phyWorld->getWorldInfo(),btVector3(-s,h,-s),btVector3(s,h,-s),btVector3(-s,h,s),btVector3(s,h,s),5,5,1+2+4+8,true);
	m_clothBody_2->m_cfg.viterations=500;
	m_clothBody_2->m_cfg.piterations=500;
	m_clothBody_2->setTotalMass(3.0);
	//m_cloth->setMass(100,100);
	//m_cloth->setCollisionFlags(m_cloth->getCollisionFlags() | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
	phyWorld->addSoftBody(m_clothBody_2);

	initSoftBody(m_clothManualObject_2, m_clothBody_2);

	Ogre::SceneNode* m_clothNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	m_clothNode->attachObject(m_clothManualObject_2);
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

void DPSSoftBodyHelper::initSoftBody(Ogre::ManualObject*& m_ManualObject, btSoftBody* body)
{
	//manual objects are used to generate new meshes based on raw vertex data
	//this is used for the liquid form
	m_ManualObject = mSceneMgr->createManualObject("liquidBody");
	m_ManualObject->setDynamic(true);
	m_ManualObject->setCastShadows(true);

	btSoftBody::tNodeArray& nodes(body->m_nodes);
	btSoftBody::tFaceArray& faces(body->m_faces);

	m_ManualObject->estimateVertexCount(faces.size()*3);
	m_ManualObject->estimateIndexCount(faces.size()*3);

	//m_ManualObject->begin("CharacterMaterials/LiquidBody", Ogre::RenderOperation::OT_TRIANGLE_LIST);
	//m_ManualObject->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);
	//m_ManualObject->begin("FlatVertexColour", Ogre::RenderOperation::OT_TRIANGLE_LIST);
	m_ManualObject->begin("softbody", Ogre::RenderOperation::OT_TRIANGLE_LIST);

	//btSoftBody::Node *node0 = 0, *node1 = 0, *node2 = 0;

	//http://www.ogre3d.org/tikiwiki/ManualObject
	for (int i = 0; i < faces.size(); ++i)
	{
		//node0 = faces[i].m_n[0];
		//node1 = faces[i].m_n[1];
		//node2 = faces[i].m_n[2];

		//problem of rendering texture - DO NOT USE NORMALS IN UPDATE FUNCTION FOR NOW! 
		m_ManualObject->position(body->m_faces[i].m_n[0]->m_x.x(),body->m_faces[i].m_n[0]->m_x.y(),body->m_faces[i].m_n[0]->m_x.z());
		m_ManualObject->colour(Ogre::ColourValue(0.7f,0.7f,0.7f,1.0f));
		//m_ManualObject->colour(0.5f,0.5f,0.5f);
		//m_ManualObject->textureCoord(dpsSoftbodyHelper->texCoord[i],dpsSoftbodyHelper->texCoord[i+1]);
		m_ManualObject->position(body->m_faces[i].m_n[1]->m_x.x(),body->m_faces[i].m_n[1]->m_x.y(),body->m_faces[i].m_n[1]->m_x.z());
		m_ManualObject->colour(Ogre::ColourValue(0.7f,0.7f,0.7f,1.0f));
		//m_ManualObject->colour(0.5f,0.5f,0.5f);
		//m_ManualObject->textureCoord(dpsSoftbodyHelper->texCoord[i+1],dpsSoftbodyHelper->texCoord[i+2]);
		m_ManualObject->position(body->m_faces[i].m_n[2]->m_x.x(),body->m_faces[i].m_n[2]->m_x.y(),body->m_faces[i].m_n[2]->m_x.z());
		m_ManualObject->colour(Ogre::ColourValue(0.7f,0.7f,0.7f,1.0f));
		//m_ManualObject->colour(0.5f,0.5f,0.5f);
		//m_ManualObject->textureCoord(dpsSoftbodyHelper->texCoord[i+2],dpsSoftbodyHelper->texCoord[i+3]);
		// 
		// 		m_ManualObject->position(body->m_faces[i].m_n[0]->m_x[0],body->m_faces[i].m_n[0]->m_x[1],body->m_faces[i].m_n[0]->m_x[2]);
		// 		m_ManualObject->position(body->m_faces[i].m_n[1]->m_x[0],body->m_faces[i].m_n[1]->m_x[1],body->m_faces[i].m_n[1]->m_x[2]);
		// 		m_ManualObject->position(body->m_faces[i].m_n[2]->m_x[0],body->m_faces[i].m_n[2]->m_x[1],body->m_faces[i].m_n[2]->m_x[2]);
		// 
		m_ManualObject->normal(body->m_faces[i].m_n[0]->m_n[0], body->m_faces[i].m_n[0]->m_n[1], body->m_faces[i].m_n[0]->m_n[2]);
		m_ManualObject->normal(body->m_faces[i].m_n[1]->m_n[0], body->m_faces[i].m_n[1]->m_n[1], body->m_faces[i].m_n[1]->m_n[2]);
		m_ManualObject->normal(body->m_faces[i].m_n[2]->m_n[0], body->m_faces[i].m_n[2]->m_n[1], body->m_faces[i].m_n[2]->m_n[2]);

		//m_ManualObject->position(node0->m_x[0], node0->m_x[1], node0->m_x[2]);
		//m_ManualObject->textureCoord(1,0);
		//m_ManualObject->normal(node0->m_n[0], node0->m_n[1], node0->m_n[2]);

		//m_ManualObject->position(node1->m_x[0], node1->m_x[1], node1->m_x[2]);
		//m_ManualObject->textureCoord(0,1);
		//m_ManualObject->normal(node1->m_n[0], node1->m_n[1], node1->m_n[2]);

		//m_ManualObject->position(node2->m_x[0], node2->m_x[1], node2->m_x[2]);
		//m_ManualObject->textureCoord(1,1);
		//m_ManualObject->normal(node2->m_n[0], node2->m_n[1], node2->m_n[2]);

		m_ManualObject->triangle(i*3,i*3+1,i*3+2);
		// 		m_ManualObject->triangle(i*3+1);
		// 		m_ManualObject->triangle(i*3+2);
	}
	// 	m_ManualObject->textureCoord(1,0);
	// 	m_ManualObject->textureCoord(0,0);
	// 	m_ManualObject->textureCoord(0,1);
	// 	m_ManualObject->textureCoord(1,1);
	m_ManualObject->end();
}


void DPSSoftBodyHelper::updateSoftBody(Ogre::ManualObject*& m_ManualObject, btSoftBody* body)
{
	//grab the calculated mesh data from the physics body
	btSoftBody::tNodeArray& nodes(body->m_nodes);
	btSoftBody::tFaceArray& faces(body->m_faces);

	m_ManualObject->beginUpdate(0);
	/*	btSoftBody::Node *node0 = 0, *node1 = 0, *node2 = 0;*/
	for (int i = 0; i < faces.size(); i++)
	{
		m_ManualObject->position(body->m_faces[i].m_n[0]->m_x.x(),body->m_faces[i].m_n[0]->m_x.y(),body->m_faces[i].m_n[0]->m_x.z());
		//m_ManualObject->colour(0.5f,0.5f,0.5f);
		m_ManualObject->position(body->m_faces[i].m_n[1]->m_x.x(),body->m_faces[i].m_n[1]->m_x.y(),body->m_faces[i].m_n[1]->m_x.z());
		//m_ManualObject->colour(0.5f,0.5f,0.5f);
		m_ManualObject->position(body->m_faces[i].m_n[2]->m_x.x(),body->m_faces[i].m_n[2]->m_x.y(),body->m_faces[i].m_n[2]->m_x.z());
		//m_ManualObject->colour(0.5f,0.5f,0.5f);
		// 
		// 		m_ManualObject->textureCoord(body->m_faces[i].m_n[1]->m_x.x(), body->m_faces[i].m_n[1]->m_x.y(), body->m_faces[i].m_n[1]->m_x.z());
		// 		m_ManualObject->textureCoord(body->m_faces[i].m_n[2]->m_x.x(), body->m_faces[i].m_n[2]->m_x.y(), body->m_faces[i].m_n[2]->m_x.z());

		// 		m_ManualObject->position(body->m_faces[i].m_n[0]->m_x[0],body->m_faces[i].m_n[0]->m_x[1],body->m_faces[i].m_n[0]->m_x[2]);
		// 		m_ManualObject->position(body->m_faces[i].m_n[1]->m_x[0],body->m_faces[i].m_n[1]->m_x[1],body->m_faces[i].m_n[1]->m_x[2]);
		// 		m_ManualObject->position(body->m_faces[i].m_n[2]->m_x[0],body->m_faces[i].m_n[2]->m_x[1],body->m_faces[i].m_n[2]->m_x[2]);
		// 
		m_ManualObject->normal(body->m_faces[i].m_n[0]->m_n[0], body->m_faces[i].m_n[0]->m_n[1], body->m_faces[i].m_n[0]->m_n[2]);
		m_ManualObject->normal(body->m_faces[i].m_n[1]->m_n[0], body->m_faces[i].m_n[1]->m_n[1], body->m_faces[i].m_n[1]->m_n[2]);
		m_ManualObject->normal(body->m_faces[i].m_n[2]->m_n[0], body->m_faces[i].m_n[2]->m_n[1], body->m_faces[i].m_n[2]->m_n[2]);

		// 		node0 = faces[i].m_n[0];
		// 		node1 = faces[i].m_n[1];
		// 		node2 = faces[i].m_n[2];

		// 		m_ManualObject->position(node0->m_x[0], node0->m_x[1], node0->m_x[2]);
		// 		m_ManualObject->normal(node0->m_n[0], node0->m_n[1], node0->m_n[2]);
		// 				
		// 		m_ManualObject->position(node1->m_x[0], node1->m_x[1], node1->m_x[2]);
		// 		m_ManualObject->normal(node1->m_n[0], node1->m_n[1], node1->m_n[2]);
		// 				
		// 		m_ManualObject->position(node2->m_x[0], node2->m_x[1], node2->m_x[2]);
		// 		m_ManualObject->normal(node2->m_n[0], node2->m_n[1], node2->m_n[2]);

		m_ManualObject->index(i*3);
		m_ManualObject->index(i*3+1);
		m_ManualObject->index(i*3+2);
	}

	m_ManualObject->end();
}


