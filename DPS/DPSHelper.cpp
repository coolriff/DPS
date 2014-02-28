#include "DPSHelper.h"

using namespace std;

DPSHelper::DPSHelper(btDynamicsWorld* phyWorld, Ogre::Camera* mCamera, Ogre::SceneManager* mSceneMgr)
{
	this->phyWorld = phyWorld;
	this->mCamera = mCamera;
	this->mSceneMgr = mSceneMgr;
}


DPSHelper::~DPSHelper(void)
{
}


void DPSHelper::setColor(Ogre::Entity* ent ,Ogre::Vector3 v)
{
	Ogre::MaterialPtr m_pMat = ent->getSubEntity(0)->getMaterial()->clone("newMat");
	m_pMat->setAmbient(v.x,v.y,v.z);
	m_pMat->setDiffuse(v.x,v.y,v.z, 1);
	ent->getSubEntity(0)->getMaterial()->getTechnique(0)->getPass(0)->setSpecular(0.072f,0.072f,0.072f, 1);
	ent->setMaterial(m_pMat);
}


void DPSHelper::createPointLight(string LightName, Ogre::Vector3 position ,Ogre::Vector3 direction)
{	
	Ogre::Light* pLight = mSceneMgr->createLight(LightName);
	pLight->setType(Ogre::Light::LT_POINT);
	pLight->setPosition(position);
	pLight->setDirection(direction);
	pLight->setDiffuseColour(1.0, 1.0, 1.0);
	pLight->setSpecularColour(0.0, 0.0, 1.0);
}


void DPSHelper::createDirectionLight(string LightName, Ogre::Vector3 position ,Ogre::Vector3 direction)
{
	Ogre::Light* pLight = mSceneMgr->createLight(LightName);
	pLight->setType(Ogre::Light::LT_DIRECTIONAL);
	pLight->setPosition(position);
	pLight->setDirection(direction);
	pLight->setDiffuseColour(1.0, 1.0, 1.0);
	pLight->setSpecularColour(0.0, 0.0, 1.0);
}


void DPSHelper::createSpotLight(string LightName, Ogre::Vector3 position ,Ogre::Vector3 direction)
{
	Ogre::Light* pLight = mSceneMgr->createLight(LightName);
	pLight->setType(Ogre::Light::LT_SPOTLIGHT);
	pLight->setPosition(position);
	pLight->setDirection(direction);
	pLight->setDiffuseColour(1.0, 1.0, 1.0);
	pLight->setSpecularColour(0.0, 0.0, 1.0);
}

void DPSHelper::createGround(void)
{
	//Create Ogre stuff.
	Ogre::Plane plane(Ogre::Vector3::UNIT_Y, 0);
	// 10 10 cube,  500 500 tex
	Ogre::MeshManager::getSingleton().createPlane("ground", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
		plane, 1500, 1500, 50, 50, true, 1, 50, 50, Ogre::Vector3::UNIT_Z);
	Ogre::Entity* entGround = mSceneMgr->createEntity("GroundEntity", "ground");

	entGround ->setMaterialName("Examples/Rockwall");
	entGround ->setCastShadows(false);

	btCollisionShape* shape = new btBoxShape (btVector3(500,1,500));

	//btCollisionShape* shape = new btStaticPlaneShape(btVector3(0, 1, 0), 1);

	btDefaultMotionState* motionState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1), btVector3(0,-1,0)));
	btRigidBody::btRigidBodyConstructionInfo rigidBodyCI(
		0,                  // mass
		motionState,        // initial position
		shape,              // collision shape of body
		btVector3(0,0,0)    // local inertia
		);
	btRigidBody *GroundBody = new btRigidBody(rigidBodyCI);
	phyWorld->addRigidBody(GroundBody);

	mSceneMgr->getRootSceneNode()->createChildSceneNode()->attachObject(entGround);
}


void DPSHelper::throwSphere(void)
{
	Ogre::Vector3 pos = mCamera->getDerivedPosition();
	Ogre::Quaternion rot = Ogre::Quaternion::IDENTITY;
	Ogre::Entity *ent = mSceneMgr->createEntity("defSphere.mesh");
	setColor(ent, Ogre::Vector3(0.3021f,0.3308f,0.3671f));
	//Ogre::Entity *ent = mSceneMgr->createEntity("cube.mesh");
	ent->setCastShadows(true);
	//ent->setMaterialName("Examples/BumpyMetal");

	Ogre::SceneNode* sphereNode = mSceneMgr->getRootSceneNode()->createChildSceneNode(pos,rot);
	//sphereNode->setScale(Ogre::Vector3(0.01f,0.01f,0.01f));

	btCollisionShape* entShape = new btSphereShape(1);
	//Calculate inertia.
	btScalar mass = 1;
	btVector3 inertia(0,0,0);
	entShape->calculateLocalInertia(mass, inertia);

	//Create BtOgre MotionState (connects Ogre and Bullet).
	BtOgre::RigidBodyState* entState = new BtOgre::RigidBodyState(sphereNode);

	//Create the Body.
	btRigidBody* entBody = new btRigidBody(mass, entState, entShape, inertia);
	Ogre::Vector3 thro = mCamera->getDirection() * 100;
	//entBody->applyCentralForce(btVector3(pos.x,pos.y,pos.z) * 50000);
	entBody->setLinearVelocity(btVector3(thro.x,thro.y,thro.z));
	phyWorld->addRigidBody(entBody);
	
	sphereNode->attachObject(ent);
}


void DPSHelper::throwCube(void)
{
	Ogre::Vector3 pos = mCamera->getDerivedPosition();
	Ogre::Quaternion rot = Ogre::Quaternion::IDENTITY;
	Ogre::Entity *ent = mSceneMgr->createEntity("defCube.mesh");
	setColor(ent, Ogre::Vector3(0.3021f,0.3308f,0.3671f));
	//Ogre::Entity *ent = mSceneMgr->createEntity("cube.mesh");
	ent->setCastShadows(true);
	//ent->setMaterialName("Examples/BumpyMetal");

	Ogre::SceneNode* sphereNode = mSceneMgr->getRootSceneNode()->createChildSceneNode(pos,rot);
	//sphereNode->setScale(Ogre::Vector3(0.01f,0.01f,0.01f));

	btCollisionShape* entShape = new btBoxShape(btVector3(0.5,0.5,0.5));
	//Calculate inertia.
	btScalar mass = 1;
	btVector3 inertia(0,0,0);
	entShape->calculateLocalInertia(mass, inertia);

	//Create BtOgre MotionState (connects Ogre and Bullet).
	BtOgre::RigidBodyState* entState = new BtOgre::RigidBodyState(sphereNode);

	//Create the Body.
	btRigidBody* entBody = new btRigidBody(mass, entState, entShape, inertia);
	Ogre::Vector3 thro = mCamera->getDirection() * 100;
	//entBody->applyCentralForce(btVector3(pos.x,pos.y,pos.z) * 50000);
	entBody->setLinearVelocity(btVector3(thro.x,thro.y,thro.z));
	phyWorld->addRigidBody(entBody);

	sphereNode->attachObject(ent);
}


void DPSHelper::createOgreHead(void)
{
	Ogre::Vector3 pos = Ogre::Vector3(0,100,0);
	Ogre::Quaternion rot = Ogre::Quaternion::IDENTITY;

	//Create Ogre stuff.
	Ogre::Entity* ogreHeadEntity = mSceneMgr->createEntity("ogrehead.mesh");
	Ogre::SceneNode* ogreHeadNode = mSceneMgr->getRootSceneNode()->createChildSceneNode(pos, rot);
	//ogreHeadNode->setScale(Ogre::Vector3(0.1f,0.1f,0.1f));
	ogreHeadEntity->setCastShadows(true);
	//setColor(mNinjaEntity, Ogre::Vector3(0.3021,0.3308,0.3671));
	//mNinjaEntity->setMaterialName("Examples/Rockwall");

	//Create shape.
	BtOgre::StaticMeshToShapeConverter converter(ogreHeadEntity);
	//BtOgre::AnimatedMeshToShapeConverter converter(mNinjaEntity);

	//mNinjaShape = converter.createTrimesh();
	btCollisionShape* ogreHeadShape = converter.createConvex();
	//mNinjaShape = converter.createConvex();

	//Calculate inertia.
	btScalar mass = 1;
	btVector3 inertia(0,0,0);
	ogreHeadShape->calculateLocalInertia(mass, inertia);

	//Create BtOgre MotionState (connects Ogre and Bullet).
	BtOgre::RigidBodyState *ogreheadState = new BtOgre::RigidBodyState(ogreHeadNode);

	//Create the Body.
	btRigidBody* ogreHeadBody = new btRigidBody(mass, ogreheadState, ogreHeadShape, inertia);
	phyWorld->addRigidBody(ogreHeadBody);
	ogreHeadNode->attachObject(ogreHeadEntity);
}