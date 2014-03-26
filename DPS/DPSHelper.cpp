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

void DPSHelper::createWorld(void)
{
	//Create Ogre stuff.
	Ogre::Plane plane(Ogre::Vector3::UNIT_Y, 0);
	// 10 10 cube,  500 500 tex
	Ogre::MeshManager::getSingleton().createPlane("ground", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
		plane, 10000, 10000, 200, 200, true, 1, 200, 200, Ogre::Vector3::UNIT_Z);
	Ogre::Entity* entGround = mSceneMgr->createEntity("GroundEntity", "ground");

	entGround ->setMaterialName("Examples/Wood");
	entGround ->setCastShadows(false);

	btCollisionShape* shape = new btBoxShape (btVector3(5000,1,5000));

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

	mSceneMgr->setSkyBox(true, "Examples/MorningSkyBox");

	createDirectionLight("mainLight",Ogre::Vector3(60,180,100),Ogre::Vector3(-60,-80,-100));
	createDirectionLight("mainLight1",Ogre::Vector3(0,50,-3),Ogre::Vector3(0,0,0));
}


btRigidBody* DPSHelper::throwSphere(void)
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

// 	Ogre::Vector3 pos = mCamera->getDerivedPosition();
// 
// 	btTransform transform;
// 	transform.setIdentity();
// 	transform.setOrigin(btVector3(pos.x,pos.y,pos.z));
// 	btDefaultMotionState* motionState=new btDefaultMotionState(transform);
// 	btSphereShape* sphereShape=new btSphereShape(1);
// 	btVector3 inertia(0,0,0);
// 	btScalar mass = 1;
// 	sphereShape->calculateLocalInertia(mass,inertia);
// 	Ogre::Vector3 thro = mCamera->getDirection() * 100;
// 	btRigidBody::btRigidBodyConstructionInfo info(mass,motionState,sphereShape,inertia);
// 	btRigidBody* body=new btRigidBody(info);
// 	body->setLinearVelocity(btVector3(thro.x,thro.y,thro.z));
// 	phyWorld->addRigidBody(body);

	return entBody;
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
	sphereNode->setScale(Ogre::Vector3(2.0f,2.0f,2.0f));

	btCollisionShape* entShape = new btBoxShape(btVector3(1,1,1));
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

void DPSHelper::createLeapMotionSphere_0(std::string fingerName, Ogre::Vector3 position)
{
	Ogre::Quaternion rot = Ogre::Quaternion::IDENTITY;
	Ogre::Entity *ent = mSceneMgr->createEntity("defSphere.mesh");
	setColor(ent, Ogre::Vector3(0.7f,0.0f,0.0f));
	ent->setCastShadows(true);
	//ent->setMaterialName("Examples/BumpyMetal");

	sphereNode_0 = mSceneMgr->getRootSceneNode()->createChildSceneNode(fingerName,position,rot);
	//sphereNode->setScale(Ogre::Vector3(0.01f,0.01f,0.01f));

	btCollisionShape* entShape = new btSphereShape(1);
	//Calculate inertia.
	btScalar mass = 1;
	btVector3 inertia(0,0,0);
	entShape->calculateLocalInertia(mass, inertia);

	//Create BtOgre MotionState (connects Ogre and Bullet).
	BtOgre::RigidBodyState* entState = new BtOgre::RigidBodyState(sphereNode_0);

	//Create the Body.
	sphereBody_0 = new btRigidBody(mass, entState, entShape, inertia);
	sphereBody_0->setCollisionFlags(sphereBody_0->getCollisionFlags());
	phyWorld->addRigidBody(sphereBody_0);

	sphereNode_0->attachObject(ent);
}

void DPSHelper::createLeapMotionSphere_1(std::string fingerName, Ogre::Vector3 position)
{
	Ogre::Quaternion rot = Ogre::Quaternion::IDENTITY;
	Ogre::Entity *ent = mSceneMgr->createEntity("defSphere.mesh");
	setColor(ent, Ogre::Vector3(0.7f,0.7f,0.0f));
	ent->setCastShadows(true);
	//ent->setMaterialName("Examples/BumpyMetal");

	sphereNode_1 = mSceneMgr->getRootSceneNode()->createChildSceneNode(fingerName,position,rot);
	//sphereNode->setScale(Ogre::Vector3(0.01f,0.01f,0.01f));

	btCollisionShape* entShape = new btSphereShape(1);
	//Calculate inertia.
	btScalar mass = 1;
	btVector3 inertia(0,0,0);
	entShape->calculateLocalInertia(mass, inertia);

	//Create BtOgre MotionState (connects Ogre and Bullet).
	BtOgre::RigidBodyState* entState = new BtOgre::RigidBodyState(sphereNode_1);

	//Create the Body.
	sphereBody_1 = new btRigidBody(mass, entState, entShape, inertia);
	sphereBody_1->setCollisionFlags(sphereBody_1->getCollisionFlags());
	phyWorld->addRigidBody(sphereBody_1);

	sphereNode_1->attachObject(ent);
}