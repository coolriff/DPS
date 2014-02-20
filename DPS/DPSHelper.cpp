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
	Ogre::MeshManager::getSingleton().createPlane("ground", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        plane, 1500, 1500, 20, 20, true, 1, 5, 5, Ogre::Vector3::UNIT_Z);
	Ogre::Entity* entGround = mSceneMgr->createEntity("GroundEntity", "ground");
    mSceneMgr->getRootSceneNode()->createChildSceneNode()->attachObject(entGround);
	entGround ->setMaterialName("Examples/BumpyMetal");
	entGround ->setCastShadows(false);

	//MeshManager::getSingleton().createPlane("groundPlane", "General", Plane(Vector3::UNIT_Y, 0), 100, 100, 
	//10, 10, true, 1, 5, 5, Vector3::UNIT_Z);
	//mGroundEntity = mSceneMgr->createEntity("groundEntity", "TestLevel_b0.mesh");
	//mGroundEntity->setMaterialName("Examples/Rockwall");
	//mSceneMgr->getRootSceneNode()->createChildSceneNode("groundNode")->attachObject(mGroundEntity);

	//Create the ground shape.
	BtOgre::StaticMeshToShapeConverter converter2(entGround);
	btBvhTriangleMeshShape* GroundShape = converter2.createTrimesh();

	//Create MotionState (no need for BtOgre here, you can use it if you want to though).
	btDefaultMotionState* groundState = new btDefaultMotionState(
	btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0)));

	//Create the Body.
	btRigidBody* GroundBody = new btRigidBody(0, groundState, GroundShape, btVector3(0,0,0));
	phyWorld->addRigidBody(GroundBody);
}

void DPSHelper::throwSphere(void)
{
	Ogre::Vector3 pos = mCamera->getDerivedPosition() + mCamera->getDerivedDirection().normalisedCopy() * 10;
	Ogre::Quaternion rot = Ogre::Quaternion::IDENTITY;
	Ogre::Entity *ent = mSceneMgr->createEntity("sphere.mesh");
	//Ogre::Entity *ent = mSceneMgr->createEntity("cube.mesh");
	ent->setCastShadows(true);
	//ent->setMaterialName("Examples/BumpyMetal");

	Ogre::SceneNode* sphereNode = mSceneMgr->getRootSceneNode()->createChildSceneNode(pos,rot);
	setColor(ent, Ogre::Vector3(0.3021f,0.3308f,0.3671f));
	sphereNode->attachObject(ent);
	sphereNode->setScale(Ogre::Vector3(0.1f,0.1f,0.1f));

	BtOgre::StaticMeshToShapeConverter converter(ent);
	btCollisionShape* entShape = converter.createSphere();
	//btCollisionShape* entShape = converter.createTrimesh();

	//Calculate inertia.
	btScalar mass = 1;
	btVector3 inertia(0,0,0);
	entShape->calculateLocalInertia(mass, inertia);

	//Create BtOgre MotionState (connects Ogre and Bullet).
	BtOgre::RigidBodyState* entState = new BtOgre::RigidBodyState(sphereNode);

	//Create the Body.
	btRigidBody* entBody = new btRigidBody(mass, entState, entShape, inertia);
	Ogre::Vector3 thro = mCamera->getRealDirection() * 800;
	entBody->applyCentralForce(btVector3(pos.x,pos.y,pos.z) * 50000);
	entBody->setLinearVelocity(btVector3(thro.x,thro.y,thro.z));
	phyWorld->addRigidBody(entBody);
}

void DPSHelper::createOgreHead(void)
{
	Ogre::Vector3 pos = Ogre::Vector3(0,100,0);
	Ogre::Quaternion rot = Ogre::Quaternion::IDENTITY;

	//Create Ogre stuff.
	Ogre::Entity* ogreHeadEntity = mSceneMgr->createEntity("ogrehead.mesh");
	Ogre::SceneNode* ogreHeadNode = mSceneMgr->getRootSceneNode()->createChildSceneNode(pos, rot);
	ogreHeadNode->attachObject(ogreHeadEntity);
	ogreHeadNode->setScale(Ogre::Vector3(1,1,1));
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
	btVector3 inertia;
	ogreHeadShape->calculateLocalInertia(mass, inertia);

	//Create BtOgre MotionState (connects Ogre and Bullet).
	BtOgre::RigidBodyState *ogreheadState = new BtOgre::RigidBodyState(ogreHeadNode);

	//Create the Body.
	btRigidBody* ogreHeadBody = new btRigidBody(mass, ogreheadState, ogreHeadShape, inertia);
	phyWorld->addRigidBody(ogreHeadBody);
}