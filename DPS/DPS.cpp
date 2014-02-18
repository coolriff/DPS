#include "DPS.h"

class BtOgreTestFrameListener : public ExampleFrameListener
{
    public:
	BtOgreTestFrameListener(RenderWindow* win, Camera* cam, SceneManager *sceneMgr)
	    : ExampleFrameListener(win, cam, false, false)
	{
	}

	bool frameStarted(const FrameEvent &evt)
	{
	    //Update Bullet world. Don't forget the debugDrawWorld() part!
	    Globals::phyWorld->stepSimulation(evt.timeSinceLastFrame, 10); 
	    Globals::phyWorld->debugDrawWorld();

	    //Shows debug if F3 key down.
	    Globals::dbgdraw->setDebugMode(mKeyboard->isKeyDown(OIS::KC_F3));
	    Globals::dbgdraw->step();

	    return ExampleFrameListener::frameStarted(evt);
	}
};

DPS::DPS(void)
{
	//Bullet initialisation.
	mBroadphase = new btAxisSweep3(btVector3(-10000,-10000,-10000), btVector3(10000,10000,10000), 1024);
	mCollisionConfig = new btDefaultCollisionConfiguration();
	mDispatcher = new btCollisionDispatcher(mCollisionConfig);
	mSolver = new btSequentialImpulseConstraintSolver();

	Globals::phyWorld = new btDiscreteDynamicsWorld(mDispatcher, mBroadphase, mSolver, mCollisionConfig);
	Globals::phyWorld->setGravity(btVector3(0,-100,0));
}


DPS::~DPS(void)
{
	//Free rigid bodies
	Globals::phyWorld->removeRigidBody(mNinjaBody);
	delete mNinjaBody->getMotionState();
	delete mNinjaBody;
	delete mNinjaShape;

	Globals::phyWorld->removeRigidBody(mGroundBody);
	delete mGroundBody->getMotionState();
	delete mGroundBody;
	delete mGroundShape->getMeshInterface();
	delete mGroundShape;

	//Free Bullet stuff.
	delete Globals::dbgdraw;
	delete Globals::phyWorld;

	delete mSolver;
	delete mDispatcher;
	delete mCollisionConfig;
	delete mBroadphase;
}

/*
void DPS::createCamera(void)
{
    // create the camera
    mCamera = mSceneMgr->createCamera("PlayerCam");
    // set its position, direction  
    mCamera->setPosition(Ogre::Vector3(0,10,500));
    mCamera->lookAt(Ogre::Vector3(0,0,0));
    // set the near clip distance
    mCamera->setNearClipDistance(5);
 
    mCameraMan = new OgreBites::SdkCameraMan(mCamera);   // create a default camera controller
}
//-------------------------------------------------------------------------------------
void DPS::createViewports(void)
{
    // Create one viewport, entire window
    Ogre::Viewport* vp = mWindow->addViewport(mCamera);
    vp->setBackgroundColour(Ogre::ColourValue(0,0,0));
    // Alter the camera aspect ratio to match the viewport
    mCamera->setAspectRatio(Ogre::Real(vp->getActualWidth()) / Ogre::Real(vp->getActualHeight()));    
}
*/
void DPS::createScene(void)
{
	Ogre::Light* pLight = mSceneMgr->createLight( "MainLight" );
	pLight->setType( Light::LightTypes::LT_DIRECTIONAL );
	pLight->setPosition( 60, 80, 100 );
	pLight->setDirection( -60, -80, -100 );
	pLight->setDiffuseColour( 1.0, 1.0, 1.0 );
	pLight->setSpecularColour( 0.0, 0.0, 1.0 );

	//Some normal stuff.
	mSceneMgr->setAmbientLight(ColourValue(0.9,0.9,0.9));
	mCamera->setPosition(Vector3(10,10,10));
	mCamera->lookAt(Vector3::ZERO);
	mCamera->setNearClipDistance(0.05);
	LogManager::getSingleton().setLogDetail(LL_BOREME);

	//----------------------------------------------------------
	// Debug drawing!
	//----------------------------------------------------------
	Globals::dbgdraw = new BtOgre::DebugDrawer(mSceneMgr->getRootSceneNode(), Globals::phyWorld);
	Globals::phyWorld->setDebugDrawer(Globals::dbgdraw);

	//----------------------------------------------------------
	// Ninja!
	//----------------------------------------------------------
	Vector3 pos = Vector3(0,100,0);
	Quaternion rot = Quaternion::IDENTITY;

	//Create Ogre stuff.
	mNinjaEntity = mSceneMgr->createEntity("ninjaEntity", "cube.mesh");
	mNinjaNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("ninjaSceneNode", pos, rot);
	mNinjaNode->attachObject(mNinjaEntity);
	mNinjaNode->setScale(Vector3(1,1,1));
	mNinjaEntity->setCastShadows(true);
	setColor(mNinjaEntity, Ogre::Vector3(0.3021,0.3308,0.3671));

	//Create shape.
	BtOgre::StaticMeshToShapeConverter converter(mNinjaEntity);
	//BtOgre::AnimatedMeshToShapeConverter converter(mNinjaEntity);

	//mNinjaShape = converter.createTrimesh();
	mNinjaShape = converter.createConvex();
	//mNinjaShape = converter.createConvex();

	//Calculate inertia.
	btScalar mass = 1;
	btVector3 inertia;
	mNinjaShape->calculateLocalInertia(mass, inertia);

	//Create BtOgre MotionState (connects Ogre and Bullet).
	BtOgre::RigidBodyState *ninjaState = new BtOgre::RigidBodyState(mNinjaNode);

	//Create the Body.
	mNinjaBody = new btRigidBody(mass, ninjaState, mNinjaShape, inertia);
	Globals::phyWorld->addRigidBody(mNinjaBody);

	//----------------------------------------------------------
	// Ground!
	//----------------------------------------------------------


	//Create Ogre stuff.
	Ogre::Plane plane(Ogre::Vector3::UNIT_Y, 0);
	Ogre::MeshManager::getSingleton().createPlane("ground", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
		plane, 1500, 1500, 20, 20, true, 1, 5, 5, Ogre::Vector3::UNIT_Z);
	mGroundEntity = mSceneMgr->createEntity("GroundEntity", "ground");
	mSceneMgr->getRootSceneNode()->createChildSceneNode()->attachObject(mGroundEntity);
	mGroundEntity->setMaterialName("Examples/Rockwall");
	mGroundEntity->setCastShadows(false);

	//MeshManager::getSingleton().createPlane("groundPlane", "General", Plane(Vector3::UNIT_Y, 0), 100, 100, 
	//10, 10, true, 1, 5, 5, Vector3::UNIT_Z);
	//mGroundEntity = mSceneMgr->createEntity("groundEntity", "TestLevel_b0.mesh");
	//mGroundEntity->setMaterialName("Examples/Rockwall");
	//mSceneMgr->getRootSceneNode()->createChildSceneNode("groundNode")->attachObject(mGroundEntity);

	//Create the ground shape.
	BtOgre::StaticMeshToShapeConverter converter2(mGroundEntity);
	mGroundShape = converter2.createTrimesh();

	//Create MotionState (no need for BtOgre here, you can use it if you want to though).
	btDefaultMotionState* groundState = new btDefaultMotionState(
		btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0)));

	//Create the Body.
	mGroundBody = new btRigidBody(0, groundState, mGroundShape, btVector3(0,0,0));
	Globals::phyWorld->addRigidBody(mGroundBody);

	//Ogre::Plane planes;
	//planes.d = 100;
	//planes.normal = Ogre::Vector3::NEGATIVE_UNIT_Y;
	//mSceneMgr->setSkyPlane(true, planes, "Examples/CloudySky", 500, 20, true, 0.5, 150, 150);

	//mSceneMgr->setSkyDome(true, "Examples/CloudySky", 5, 8);
}

void DPS::createFrameListenerBtOgre(void)
{
	mFrameListener = new BtOgreTestFrameListener(mWindow, mCamera, mSceneMgr);
	mRoot->addFrameListener(mFrameListener);
}

void DPS::setColor(Ogre::Entity* ent ,Ogre::Vector3 v)
{
	Ogre::MaterialPtr m_pMat = ent->getSubEntity(0)->getMaterial()->clone("newMat");
	m_pMat->setAmbient(v.x,v.y,v.z);
	m_pMat->setDiffuse(v.x,v.y,v.z, 1);
	ent->getSubEntity(0)->getMaterial()->getTechnique(0)->getPass(0)->setSpecular(0.072,0.072,0.072, 1);
	ent->setMaterial(m_pMat);
}

void DPS::throws(void)
{
	Ogre::Vector3 pos = mCamera->getDerivedPosition() + mCamera->getDerivedDirection().normalisedCopy() * 10;

	Quaternion rot = Quaternion::IDENTITY;
	Ogre::Entity *ent = mSceneMgr->createEntity("sphere.mesh");
	//Ogre::Entity *ent = mSceneMgr->createEntity("cube.mesh");
	ent->setCastShadows(true);
	//ent->setMaterialName("Examples/BumpyMetal");

	Ogre::SceneNode* sphereNode = mSceneMgr->getRootSceneNode()->createChildSceneNode(pos,rot);
	setColor(ent, Ogre::Vector3(0.3021,0.3308,0.3671));
	sphereNode->attachObject(ent);
	sphereNode->setScale(Vector3(0.5,0.5,0.5));

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
	Globals::phyWorld->addRigidBody(entBody);
}

//void DPS::createBoxShape(float width, float height, float depth, Ogre::Entity* entity, Ogre::Vector3 position, bool bStatic)
//{
//	Ogre::SceneNode *node = entity->getParentSceneNode();
//	Ogre::Vector3 size = node->_getDerivedScale()*entity->getBoundingBox().getHalfSize();
//	float mass =  bStatic ? 0.0f : 1.0f;
//	srand( (unsigned)time( NULL ) );
//
//	node->setPosition(position);
//	node->setOrientation(Quaternion(Degree(Ogre::Math::RangeRandom(0.0,60.0)), Vector3::UNIT_Y));
//
//	btBoxShape* sceneBoxShape = new btBoxShape(btVector3(width, height, depth) * 0.50);
//
//	// and the Bullet rigid body
//	MyMotionState * defaultMotionState = new MyMotionState(btTransform(btQuaternion(btScalar(0),btScalar(0),btScalar(0),btScalar(1))), node);
//	btRigidBody *defaultBody = new btRigidBody(btScalar(1), defaultMotionState, sceneBoxShape);
//	/* defaultBody->setShape(node, 
//						sceneBoxShape, 
//						0.6f,                             // dynamic body restitution
//						0.6f,                             // dynamic body friction
//						mass,                             // dynamic bodymass
//						node->_getDerivedPosition(),      // starting position of the box
//						node->_getDerivedOrientation());  // orientation of the box**/
//	mShapes.push_back(sceneBoxShape);
//	mBodies.push_back(defaultBody);
//	mNumEntitiesInstanced++;		
//}

bool DPS::keyPressed(const OIS::KeyEvent &arg)
{
	if (arg.key == OIS::KC_1) 
	{
		throws();
	}
	return BaseApplication::keyPressed(arg);
}

#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
#define WIN32_LEAN_AND_MEAN
#include "windows.h"
#endif
 
#ifdef __cplusplus
	extern "C" {
#endif
 
#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
		INT WINAPI WinMain( HINSTANCE hInst, HINSTANCE, LPSTR strCmdLine, INT )
#else
		int main(int argc, char *argv[])
#endif
		{
			// Create application object
			DPS app;
 
			try {
				app.go();
			} catch( Ogre::Exception& e ) {
#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
				//MessageBox( NULL, e.getFullDescription().c_str(), "An exception has occured!", MB_OK | MB_ICONERROR | MB_TASKMODAL);
#else
				std::cerr << "An exception has occured: " <<
					e.getFullDescription().c_str() << std::endl;
#endif
			}
 
			return 0;
		}
 
#ifdef __cplusplus
	}
#endif