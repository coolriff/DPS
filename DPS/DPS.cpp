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
	Globals::phyWorld->setGravity(btVector3(0,-9.8,0));
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
	//Some normal stuff.
	mSceneMgr->setAmbientLight(ColourValue(1,1,1));
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

	mNinjaEntity = mSceneMgr->createEntity("ninjaEntity", "ogrehead.mesh");
	mNinjaNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("ninjaSceneNode", pos, rot);
	mNinjaNode->attachObject(mNinjaEntity);
	mNinjaNode->setScale(Vector3(1,1,1));

	//Create shape.
	BtOgre::StaticMeshToShapeConverter converter(mNinjaEntity);
	//BtOgre::AnimatedMeshToShapeConverter converter(mNinjaEntity);

	//mNinjaShape = converter.createTrimesh();
	mNinjaShape = converter.createConvex();
	//mNinjaShape = converter.createConvex();

	//Calculate inertia.
	btScalar mass = 5;
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
}

void DPS::createFrameListener(void)
{
	mFrameListener = new BtOgreTestFrameListener(mWindow, mCamera, mSceneMgr);
	mRoot->addFrameListener(mFrameListener);
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