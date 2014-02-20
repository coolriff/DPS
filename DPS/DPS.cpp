#include "DPS.h"
//#include <memory>
//using namespace std;

class BtOgreTestFrameListener : public ExampleFrameListener
{
    public:
	BtOgreTestFrameListener(RenderWindow* win, Camera* cam, SceneManager *sceneMgr)
	    : ExampleFrameListener(win, cam, false, false)
	{
	}

	bool frameStarted(const FrameEvent &evt)
	{
	    //Update Bullet world
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
	// Basic Ogre stuff.
	mSceneMgr->setAmbientLight(ColourValue(0.9f,0.9f,0.9f));
	mCamera->setPosition(Vector3(0,100,100));
	mCamera->lookAt(Vector3(0,0,-100));
	mCamera->setNearClipDistance(0.05f);
	LogManager::getSingleton().setLogDetail(LL_BOREME);

	// create ground
	dpsHelper = std::make_shared<DPSHelper>(Globals::phyWorld, mCamera, mSceneMgr);
	dpsHelper->createGround();

	// Main light in scene
	dpsHelper->createDirectionLight("mainLight",Ogre::Vector3(60,80,100),Ogre::Vector3(-60,-80,-100));

	// Debug drawing
	Globals::dbgdraw = new BtOgre::DebugDrawer(mSceneMgr->getRootSceneNode(), Globals::phyWorld);
	Globals::phyWorld->setDebugDrawer(Globals::dbgdraw);

	//Create Ogre stuff
	//Ogre::Plane planes;
	//planes.d = 100;
	//planes.normal = Ogre::Vector3::NEGATIVE_UNIT_Y;
	//mSceneMgr->setSkyPlane(true, planes, "Examples/CloudySky", 500, 20, true, 0.5, 150, 150);
	mSceneMgr->setSkyDome(true, "Examples/CloudySky", 5, 8);
}

void DPS::createFrameListenerBtOgre(void)
{
	mFrameListener = new BtOgreTestFrameListener(mWindow, mCamera, mSceneMgr);
	mRoot->addFrameListener(mFrameListener);
}

bool DPS::keyPressed(const OIS::KeyEvent &arg)
{
	if (arg.key == OIS::KC_1) 
	{
		dpsHelper->throwSphere();
	}
	if (arg.key == OIS::KC_2) 
	{
		dpsHelper->createOgreHead();
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