#include "DPS.h"
#include "BtOgrePG.h"
#include "BtOgreGP.h"
#include "BtOgreExtras.h"
#include "ExampleApplication.h"

namespace Globals
{
    btDynamicsWorld *phyWorld;
	BtOgre::DebugDrawer *dbgdraw;
}

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
	Globals::phyWorld->setGravity(btVector3(0,-10,0));
}


DPS::~DPS(void)
{

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
	/*
	mSceneMgr->setAmbientLight(Ogre::ColourValue(1, 1, 1));
    mSceneMgr->setShadowTechnique(Ogre::SHADOWTYPE_STENCIL_ADDITIVE);
 
    Ogre::Entity* entNinja = mSceneMgr->createEntity("Ninja", "ninja.mesh");
    entNinja->setCastShadows(true);
    mSceneMgr->getRootSceneNode()->createChildSceneNode()->attachObject(entNinja);

	Ogre::Entity* ogreHead2 = mSceneMgr->createEntity( "Head2", "ogrehead.mesh" );
	ogreHead2->setCastShadows(true);
	Ogre::SceneNode* headNode2 = mSceneMgr->getRootSceneNode()->createChildSceneNode( "HeadNode2", Ogre::Vector3( 100, 50, 0 ) );
	headNode2->attachObject( ogreHead2 );
	headNode2->pitch( Ogre::Degree( -90 ) );
 
	Ogre::Entity* ogreHead3 = mSceneMgr->createEntity( "Head3", "ogrehead.mesh" );
	ogreHead3->setCastShadows(true);
	Ogre::SceneNode* headNode3 = mSceneMgr->getRootSceneNode()->createChildSceneNode( "HeadNode3", Ogre::Vector3( 200, 50, 0 ) );
	headNode3->attachObject( ogreHead3 );
	headNode3->roll( Ogre::Degree( -90 ) );

	Ogre::Entity* entCube = mSceneMgr->createEntity("cube", "cube.mesh");
    entCube->setCastShadows(false);
    Ogre::SceneNode* cubeNode1 = mSceneMgr->getRootSceneNode()->createChildSceneNode("cubeNode", Ogre::Vector3( 300, 100, 0 ));
	cubeNode1->attachObject(entCube);

	Ogre::Entity* entBarrel = mSceneMgr->createEntity("barrel", "Barrel.mesh");
    entBarrel->setCastShadows(false);
    Ogre::SceneNode* barrelNote1 = mSceneMgr->getRootSceneNode()->createChildSceneNode("barrelNote", Ogre::Vector3( -100, 100, 0 ));
	barrelNote1->attachObject(entBarrel);
 
    Ogre::Plane plane(Ogre::Vector3::UNIT_Y, 0);
 
    Ogre::MeshManager::getSingleton().createPlane("ground", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        plane, 1500, 1500, 20, 20, true, 1, 5, 5, Ogre::Vector3::UNIT_Z);
 
    Ogre::Entity* entGround = mSceneMgr->createEntity("GroundEntity", "ground");
    mSceneMgr->getRootSceneNode()->createChildSceneNode()->attachObject(entGround);
 
    entGround->setMaterialName("Examples/Rockwall");
    entGround->setCastShadows(false);
 
    Ogre::Light* pointLight = mSceneMgr->createLight("pointLight");
    pointLight->setType(Ogre::Light::LT_POINT);
    pointLight->setPosition(Ogre::Vector3(0, 150, 250));
 
    pointLight->setDiffuseColour(1.0, 0.0, 0.0);
    pointLight->setSpecularColour(1.0, 0.0, 0.0);
 
    Ogre::Light* directionalLight = mSceneMgr->createLight("directionalLight");
    directionalLight->setType(Ogre::Light::LT_DIRECTIONAL);
    directionalLight->setDiffuseColour(Ogre::ColourValue(.25, .25, 0));
    directionalLight->setSpecularColour(Ogre::ColourValue(.25, .25, 0));
 
    directionalLight->setDirection(Ogre::Vector3( 0, -1, 1 )); 
 
    Ogre::Light* spotLight = mSceneMgr->createLight("spotLight");
    spotLight->setType(Ogre::Light::LT_SPOTLIGHT);
    spotLight->setDiffuseColour(0, 0, 1.0);
    spotLight->setSpecularColour(0, 0, 1.0);
 
    spotLight->setDirection(-1, -1, 0);
    spotLight->setPosition(Ogre::Vector3(300, 300, 0));
 
    spotLight->setSpotlightRange(Ogre::Degree(35), Ogre::Degree(50));
	*/

		mSceneMgr->setAmbientLight(Ogre::ColourValue(0.7,0.7,0.7));
	    mCamera->setPosition(Ogre::Vector3(10,10,10));
	    mCamera->lookAt(Ogre::Vector3::ZERO);
	    mCamera->setNearClipDistance(0.05);
	    Ogre::LogManager::getSingleton().setLogDetail(Ogre::LL_BOREME);

	    //----------------------------------------------------------
	    // Debug drawing!
	    //----------------------------------------------------------

	    Globals::dbgdraw = new BtOgre::DebugDrawer(mSceneMgr->getRootSceneNode(), Globals::phyWorld);
	    Globals::phyWorld->setDebugDrawer(Globals::dbgdraw);

	    //----------------------------------------------------------
	    // Ninja!
	    //----------------------------------------------------------

	    Ogre::Vector3 pos = Ogre::Vector3(0,10,0);
	    Ogre::Quaternion rot = Ogre::Quaternion::IDENTITY;

	    //Create Ogre stuff.

	    mNinjaEntity = mSceneMgr->createEntity("ninjaEntity", "Player.mesh");
	    mNinjaNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("ninjaSceneNode", pos, rot);
	    mNinjaNode->attachObject(mNinjaEntity);

	    //Create shape.
	    BtOgre::StaticMeshToShapeConverter converter(mNinjaEntity);
	    mNinjaShape = converter.createSphere();

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
	    //MeshManager::getSingleton().createPlane("groundPlane", "General", Plane(Vector3::UNIT_Y, 0), 100, 100, 
	    //10, 10, true, 1, 5, 5, Vector3::UNIT_Z);
	    mGroundEntity = mSceneMgr->createEntity("groundEntity", "TestLevel_b0.mesh");
	    //mGroundEntity->setMaterialName("Examples/Rockwall");
	    mSceneMgr->getRootSceneNode()->createChildSceneNode("groundNode")->attachObject(mGroundEntity);

	    //Create the ground shape.
	    BtOgre::StaticMeshToShapeConverter converter2(mGroundEntity);
	    mGroundShape = converter2.createTrimesh();

	    //Create MotionState (no need for BtOgre here, you can use it if you want to though).
	    btDefaultMotionState* groundState = new btDefaultMotionState(
		    btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0)));

	    //Create the Body.
	    mGroundBody = new btRigidBody(0, groundState, mGroundShape, btVector3(0,0,0));
	    Globals::phyWorld->addRigidBody(mGroundBody);
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