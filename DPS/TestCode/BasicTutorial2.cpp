/*
-----------------------------------------------------------------------------
Filename:    BasicTutorial2.cpp
-----------------------------------------------------------------------------
 
This source file is part of the
   ___                 __    __ _ _    _ 
  /___\__ _ _ __ ___  / / /\ \ (_) | _(_)
 //  // _` | '__/ _ \ \ \/  \/ / | |/ / |
/ \_// (_| | | |  __/  \  /\  /| |   <| |
\___/ \__, |_|  \___|   \/  \/ |_|_|\_\_|
      |___/                              
	  Tutorial Framework
	  http://www.ogre3d.org/tikiwiki/
-----------------------------------------------------------------------------
*/
#include "Physics.h"
#include "BasicTutorial2.h"
 
//-------------------------------------------------------------------------------------
BasicTutorial2::BasicTutorial2(void)
{
}
//-------------------------------------------------------------------------------------
BasicTutorial2::~BasicTutorial2(void)
{
}
//-------------------------------------------------------------------------------------
void BasicTutorial2::createCamera(void)
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
void BasicTutorial2::createViewports(void)
{
    // Create one viewport, entire window
    Ogre::Viewport* vp = mWindow->addViewport(mCamera);
    vp->setBackgroundColour(Ogre::ColourValue(0,0,0));
    // Alter the camera aspect ratio to match the viewport
    mCamera->setAspectRatio(Ogre::Real(vp->getActualWidth()) / Ogre::Real(vp->getActualHeight()));    
}
//-------------------------------------------------------------------------------------
void BasicTutorial2::createScene(void)
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

	//create the actual plane in Ogre3D
	physics::initObjects();
	Ogre::Plane plane(Ogre::Vector3::UNIT_Y, 0);
	Ogre::MeshPtr planePtr = Ogre::MeshManager::getSingleton().createPlane("ground", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, plane, 1500, 1500, 20, 20, true, 1, 5, 5, Ogre::Vector3::UNIT_Z);
 
	Ogre::Entity *entGround = mSceneMgr->createEntity("GroundEntity", "ground");
	Ogre::SceneNode *groundNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("groundNode");
 
	groundNode->attachObject(entGround);
 
	//create the plane entity to the physics engine, and attach it to the node
 
	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0, -50, 0));
 
	btScalar groundMass(0.); //the mass is 0, because the ground is immovable (static)
	btVector3 localGroundInertia(0, 0, 0);
 
	btCollisionShape *groundShape = new btBoxShape(btVector3(btScalar(50.), btScalar(50.), btScalar(50.)));
	btDefaultMotionState *groundMotionState = new btDefaultMotionState(groundTransform);
 
	groundShape->calculateLocalInertia(groundMass, localGroundInertia);
 
	btRigidBody::btRigidBodyConstructionInfo groundRBInfo(groundMass, groundMotionState, groundShape, localGroundInertia);
	btRigidBody *groundBody = new btRigidBody(groundRBInfo);
 
	//add the body to the dynamics world
	this->physicsEngine->getDynamicsWorld()->addRigidBody(groundBody);



	Ogre::MeshPtr mesh = Ogre::MeshManager::getSingleton().getByName("Cube.mesh").staticCast<Ogre::Mesh>();
 
	Ogre::Entity *entity = this->mSceneMgr->createEntity(mesh);
 
	Ogre::SceneNode *newNode = this->mSceneMgr->getRootSceneNode()->createChildSceneNode(physicsCubeName);
	newNode->attachObject(entity);
 
	//create the new shape, and tell the physics that is a Box
	btCollisionShape *newRigidShape = new btBoxShape(btVector3(1.0f, 1.0f, 1.0f));
	this->physicsEngine->getCollisionShapes().push_back(newRigidShape);
 
	//set the initial position and transform. For this demo, we set the tranform to be none
	btTransform startTransform;
	startTransform.setIdentity();
	startTransform.setRotation(btQuaternion(1.0f, 1.0f, 1.0f, 0));
 
	//set the mass of the object. a mass of "0" means that it is an immovable object
	btScalar mass = 0.1f;
	btVector3 localInertia(0,0,0);
 
	startTransform.setOrigin(initialPosition);
	newRigidShape->calculateLocalInertia(mass, localInertia);
 
	//actually contruvc the body and add it to the dynamics world
	btDefaultMotionState *myMotionState = new btDefaultMotionState(startTransform);
 
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, newRigidShape, localInertia);
	btRigidBody *body = new btRigidBody(rbInfo);
	body->setRestitution(1);
	body->setUserPointer(newNode);
 
	physicsEngine->getDynamicsWorld()->addRigidBody(body);
	physicsEngine->trackRigidBodyWithName(body, physicsCubeName);
}


bool Engine::frameStarted (const Ogre::FrameEvent &evt){
    if (this->physicsEngine != NULL){
        physicsEngine->getDynamicsWorld()->stepSimulation(1.0f/60.0f); //suppose you have 60 frames per second
 
        for (int i = 0; i< this->physicsEngine->getCollisionObjectCount(); i++) {
            btCollisionObject* obj = this->physicsEngine->getDynamicsWorld()->getCollisionObjectArray()[i];
            btRigidBody* body = btRigidBody::upcast(obj);
 
            if (body && body->getMotionState()){
                btTransform trans;
                body->getMotionState()->getWorldTransform(trans);
 
                void *userPointer = body->getUserPointer();
                if (userPointer) {
                    btQuaternion orientation = trans.getRotation();
                    Ogre::SceneNode *sceneNode = static_cast<Ogre::SceneNode *>(userPointer);
                    sceneNode->setPosition(Ogre::Vector3(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ()));
                    sceneNode->setOrientation(Ogre::Quaternion(orientation.getW(), orientation.getX(), orientation.getY(), orientation.getZ()));
                }
            }
        }
    }
    return true;
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
			BasicTutorial2 app;
 
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