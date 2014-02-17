/*
-----------------------------------------------------------------------------
Filename:    BasicTutorial4.cpp
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
#include "BasicTutorial4.h"
#include "btBulletDynamicsCommon.h"
 
//-------------------------------------------------------------------------------------
BasicTutorial4::BasicTutorial4(void)
{
}
//-------------------------------------------------------------------------------------
BasicTutorial4::~BasicTutorial4(void)
{
}
//-------------------------------------------------------------------------------------
void BasicTutorial4::createScene(void)
{
    mSceneMgr->setAmbientLight(Ogre::ColourValue(0.25, 0.25, 0.25));
 
    Ogre::Entity* ninjaEntity = mSceneMgr->createEntity("Ninja", "ninja.mesh");
    Ogre::SceneNode *node = mSceneMgr->getRootSceneNode()->createChildSceneNode("NinjaNode");
    node->attachObject(ninjaEntity);
 
    Ogre::Light* pointLight = mSceneMgr->createLight("pointLight");
    pointLight->setType(Ogre::Light::LT_POINT);
    pointLight->setPosition(Ogre::Vector3(250, 150, 250));
    pointLight->setDiffuseColour(Ogre::ColourValue::White);
    pointLight->setSpecularColour(Ogre::ColourValue::White);
}
//-------------------------------------------------------------------------------------
bool BasicTutorial4::processUnbufferedInput(const Ogre::FrameEvent& evt)
{
    static bool mMouseDown = false;     // If a mouse button is depressed
    static Ogre::Real mToggle = 0.0;    // The time left until next toggle
    static Ogre::Real mRotate = 0.13;   // The rotate constant
    static Ogre::Real mMove = 250;      // The movement constant
 
    bool currMouse = mMouse->getMouseState().buttonDown(OIS::MB_Left);
 
    if (currMouse && ! mMouseDown)
    {
        Ogre::Light* light = mSceneMgr->getLight("pointLight");
        light->setVisible(! light->isVisible());
    }
 
    mMouseDown = currMouse;
 
    mToggle -= evt.timeSinceLastFrame;
 
    if ((mToggle < 0.0f ) && mKeyboard->isKeyDown(OIS::KC_1))
    {
        mToggle  = 0.5;
        Ogre::Light* light = mSceneMgr->getLight("pointLight");
        light->setVisible(! light->isVisible());
    }
 
    Ogre::Vector3 transVector = Ogre::Vector3::ZERO;
 
    if (mKeyboard->isKeyDown(OIS::KC_I)) // Forward
    {
        transVector.z -= mMove;
    }
    if (mKeyboard->isKeyDown(OIS::KC_K)) // Backward
    {
        transVector.z += mMove;
    }
    if (mKeyboard->isKeyDown(OIS::KC_J)) // Left - yaw or strafe
    {
        if(mKeyboard->isKeyDown( OIS::KC_LSHIFT ))
        {
            // Yaw left
            mSceneMgr->getSceneNode("NinjaNode")->yaw(Ogre::Degree(mRotate * 5));
        } else {
            transVector.x -= mMove; // Strafe left
        }
    }
    if (mKeyboard->isKeyDown(OIS::KC_L)) // Right - yaw or strafe
    {
        if(mKeyboard->isKeyDown( OIS::KC_LSHIFT ))
        {
            // Yaw right
            mSceneMgr->getSceneNode("NinjaNode")->yaw(Ogre::Degree(-mRotate * 5));
        } else {
            transVector.x += mMove; // Strafe right
        }
    }
    if (mKeyboard->isKeyDown(OIS::KC_U)) // Up
    {
        transVector.y += mMove;
    }
    if (mKeyboard->isKeyDown(OIS::KC_O)) // Down
    {
        transVector.y -= mMove;
    }
 
    mSceneMgr->getSceneNode("NinjaNode")->translate(transVector * evt.timeSinceLastFrame, Ogre::Node::TS_LOCAL);
 
    return true;
}
//-------------------------------------------------------------------------------------
bool BasicTutorial4::frameRenderingQueued(const Ogre::FrameEvent& evt)
{
    bool ret = BaseApplication::frameRenderingQueued(evt);
 
    if(!processUnbufferedInput(evt)) return false;
 
    return ret;
}
//-------------------------------------------------------------------------------------
 
 
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
        // Bullset testing
		///-----includes_end-----

		int i;
		///-----initialization_start-----

		///collision configuration contains default setup for memory, collision setup. Advanced users can create their own configuration.
		btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();

		///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
		btCollisionDispatcher* dispatcher = new	btCollisionDispatcher(collisionConfiguration);

		///btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
		btBroadphaseInterface* overlappingPairCache = new btDbvtBroadphase();

		///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
		btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

		btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,overlappingPairCache,solver,collisionConfiguration);

		dynamicsWorld->setGravity(btVector3(0,-10,0));

		///-----initialization_end-----

		///create a few basic rigid bodies
		btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(50.),btScalar(50.),btScalar(50.)));

		//keep track of the shapes, we release memory at exit.
		//make sure to re-use collision shapes among rigid bodies whenever possible!
		btAlignedObjectArray<btCollisionShape*> collisionShapes;

		collisionShapes.push_back(groundShape);

		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0,-56,0));

		{
			btScalar mass(0.);

			//rigidbody is dynamic if and only if mass is non zero, otherwise static
			bool isDynamic = (mass != 0.f);

			btVector3 localInertia(0,0,0);
			if (isDynamic)
				groundShape->calculateLocalInertia(mass,localInertia);

			//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
			btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
			btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,groundShape,localInertia);
			btRigidBody* body = new btRigidBody(rbInfo);

			//add the body to the dynamics world
			dynamicsWorld->addRigidBody(body);
		}


		{
			//create a dynamic rigidbody

			//btCollisionShape* colShape = new btBoxShape(btVector3(1,1,1));
			btCollisionShape* colShape = new btSphereShape(btScalar(1.));
			collisionShapes.push_back(colShape);

			/// Create Dynamic Objects
			btTransform startTransform;
			startTransform.setIdentity();

			btScalar	mass(1.f);

			//rigidbody is dynamic if and only if mass is non zero, otherwise static
			bool isDynamic = (mass != 0.f);

			btVector3 localInertia(0,0,0);
			if (isDynamic)
				colShape->calculateLocalInertia(mass,localInertia);

				startTransform.setOrigin(btVector3(2,10,0));
		
				//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
				btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
				btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,colShape,localInertia);
				btRigidBody* body = new btRigidBody(rbInfo);

				dynamicsWorld->addRigidBody(body);
		}



	/// Do some simulation


		///-----stepsimulation_start-----
		for (i=0;i<100;i++)
		{
			dynamicsWorld->stepSimulation(1.f/60.f,10);
		
			//print positions of all objects
			for (int j=dynamicsWorld->getNumCollisionObjects()-1; j>=0 ;j--)
			{
				btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[j];
				btRigidBody* body = btRigidBody::upcast(obj);
				if (body && body->getMotionState())
				{
					btTransform trans;
					body->getMotionState()->getWorldTransform(trans);
					printf("world pos = %f,%f,%f\n",float(trans.getOrigin().getX()),float(trans.getOrigin().getY()),float(trans.getOrigin().getZ()));
				}
			}
		}

		///-----stepsimulation_end-----

		//cleanup in the reverse order of creation/initialization
	
		///-----cleanup_start-----

		//remove the rigidbodies from the dynamics world and delete them
		for (i=dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
		{
			btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[i];
			btRigidBody* body = btRigidBody::upcast(obj);
			if (body && body->getMotionState())
			{
				delete body->getMotionState();
			}
			dynamicsWorld->removeCollisionObject( obj );
			delete obj;
		}

		//delete collision shapes
		for (int j=0;j<collisionShapes.size();j++)
		{
			btCollisionShape* shape = collisionShapes[j];
			collisionShapes[j] = 0;
			delete shape;
		}

		//delete dynamics world
		delete dynamicsWorld;

		//delete solver
		delete solver;

		//delete broadphase
		delete overlappingPairCache;

		//delete dispatcher
		delete dispatcher;

		delete collisionConfiguration;

		//next line is optional: it will be cleared by the destructor when the array goes out of scope
		collisionShapes.clear();

		///-----cleanup_end-----
		// End of bullet testing
		
		// Create application object
        BasicTutorial4 app;
 
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