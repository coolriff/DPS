#include "DPS.h"
#include <vector>
#include <iostream>
#include <fstream> 
#include <string> 
//#include <memory>

const int maxProxies = 32766;

void DPS::initPhysics(void)
{
	//Bullet initialisation.
	m_dispatcher=0;
	///register some softbody collision algorithms on top of the default btDefaultCollisionConfiguration
	m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();
	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);
	m_softBodyWorldInfo.m_dispatcher = m_dispatcher;
	btVector3 worldAabbMin(-1000,-1000,-1000);
	btVector3 worldAabbMax(1000,1000,1000);
	m_broadphase = new btAxisSweep3(worldAabbMin,worldAabbMax,maxProxies);
	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();
	m_solver = solver;
	btSoftBodySolver* softBodySolver = 0;
	Globals::phyWorld = new btSoftRigidDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration,softBodySolver);

	//phyWorld->setInternalTickCallback(pickingPreTickCallback,this,true);
	Globals::phyWorld->getDispatchInfo().m_enableSPU = true;
	Globals::phyWorld->setGravity(btVector3(0,-10,0));
	m_softBodyWorldInfo.m_gravity.setValue(0,-10,0);
	//	clientResetScene();
	m_softBodyWorldInfo.m_sparsesdf.Initialize();
}

void DPS::exitPhysics(void)
{
	//cleanup in the reverse order of creation/initialization
	//remove the rigidbodies from the dynamics world and delete them
	int i;
	for (i=Globals::phyWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = Globals::phyWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		Globals::phyWorld->removeCollisionObject( obj );
		delete obj;
	}

	//delete collision shapes
	for (int j=0;j<m_collisionShapes.size();j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		m_collisionShapes[j] = 0;
		delete shape;
	}

	delete Globals::phyWorld;
	delete m_solver;
	delete m_broadphase;
	delete m_dispatcher;
	delete m_collisionConfiguration;
	delete Globals::dbgdraw;
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
	mSceneMgr->setAmbientLight(ColourValue(0.3, 0.3, 0.3));
	mSceneMgr->createLight()->setPosition(20, 80, 50);
	mCamera->setPosition(Vector3(0,20,20));
	mCamera->lookAt(Vector3(0,20,0));
	mCamera->setNearClipDistance(0.05f);
	//LogManager::getSingleton().setLogDetail(LL_BOREME);

	dpsHelper = std::make_shared<DPSHelper>(Globals::phyWorld, mCamera, mSceneMgr);
	dpsSoftbodyHelper = std::make_shared<DPSSoftBodyHelper>(Globals::phyWorld, mCamera, mSceneMgr);

	dpsHelper->createGround();

	// Main light in scene
	//dpsHelper->createDirectionLight("mainLight",Ogre::Vector3(60,180,100),Ogre::Vector3(-60,-80,-100));
	//dpsHelper->createDirectionLight("mainLight1",Ogre::Vector3(0,200,0),Ogre::Vector3(0,0,0));

	// Debug drawing
	Globals::dbgdraw = new BtOgre::DebugDrawer(mSceneMgr->getRootSceneNode(), Globals::phyWorld);
	Globals::phyWorld->setDebugDrawer(Globals::dbgdraw);

	//mSceneMgr->setSkyBox(true, "Examples/SpaceSkyBox");
	mSceneMgr->setSkyPlane(true, Plane(0, -1, 0, 5000), "Examples/SpaceSkyPlane", 10000, 3);


	//mSceneMgr->getRootSceneNode()->attachObject(mSceneMgr->createEntity("Dragon", "dragon.mesh"));

	Ogre::Vector3 pos = Ogre::Vector3(0,150,0);
	Ogre::Quaternion rot = Ogre::Quaternion::IDENTITY;
	Ogre::Entity *entDrangon = mSceneMgr->createEntity("Dragon","dragon.mesh");
	entDrangon->setCastShadows(true);
	Ogre::SceneNode* dragon = mSceneMgr->getRootSceneNode()->createChildSceneNode(pos,rot);
	dragon->attachObject(entDrangon);
	//dragon->attachObject(mSceneMgr->createParticleSystem("Smoke", "Examples/Smoke"));

	Ogre::Entity *entFire1 = mSceneMgr->createEntity("defCube.mesh");
	Ogre::SceneNode* fire1 = mSceneMgr->getRootSceneNode()->createChildSceneNode(Ogre::Vector3(0,0,20));
	//dpsHelper->setColor(entFire1, Ogre::Vector3(0.3021f,0.3308f,0.3671f));
	fire1->attachObject(entFire1);
	fire1->setPosition(Ogre::Vector3(0,0,20));
	fireOnCube_1 = mSceneMgr->createParticleSystem("Smoke", "Examples/Smoke");
	fire1->attachObject(fireOnCube_1);

// 	Ogre::Entity *entFire2 = mSceneMgr->createEntity("defCube.mesh");
// 	Ogre::SceneNode* fire2 = mSceneMgr->getRootSceneNode()->createChildSceneNode();
// 	//dpsHelper->setColor(entFire2, Ogre::Vector3(0.3021f,0.3308f,0.3671f));
// 	fire2->attachObject(entFire2);
// 	fire2->setPosition(Ogre::Vector3(0,0,-20));
// 	fireOnCube_2 = mSceneMgr->createParticleSystem("Smoke", "Examples/Smoke");
// 	fire2->attachObject(fireOnCube_2);

}


bool DPS::frameRenderingQueued(const Ogre::FrameEvent& evt)
{
	//Update Bullet world
	Globals::phyWorld->stepSimulation(evt.timeSinceLastFrame, 10); 
	Globals::phyWorld->debugDrawWorld();

	//Shows debug if F3 key down.
	Globals::dbgdraw->setDebugMode(mKeyboard->isKeyDown(OIS::KC_F3));
	Globals::dbgdraw->step();

	//Globals::app->updateSoftBody(dpsSoftbodyHelper->m_cloth);
	//Globals::app->updateSoftBody(dpsSoftbodyHelper->m_SoftBody);
	//Globals::app->updateSoftBody(dpsSoftbodyHelper->m_deformableModel);
	//Globals::app->updateSoftBody(dpsSoftbodyHelper->m_bunny);

	return BaseApplication::frameRenderingQueued(evt);
}


bool DPS::keyPressed(const OIS::KeyEvent &arg)
{
	if (arg.key == OIS::KC_1) 
	{
		dpsHelper->throwSphere();
	}
	if (arg.key == OIS::KC_2) 
	{
		dpsHelper->throwCube();
	}
	if (arg.key == OIS::KC_3) 
	{
		dpsHelper->createOgreHead();
	}
	return BaseApplication::keyPressed(arg);
}


void DPS::initSoftBody(btSoftBody* body)
{
	//manual objects are used to generate new meshes based on raw vertex data
	//this is used for the liquid form
	m_ManualObject = mSceneMgr->createManualObject("liquidBody");


	btSoftBody::tNodeArray& nodes(body->m_nodes);
	btSoftBody::tFaceArray& faces(body->m_faces);

	m_ManualObject->estimateVertexCount(faces.size()*3);
	m_ManualObject->estimateIndexCount(faces.size()*3);

	m_ManualObject->begin("CharacterMaterials/LiquidBody", Ogre::RenderOperation::OT_TRIANGLE_LIST);

	btSoftBody::Node *node0 = 0, *node1 = 0, *node2 = 0;

	//http://www.ogre3d.org/tikiwiki/ManualObject
	for (int i = 0; i < faces.size(); ++i)
	{
		node0 = faces[i].m_n[0];
		node1 = faces[i].m_n[1];
		node2 = faces[i].m_n[2];
				
		m_ManualObject->position(node0->m_x[0], node0->m_x[1], node0->m_x[2]);
		//m_ManualObject->textureCoord(1,0);
		m_ManualObject->normal(node0->m_n[0], node0->m_n[1], node0->m_n[2]);
				
		m_ManualObject->position(node1->m_x[0], node1->m_x[1], node1->m_x[2]);
		//m_ManualObject->textureCoord(0,1);
		m_ManualObject->normal(node1->m_n[0], node1->m_n[1], node1->m_n[2]);
				
		m_ManualObject->position(node2->m_x[0], node2->m_x[1], node2->m_x[2]);
		//m_ManualObject->textureCoord(1,1);
		m_ManualObject->normal(node2->m_n[0], node2->m_n[1], node2->m_n[2]);

		m_ManualObject->triangle(i*3,i*3+1,i*3+2);
		//m_ManualObject->triangle(i*3+1);
		//m_ManualObject->triangle(i*3+2);
	}
// 	m_ManualObject->textureCoord(1,0);
// 	m_ManualObject->textureCoord(0,0);
// 	m_ManualObject->textureCoord(0,1);
// 	m_ManualObject->textureCoord(1,1);
	m_ManualObject->end();
	m_ManualObject->setDynamic(true);

	Ogre::SceneNode* mLiquidBodyNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	mLiquidBodyNode->attachObject(m_ManualObject);
}


void DPS::updateSoftBody(btSoftBody* body)
{
	//grab the calculated mesh data from the physics body
	btSoftBody::tNodeArray& nodes(body->m_nodes);
	btSoftBody::tFaceArray& faces(body->m_faces);

	m_ManualObject->beginUpdate(0);
	btSoftBody::Node *node0 = 0, *node1 = 0, *node2 = 0;
	for (int i = 0; i < faces.size(); i++)
	{
		node0 = faces[i].m_n[0];
		node1 = faces[i].m_n[1];
		node2 = faces[i].m_n[2];
				
		m_ManualObject->position(node0->m_x[0], node0->m_x[1], node0->m_x[2]);
		m_ManualObject->normal(node0->m_n[0], node0->m_n[1], node0->m_n[2]);
				
		m_ManualObject->position(node1->m_x[0], node1->m_x[1], node1->m_x[2]);
		m_ManualObject->normal(node1->m_n[0], node1->m_n[1], node1->m_n[2]);
				
		m_ManualObject->position(node2->m_x[0], node2->m_x[1], node2->m_x[2]);
		m_ManualObject->normal(node2->m_n[0], node2->m_n[1], node2->m_n[2]);

		m_ManualObject->index(i*3);
		m_ManualObject->index(i*3+1);
		m_ManualObject->index(i*3+2);
	}
	m_ManualObject->setCastShadows(true);
	m_ManualObject->end();
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
			//DPS app;
			Globals::app = new DPS();
			try {
				Globals::app->go();
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