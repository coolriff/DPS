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

		Globals::app->updateLiquidBody();
		
	    return ExampleFrameListener::frameStarted(evt);
	}
};

DPS::DPS(void)
{
	//Bullet initialisation.
	//mBroadphase = new btAxisSweep3(btVector3(-10000,-10000,-10000), btVector3(10000,10000,10000), 1024);
	//mCollisionConfig = new btDefaultCollisionConfiguration();
	//mDispatcher = new btCollisionDispatcher(mCollisionConfig);
	//mSolver = new btSequentialImpulseConstraintSolver();

	collisionConfig = new btSoftBodyRigidBodyCollisionConfiguration();
	dispatcher = new btCollisionDispatcher(collisionConfig);
	broadphase = new btDbvtBroadphase();
	solver = new btSequentialImpulseConstraintSolver();
	softbodySolver = new btDefaultSoftBodySolver();
	Globals::phyWorld = new btSoftRigidDynamicsWorld(dispatcher,broadphase,solver,collisionConfig,softbodySolver);
	Globals::phyWorld->setGravity(btVector3(0,-10,0));


	//Globals::phyWorld = new btDiscreteDynamicsWorld(mDispatcher, mBroadphase, mSolver, mCollisionConfig);
	//Globals::phyWorld->setGravity(btVector3(0,-10,0));
}


DPS::~DPS(void)
{
	//Free Bullet stuff
	/*delete mSolver;
	delete mDispatcher;
	delete mCollisionConfig;
	delete mBroadphase;*/

	delete Globals::dbgdraw;
	delete Globals::phyWorld;
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
	//LogManager::getSingleton().setLogDetail(LL_BOREME);

	// create ground
	dpsHelper = std::make_shared<DPSHelper>(Globals::phyWorld, mCamera, mSceneMgr);
	dpsHelper->createGround();

	// Main light in scene
	dpsHelper->createDirectionLight("mainLight",Ogre::Vector3(60,180,100),Ogre::Vector3(-60,-80,-100));
	dpsHelper->createDirectionLight("mainLight1",Ogre::Vector3(0,200,0),Ogre::Vector3(0,0,0));

	// Debug drawing
	Globals::dbgdraw = new BtOgre::DebugDrawer(mSceneMgr->getRootSceneNode(), Globals::phyWorld);
	Globals::phyWorld->setDebugDrawer(Globals::dbgdraw);

	//Create Ogre stuff
	//Ogre::Plane planes;
	//planes.d = 100;
	//planes.normal = Ogre::Vector3::NEGATIVE_UNIT_Y;
	//mSceneMgr->setSkyPlane(true, planes, "Examples/CloudySky", 500, 20, true, 0.5, 150, 150);
	//mSceneMgr->setSkyDome(true, "Examples/CloudySky", 5, 8);

	//createLiquidBody(btVector3(0,150,0));


	m_LiquidBody=btSoftBodyHelpers::CreateEllipsoid(Globals::phyWorld->getWorldInfo(),btVector3(10,50,10),btVector3(20,20,20),1000);
	m_LiquidBody->m_cfg.viterations=200;
	m_LiquidBody->m_cfg.piterations=200;
	m_LiquidBody->m_cfg.kPR=10000;
	m_LiquidBody->setTotalMass(3.0);
	m_LiquidBody->setMass(0,0);
	Globals::phyWorld->addSoftBody(m_LiquidBody);


	initLiquidBody();
	//updateLiquidBody();
	//Globals::phyWorld->addSoftBody(m_LiquidBody);


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

void DPS::createLiquidBody(const btVector3& startPos)
{
	m_LiquidBody = btSoftBodyHelpers::CreateEllipsoid(*m_SoftBodyWorldInfo, startPos, btVector3(20,20,20), 100);

	//set the liquid body properties
	m_LiquidBody->m_cfg.kPR = 3500.f;
	m_LiquidBody->m_cfg.kDP = 0.001f;
	m_LiquidBody->m_cfg.kDF = 0.1f;
	m_LiquidBody->m_cfg.kKHR = 1.f; //we hardcode this parameter, since any value below 1.0 means the soft body does less than full correction on penetration
	m_LiquidBody->m_cfg.kCHR  = 1.f;
	
	m_LiquidBody->generateClusters(100);
	m_LiquidBody->m_materials[0]->m_kLST = 0.1f;
	//Globals::phyWorld->addSoftBody(m_LiquidBody);
}

void DPS::initLiquidBody(void)
{
	//manual objects are used to generate new meshes based on raw vertex data
	//this is used for the liquid form
	m_ManualObject = mSceneMgr->createManualObject("liquidBody");

	/*
		The following code needs to be run once to setup the vertex buffer with data based on
		the bullet soft body information.
	*/
	btSoftBody::tNodeArray& nodes(m_LiquidBody->m_nodes);
	btSoftBody::tFaceArray& faces(m_LiquidBody->m_faces);

	m_ManualObject->estimateVertexCount(faces.size()*3);
	m_ManualObject->estimateIndexCount(faces.size()*3);

	m_ManualObject->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);
	for (int i = 0; i < faces.size(); i++)
	{
		btSoftBody::Node *node0 = 0, *node1 = 0, *node2 = 0;
		node0 = faces[i].m_n[0];
		node1 = faces[i].m_n[1];
		node2 = faces[i].m_n[2];
				
		m_ManualObject->position(node0->m_x[0], node0->m_x[1], node0->m_x[2]);
		m_ManualObject->colour(Ogre::ColourValue(0.7f,0.7f,0.7f,1.0f));
		m_ManualObject->normal(node0->m_n[0], node0->m_n[1], node0->m_n[2]);
				
		m_ManualObject->position(node1->m_x[0], node1->m_x[1], node1->m_x[2]);
		m_ManualObject->colour(Ogre::ColourValue(0.7f,0.7f,0.7f,1.0f));
		m_ManualObject->normal(node1->m_n[0], node1->m_n[1], node1->m_n[2]);
				
		m_ManualObject->position(node2->m_x[0], node2->m_x[1], node2->m_x[2]);
		m_ManualObject->colour(Ogre::ColourValue(0.7f,0.7f,0.7f,1.0f));
		m_ManualObject->normal(node2->m_n[0], node2->m_n[1], node2->m_n[2]);

	

		m_ManualObject->index(i*3);
		m_ManualObject->index(i*3+1);
		m_ManualObject->index(i*3+2);
	}
	m_ManualObject->end();

	m_ManualObject->setDynamic(true);
	m_ManualObject->setCastShadows(true);

	Ogre::SceneNode* mLiquidBodyNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	mLiquidBodyNode->attachObject(m_ManualObject);
}

void DPS::updateLiquidBody(void)
{
	//grab the calculated mesh data from the physics body
	btSoftBody::tNodeArray& nodes(m_LiquidBody->m_nodes);
	btSoftBody::tFaceArray& faces(m_LiquidBody->m_faces);

	float minx, miny, minz, maxx, maxy, maxz;
	minx = maxx = nodes[0].m_x[0];
	miny = maxy = nodes[0].m_x[1];
	minz = maxz = nodes[0].m_x[2];

	m_ManualObject->beginUpdate(0);
	for (int i = 0; i < faces.size(); i++)
	{
		btSoftBody::Node *node0 = 0, *node1 = 0, *node2 = 0;
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