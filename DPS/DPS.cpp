#include "DPS.h"
#include <vector>
#include <iostream>
#include <string> 
#include "BulletCollision/Gimpact/btGImpactShape.h"
#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"
#include "Mesh/BunnyMesh.h"
#include "GUI.h"
#include <string>
#include <sstream>

const int maxProxies = 32766;
btRigidBody * hit_body = 0;
btVector3 hit_rel_pos;
btVector3 shot_imp = btVector3(0,0,0);

DPS::DPS(void)
{
	initPhysics();
	rayNode = nullptr;
	leapMotionCounter = 0;
	runClothDome_1 = false;
	runClothDome_2 = false;
	runClothDome_3 = false;
	runClothDome_4 = false;
	runClothDome_5 = false;
	runClothDome_6 = false;
	runClothDome_7 = false;

	runDeformDome_1 = false;
	runDeformDome_2 = false;
	runDeformDome_3 = false;
	runDeformDome_4 = false;
	runDeformDome_5 = false;
	runDeformDome_6 = false;
	runDeformDome_7 = false;

	runPlaygroud_1 = false;
	runPlaygroud_2 = false;
	runPlaygroud_3 = false;
	runPlaygroud_4 = false;
	runPlaygroud_5 = false;

	playgroundCount = 0;
}

DPS::~DPS(void)
{
	deletePhysicsShapes();
	exitPhysics();
	leapMotionCleanup();
}

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
	btGImpactCollisionAlgorithm::registerAlgorithm(m_dispatcher);

	//phyWorld->setInternalTickCallback(pickingPreTickCallback,this,true);
	Globals::phyWorld->getDispatchInfo().m_enableSPU = true;
	Globals::phyWorld->setGravity(btVector3(0,-10,0));
	m_softBodyWorldInfo.m_gravity.setValue(0,-10,0);
	m_softBodyWorldInfo.m_sparsesdf.Initialize();

	dt = evt.timeSinceLastFrame;
}


void DPS::exitPhysics(void)
{
	delete Globals::phyWorld;
	delete m_solver;
	delete m_broadphase;
	delete m_dispatcher;
	delete m_collisionConfiguration;
	delete Globals::dbgdraw;
}


void DPS::deletePhysicsShapes(void)
{
	int i;
	for (i=Globals::phyWorld->getNumCollisionObjects()-1 ; i>=0 ; i--)
	{
		btCollisionObject* obj = Globals::phyWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		Globals::phyWorld->removeCollisionObject(obj);
		delete obj;
	}

	//delete collision shapes
	for (int j=0;j<m_collisionShapes.size();j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		m_collisionShapes[j] = 0;
		delete shape;
	}
}


void DPS::createScene(void)
{
	mGUI->createGUI(1);

	// Basic Ogre stuff.
	mSceneMgr->setAmbientLight(Ogre::ColourValue(0.7f,0.7f,0.5f));
	mCamera->setPosition(Ogre::Vector3(0,5,20));
	mCamera->lookAt(Ogre::Vector3(0,5,-10));
	mCamera->setNearClipDistance(0.05f);
	//LogManager::getSingleton().setLogDetail(LL_BOREME);

	dpsHelper = std::make_shared<DPSHelper>(Globals::phyWorld, mCamera, mSceneMgr);
	dpsSoftbodyHelper = std::make_shared<DPSSoftBodyHelper>(Globals::phyWorld, mCamera, mSceneMgr, m_collisionShapes);

	dpsHelper->createWorld();

	// Debug drawing
	Globals::dbgdraw = new BtOgre::DebugDrawer(mSceneMgr->getRootSceneNode(), Globals::phyWorld);
	Globals::phyWorld->setDebugDrawer(Globals::dbgdraw);

	leapMotionInit();
	setMiniCamPosition(Ogre::Vector3(10,10,10));
}

bool DPS::frameRenderingQueued(const Ogre::FrameEvent& evt)
{
	if(mGUI->Simulation_Stop)
	{
		dt = 0;
	}
	else
	{	
		if(mGUI->Command_Enable_Slow)
		{
			dt = 0.0002;
		}
		else
		{
			dt = evt.timeSinceLastFrame + mGUI->demoDt;
		}
	}

	GUIeventHandler();
	demoController();

	if(rayNode)
	{
		setMiniCamPosition(rayNode->getPosition());
	}

	//Update Bullet world
	Globals::phyWorld->stepSimulation(dt,0); 
	Globals::phyWorld->debugDrawWorld();

	//Shows debug if F3 key down.
	//Globals::dbgdraw->setDebugMode(mKeyboard->isKeyDown(OIS::KC_F3));
	Globals::dbgdraw->setDebugMode(mGUI->Command_Bullet_Debug_Mode);
	Globals::dbgdraw->step();

	Ogre::Vector3 camPos = mCamera->getDerivedPosition();

	if (camPos.y<0.5)
	{
		mCamera->setPosition(camPos.x, 0.5, camPos.z);
	}

	if(leapMotionRunning)
	{
		leapMotionUpdate();
	}

	return BaseApplication::frameRenderingQueued(evt);
}

void DPS::clearScreen(void)
{
	mGUI->Command_Clear_Screen = false;

	leapMotionCleanup();

	deletePhysicsShapes();
	deleteOgreEntities();

	leapMotionRunning = false;
	leapMotionInit();

	//stop demos loops
	runClothDome_1 = false;
	runClothDome_2 = false;
	runClothDome_3 = false;
	runClothDome_4 = false;
	runClothDome_5 = false;
	runClothDome_6 = false;
	runClothDome_7 = false;

 	runDeformDome_1 = false;
	runDeformDome_2 = false;
	runDeformDome_3 = false;
 	runDeformDome_4 = false;
 	runDeformDome_5 = false;
	runDeformDome_6 = false;
	runDeformDome_7 = false;

	runPlaygroud_1 = false;
	runPlaygroud_2 = false;
	runPlaygroud_3 = false;
	runPlaygroud_4 = false;
	runPlaygroud_5 = false;

	//enable re click for all demos simply delete all following code.
	mGUI->mGuiSystem->findWidget<MyGUI::Widget>("Command_Cloth_Demo_1")->setEnabled(true);
	mGUI->mGuiSystem->findWidget<MyGUI::Widget>("Command_Cloth_Demo_2")->setEnabled(true);
	mGUI->mGuiSystem->findWidget<MyGUI::Widget>("Command_Cloth_Demo_3")->setEnabled(true);
	mGUI->mGuiSystem->findWidget<MyGUI::Widget>("Command_Cloth_Demo_4")->setEnabled(true);
	mGUI->mGuiSystem->findWidget<MyGUI::Widget>("Command_Cloth_Demo_5")->setEnabled(true);
	mGUI->mGuiSystem->findWidget<MyGUI::Widget>("Command_Cloth_Demo_6")->setEnabled(true);
	mGUI->mGuiSystem->findWidget<MyGUI::Widget>("Command_Cloth_Demo_7")->setEnabled(true);

	mGUI->mGuiSystem->findWidget<MyGUI::Widget>("Command_Deformable_Demo_1")->setEnabled(true);
	mGUI->mGuiSystem->findWidget<MyGUI::Widget>("Command_Deformable_Demo_2")->setEnabled(true);
	mGUI->mGuiSystem->findWidget<MyGUI::Widget>("Command_Deformable_Demo_3")->setEnabled(true);
	mGUI->mGuiSystem->findWidget<MyGUI::Widget>("Command_Deformable_Demo_4")->setEnabled(true);
	mGUI->mGuiSystem->findWidget<MyGUI::Widget>("Command_Deformable_Demo_5")->setEnabled(true);
	mGUI->mGuiSystem->findWidget<MyGUI::Widget>("Command_Deformable_Demo_6")->setEnabled(true);
	mGUI->mGuiSystem->findWidget<MyGUI::Widget>("Command_Deformable_Demo_7")->setEnabled(true);

	mGUI->mGuiSystem->findWidget<MyGUI::Widget>("Command_Playgroud_1")->setEnabled(true);
	mGUI->mGuiSystem->findWidget<MyGUI::Widget>("Command_Playgroud_2")->setEnabled(true);
	mGUI->mGuiSystem->findWidget<MyGUI::Widget>("Command_Playgroud_3")->setEnabled(true);
	mGUI->mGuiSystem->findWidget<MyGUI::Widget>("Command_Playgroud_4")->setEnabled(true);
	mGUI->mGuiSystem->findWidget<MyGUI::Widget>("Command_Playgroud_5")->setEnabled(true);

	mGUI->Command_Disable_Mini_Camera = true;
	mGUI->Command_Enable_Mini_Camera = false;
	mGUI->mGuiSystem->findWidget<MyGUI::Widget>("Command_Enable_Mini_Camera")->setEnabled(true);
	mGUI->mGuiSystem->findWidget<MyGUI::Widget>("Command_Disable_Mini_Camera")->setEnabled(false);
	mGUI->miniCameraWindow->setVisible(false);


	solidScreen();
}


void DPS::GUIeventHandler(void)
{
	if(mGUI->Command_Clear_Screen)
	{
		clearScreen();
	}
	if(mGUI->Command_Enable_FPS)
	{
		mGUI->Command_Enable_FPS = false;
		mTrayMgr->showFrameStats(OgreBites::TL_BOTTOMLEFT);
	}
	if(mGUI->Command_Disable_FPS)
	{
		mGUI->Command_Disable_FPS = false;
		mTrayMgr->hideFrameStats();
	}
	if(mGUI->Command_Quit)
	{
		mGUI->Command_Quit = false;
		mShutDown = true;
	}
	if(mGUI->Command_ScreenShot)
	{
		mGUI->Command_ScreenShot = false;
		mWindow->writeContentsToTimestampedFile("screenshot", ".jpg");
	}
	if(mGUI->Command_Disable_Slow)
	{
		mGUI->Command_Disable_Slow = false;
		mGUI->Command_Enable_Slow = false;
	}
	if(mGUI->Command_Reset_Camera)
	{
		resetCamera();
		mGUI->Command_Reset_Camera = false;
	}
	if(mGUI->Command_Cloth_Demo_1)
	{
		mGUI->Command_Cloth_Demo_1 = false;

		//clean screen before create new demo
		clearScreen();

		resetCamera(Ogre::Vector3(0.0f,8.0f,25.0f));

		//create demo
		dpsSoftbodyHelper->createClothDemo_1();

		//run demo after ceate
		runClothDome_1 = true;
	}
	if(mGUI->Command_Cloth_Demo_2)
	{
		mGUI->Command_Cloth_Demo_2 = false;

		//clean screen before create new demo
		clearScreen();

		
		//create demo
		resetCamera(Ogre::Vector3(0.0f,12.0f,25.0f));
		dpsSoftbodyHelper->createClothDemo_2();
		for(int i=0 ; i<26; i+=5)
		{
			dpsHelper->createCube(Ogre::Vector3(0,40+i,0),Ogre::Vector3(2,2,2),1);
		}

		//run demo after ceate
		runClothDome_2 = true;
	}
	if(mGUI->Command_Cloth_Demo_3)
	{
		mGUI->Command_Cloth_Demo_3 = false;

		//clean screen before create new demo
		clearScreen();

		//create demo
		resetCamera(Ogre::Vector3(0.0f,6.0f,25.0f));
		dpsSoftbodyHelper->createClothDemo_3();
		dpsHelper->createFixedSphere(Ogre::Vector3(0,5,0),0);

		//run demo after ceate
		runClothDome_3 = true;
	}
	if(mGUI->Command_Cloth_Demo_4)
	{
		mGUI->Command_Cloth_Demo_4 = false;

		//clean screen before create new demo
		clearScreen();

		//create demo
		resetCamera(Ogre::Vector3(0.0f,12.0f,25.0f));
		dpsSoftbodyHelper->createClothDemo_4(dpsHelper->createCubeAndReturnBody(Ogre::Vector3(0,15,-7.5),Ogre::Vector3(10,1,3),5));

		//run demo after ceate
		runClothDome_4 = true;
	}
	if(mGUI->Command_Cloth_Demo_5)
	{
		mGUI->Command_Cloth_Demo_5 = false;

		//clean screen before create new demo
		clearScreen();

		//create demo
		resetCamera(Ogre::Vector3(0.0f,12.0f,25.0f));
		dpsSoftbodyHelper->createClothDemo_5();

		//run demo after ceate
		runClothDome_5 = true;
	}
	if(mGUI->Command_Cloth_Demo_6)
	{
		mGUI->Command_Cloth_Demo_6 = false;

		//clean screen before create new demo
		clearScreen();

		//create demo
		resetCamera(Ogre::Vector3(0.0f,30.0f,150.0f));
		dpsSoftbodyHelper->createClothDemo_6();

		//run demo after ceate
		runClothDome_6 = true;
	}
	if(mGUI->Command_Cloth_Demo_7)
	{
		mGUI->Command_Cloth_Demo_7 = false;

		//clean screen before create new demo
		clearScreen();

		//create demo
		resetCamera(Ogre::Vector3(0.0f,45.0f,40.0f));
		dpsSoftbodyHelper->createClothDemo_7();

		//run demo after ceate
		runClothDome_7 = true;
	}


	//deformable demos
	if(mGUI->Command_Deformable_Demo_1)
	{
		mGUI->Command_Deformable_Demo_1 = false;

		//clean screen before create new demo
		clearScreen();

		//create demo
		resetCamera(Ogre::Vector3(0.0f,6.0f,40.0f));
		dpsSoftbodyHelper->createDeformDemo_1(btVector3(2,1,0));
		//dpsHelper->createCube(Ogre::Vector3(-5,1.5,0),Ogre::Vector3(4,4,4),1);
		dpsHelper->createMesh(Ogre::Vector3(-3,1,0),20,"softcube.mesh",Ogre::Vector3(2,2,2));

		//run demo after ceate
		runDeformDome_1 = true;
	}

	if(mGUI->Command_Deformable_Demo_2)
	{
		mGUI->Command_Deformable_Demo_2 = false;

		//clean screen before create new demo
		clearScreen();

		//create demo
		resetCamera(Ogre::Vector3(0.0f,10.0f,40.0f));
		dpsSoftbodyHelper->createDeformDemo_2(btVector3(0,2,0));
		dpsHelper->createMesh(Ogre::Vector3(-8,2,0),1,"Barrel.mesh",Ogre::Vector3(1,1,1));

		//run demo after ceate
		runDeformDome_2 = true;
	}
	if(mGUI->Command_Deformable_Demo_3)
	{
		mGUI->Command_Deformable_Demo_3 = false;

		//clean screen before create new demo
		clearScreen();

		//create demo
		resetCamera(Ogre::Vector3(0.0f,6.0f,20.0f));
		dpsSoftbodyHelper->createDeformDemo_3(btVector3(3,1,0));
		dpsHelper->createMesh(Ogre::Vector3(-4,2,0),1,"m845.mesh",Ogre::Vector3(2,2,2));

		//run demo after ceate
		runDeformDome_3 = true;
	}
	if(mGUI->Command_Deformable_Demo_4)
	{
		mGUI->Command_Deformable_Demo_4 = false;

		//clean screen before create new demo
		clearScreen();

		//create demo
		resetCamera(Ogre::Vector3(0.0f,6.0f,40.0f));
		dpsSoftbodyHelper->createDeformDemo_4(btVector3(0,3,0));

		//run demo after ceate
		runDeformDome_4 = true;
	}
	if(mGUI->Command_Deformable_Demo_5)
	{
		mGUI->Command_Deformable_Demo_5 = false;

		//clean screen before create new demo
		clearScreen();

		//create demo
		resetCamera(Ogre::Vector3(0.0f,17.0f,20.0f));
		dpsSoftbodyHelper->createDeformDemo_5(btVector3(0,20,0));

		//run demo after ceate
		runDeformDome_5 = true;
	}
	if(mGUI->Command_Deformable_Demo_6)
	{
		mGUI->Command_Deformable_Demo_6 = false;

		//clean screen before create new demo
		clearScreen();

		//create demo
		resetCamera(Ogre::Vector3(0.0f,6.0f,20.0f));
		dpsSoftbodyHelper->createDeformDemo_6(btVector3(0,1,0));

		//run demo after ceate
		runDeformDome_6 = true;
	}
	if(mGUI->Command_Deformable_Demo_7)
	{
		mGUI->Command_Deformable_Demo_7 = false;

		//clean screen before create new demo
		clearScreen();

		//create demo
		resetCamera(Ogre::Vector3(0.0f,6.0f,20.0f));
		dpsSoftbodyHelper->createGimpactBuuny();

		//might not needed. 
		bulletDebugScreen();

		//run demo after ceate
		runDeformDome_7 = true;
	}


	if(mGUI->Command_Playgroud_1)
	{
		mGUI->Command_Playgroud_1 = false;

		//clean screen before create new demo
		clearScreen();

		//create demo
		resetCamera(Ogre::Vector3(0.0f,6.0f,50.0f));
		dpsSoftbodyHelper->createPlayground_1(btVector3(0,10,0));

		//run demo after ceate
		runPlaygroud_1 = true;
	}
	if(mGUI->Command_Playgroud_2)
	{
		mGUI->Command_Playgroud_2 = false;

		//clean screen before create new demo
		clearScreen();

		//create demo
		resetCamera(Ogre::Vector3(0.0f,6.0f,40.0f));
		dpsSoftbodyHelper->createPlayground_2();
		dpsHelper->createMesh(Ogre::Vector3(0,2,0),50,"defCube.mesh",Ogre::Vector3(10,2,10));
 		dpsHelper->createMesh(Ogre::Vector3(0,10,0),10,"defCube.mesh",Ogre::Vector3(10,2,10));
 		dpsHelper->createMesh(Ogre::Vector3(0,17,0),10,"defCube.mesh",Ogre::Vector3(10,2,10));


		//run demo after ceate
		runPlaygroud_2 = true;
	}
	if(mGUI->Command_Playgroud_3)
	{
		mGUI->Command_Playgroud_3 = false;

		//clean screen before create new demo
		clearScreen();

		//create demo
		resetCamera(Ogre::Vector3(0.0f,10.0f,40.0f));
		dpsSoftbodyHelper->createPlayground_3();
		dpsHelper->createMesh(Ogre::Vector3(-10,0.1,0),50,"car.mesh",Ogre::Vector3(1,1,1));
		//createFromMesh(m_playgroundManualObject_3, m_playgroundBody_3, btVector3(0,0,0), btVector3(0,0,0), btVector3(1,1,1));

		if (playgroundCount == 0)
		{
			dpsHelper->createMesh(Ogre::Vector3(-5,50,0),50,"defCube.mesh",Ogre::Vector3(10,2,10));
			dpsHelper->createMesh(Ogre::Vector3(15,50,0),50,"defCube.mesh",Ogre::Vector3(10,2,10));
			playgroundCount++;
		}
		else if(playgroundCount == 1)
		{
			dpsHelper->createMesh(Ogre::Vector3(-10,50,0),50,"defCube.mesh",Ogre::Vector3(10,2,10));
			dpsHelper->createMesh(Ogre::Vector3(10,50,0),50,"defCube.mesh",Ogre::Vector3(10,2,10));
			playgroundCount++;
		}
		else if (playgroundCount == 2)
		{
			dpsHelper->createMesh(Ogre::Vector3(-15,50,0),50,"defCube.mesh",Ogre::Vector3(10,2,10));
			dpsHelper->createMesh(Ogre::Vector3(5,50,0),50,"defCube.mesh",Ogre::Vector3(10,2,10));
			playgroundCount++;
		}
		else if(playgroundCount == 3)
		{
			dpsHelper->createMesh(Ogre::Vector3(-10,50,0),50,"defCube.mesh",Ogre::Vector3(2,10,10));
			dpsHelper->createMesh(Ogre::Vector3(10,50,0),50,"defCube.mesh",Ogre::Vector3(2,10,10));
			playgroundCount = 0;
		}

		//run demo after ceate
		runPlaygroud_3 = true;
	}
	if(mGUI->Command_Playgroud_4)
	{
		mGUI->Command_Playgroud_4 = false;

		//clean screen before create new demo
		clearScreen();

		//create demo
		resetCamera(Ogre::Vector3(0.0f,10.0f,40.0f));
		dpsSoftbodyHelper->createPlayground_4(btVector3(8,7,0));
		dpsHelper->createMesh(Ogre::Vector3(-8,5,0),50,"cylinder_high.mesh",Ogre::Vector3(0.7,0.7,0.7));
		dpsHelper->createMesh(Ogre::Vector3(-8,50,0),400,"defCube.mesh",Ogre::Vector3(10,2,10));
		dpsHelper->createMesh(Ogre::Vector3(6,50,0),400,"defCube.mesh",Ogre::Vector3(10,2,10));


		//run demo after ceate
		runPlaygroud_4 = true;
	}
}


void DPS::solidScreen(void)
{
	vp->getCamera()->setPolygonMode(Ogre::PM_SOLID);
	mGUI->Command_Bullet_Debug_Mode = false;
	mGUI->mGuiSystem->findWidget<MyGUI::Widget>("Command_Solid")->setEnabled(false);
	mGUI->mGuiSystem->findWidget<MyGUI::Widget>("Command_Wireframe")->setEnabled(true);
	mGUI->mGuiSystem->findWidget<MyGUI::Widget>("Command_Points")->setEnabled(true);
	mGUI->mGuiSystem->findWidget<MyGUI::Widget>("Command_Bullet_Debug_Mode")->setEnabled(true);
}


void DPS::bulletDebugScreen(void)
{
	mGUI->Command_Bullet_Debug_Mode = true;
	mGUI->mGuiSystem->findWidget<MyGUI::Widget>("Command_Solid")->setEnabled(true);
	mGUI->mGuiSystem->findWidget<MyGUI::Widget>("Command_Wireframe")->setEnabled(true);
	mGUI->mGuiSystem->findWidget<MyGUI::Widget>("Command_Points")->setEnabled(true);
	mGUI->mGuiSystem->findWidget<MyGUI::Widget>("Command_Bullet_Debug_Mode")->setEnabled(false);
}


void DPS::demoController(void)
{
	//cloth demos
	if(runClothDome_1)
	{
		dpsSoftbodyHelper->updateSoftBody(dpsSoftbodyHelper->m_clothManualObject_1, dpsSoftbodyHelper->m_clothBody_1);
	}
	if(runClothDome_2)
	{
		dpsSoftbodyHelper->updateSoftBody(dpsSoftbodyHelper->m_clothManualObject_2, dpsSoftbodyHelper->m_clothBody_2);
	}
	if(runClothDome_3)
	{
		dpsSoftbodyHelper->updateSoftBody(dpsSoftbodyHelper->m_clothManualObject_3, dpsSoftbodyHelper->m_clothBody_3);
	}
	if(runClothDome_4)
	{
		dpsSoftbodyHelper->updateSoftBody(dpsSoftbodyHelper->m_clothManualObject_4, dpsSoftbodyHelper->m_clothBody_4);
	}
	if(runClothDome_5)
	{
		dpsSoftbodyHelper->updateSoftBody(dpsSoftbodyHelper->m_clothManualObject_5_0, dpsSoftbodyHelper->m_clothBody_5_0);
		dpsSoftbodyHelper->updateSoftBody(dpsSoftbodyHelper->m_clothManualObject_5_1, dpsSoftbodyHelper->m_clothBody_5_1);
	}
	if(runClothDome_6)
	{
		dpsSoftbodyHelper->updateSoftBody(dpsSoftbodyHelper->m_clothManualObject_6_0, dpsSoftbodyHelper->m_clothBody_6_0);
		dpsSoftbodyHelper->updateSoftBody(dpsSoftbodyHelper->m_clothManualObject_6_1, dpsSoftbodyHelper->m_clothBody_6_1);
		dpsSoftbodyHelper->updateSoftBody(dpsSoftbodyHelper->m_clothManualObject_6_2, dpsSoftbodyHelper->m_clothBody_6_2);
	}
	if(runClothDome_7)
	{
		dpsSoftbodyHelper->updateSoftBody(dpsSoftbodyHelper->m_clothManualObject_7_0, dpsSoftbodyHelper->m_clothBody_7_0);
		dpsSoftbodyHelper->updateSoftBody(dpsSoftbodyHelper->m_clothManualObject_7_1, dpsSoftbodyHelper->m_clothBody_7_1);
		dpsSoftbodyHelper->updateSoftBody(dpsSoftbodyHelper->m_clothManualObject_7_2, dpsSoftbodyHelper->m_clothBody_7_2);
	}


	//deformation
	if(runDeformDome_1)
	{
		dpsSoftbodyHelper->updateSoftBody(dpsSoftbodyHelper->m_deformManualObject_1, dpsSoftbodyHelper->m_deformBody_1);
	}
	if(runDeformDome_2)
	{
		dpsSoftbodyHelper->updateSoftBody(dpsSoftbodyHelper->m_deformManualObject_2, dpsSoftbodyHelper->m_deformBody_2);
	}
	if(runDeformDome_3)
	{
		dpsSoftbodyHelper->updateSoftBody(dpsSoftbodyHelper->m_deformManualObject_3, dpsSoftbodyHelper->m_deformBody_3);
	}
	if(runDeformDome_4)
	{
		dpsSoftbodyHelper->updateSoftBody(dpsSoftbodyHelper->m_deformManualObject_4, dpsSoftbodyHelper->m_deformBody_4);
	}
	if(runDeformDome_5)
	{
		dpsSoftbodyHelper->updateSoftBody(dpsSoftbodyHelper->m_deformManualObject_5, dpsSoftbodyHelper->m_deformBody_5);
	}
	if(runDeformDome_6)
	{
		dpsSoftbodyHelper->updateSoftBody(dpsSoftbodyHelper->m_deformManualObject_6, dpsSoftbodyHelper->m_deformBody_6);
	}
// 	if(runDeformDome_7)
// 	{
// 		dpsSoftbodyHelper->updateSoftBody(dpsSoftbodyHelper->m_deformManualObject_7, dpsSoftbodyHelper->m_deformBody_7);
// 	}

	//playground
	if(runPlaygroud_1)
	{
		dpsSoftbodyHelper->updateCar(dpsSoftbodyHelper->m_playgroundManualObject_fl, dpsSoftbodyHelper->m_playgroundBody_fl);
		dpsSoftbodyHelper->updateCar(dpsSoftbodyHelper->m_playgroundManualObject_fr, dpsSoftbodyHelper->m_playgroundBody_fr);
		dpsSoftbodyHelper->updateCar(dpsSoftbodyHelper->m_playgroundManualObject_rl, dpsSoftbodyHelper->m_playgroundBody_rl);
		dpsSoftbodyHelper->updateCar(dpsSoftbodyHelper->m_playgroundManualObject_rr, dpsSoftbodyHelper->m_playgroundBody_rr);
		dpsSoftbodyHelper->updateCar(dpsSoftbodyHelper->m_playgroundManualObject_carBody, dpsSoftbodyHelper->m_playgroundBody_carBody);
	}
	if(runPlaygroud_2)
	{
		dpsSoftbodyHelper->updateCar(dpsSoftbodyHelper->m_playgroundManualObject_2_1, dpsSoftbodyHelper->m_playgroundBody_2_1);
 		dpsSoftbodyHelper->updateCar(dpsSoftbodyHelper->m_playgroundManualObject_2_2, dpsSoftbodyHelper->m_playgroundBody_2_2);
	}
	if(runPlaygroud_3)
	{
		dpsSoftbodyHelper->updateCar(dpsSoftbodyHelper->m_playgroundManualObject_3, dpsSoftbodyHelper->m_playgroundBody_3);
	}
	if(runPlaygroud_4)
	{
		dpsSoftbodyHelper->updateCar(dpsSoftbodyHelper->m_playgroundManualObject_4, dpsSoftbodyHelper->m_playgroundBody_4);
	}
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
	if (arg.key == OIS::KC_SPACE) 
	{
		GimpactRayCallBack();
	}
	return BaseApplication::keyPressed(arg);
}


void DPS::GimpactRayCallBack(void)
{
	Ogre::Vector3 camPos = mCamera->getPosition();
	Ogre::Vector3 viewPos = mCamera->getDirection();
	btVector3 to = btVector3(viewPos.x,viewPos.y,viewPos.z);
	to.normalize();
	to *= 1000;
	MyClosestRayResultCallback rayCallback(btVector3(camPos.x,camPos.y,camPos.z), to);
	Globals::phyWorld->rayTest(btVector3(camPos.x,camPos.y,camPos.z), to, rayCallback);

	if (rayCallback.hasHit())
	{
		btVector3 hit = rayCallback.m_hitPointWorld;
		btCollisionObject obj = *rayCallback.m_collisionObject;
		//downcast
		btRigidBody* body = btRigidBody::upcast(&obj);
		if (body)
		{
			hit_body = body;
			if (rayCallback.m_hitTriangleShape)
			{
				bool triangle_ok = process_triangle(hit_body->getCollisionShape(),
					rayCallback.m_hitTriangleIndex);
			}
		}
	}
}


// base on Bullet engine forum Example
bool DPS::process_triangle(btCollisionShape* shape, int hitTriangleIndex)
{
	btStridingMeshInterface * meshInterface = NULL;

	if (shape->getShapeType() == GIMPACT_SHAPE_PROXYTYPE)
	{
		meshInterface = (static_cast<btGImpactMeshShape*>(shape))->getMeshInterface();
	}
	else if (shape->getShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE)
	{
		meshInterface = (static_cast<btBvhTriangleMeshShape*>(shape))->getMeshInterface();
	}
	else
	{
		return false;
	}

	if (!meshInterface) return false;

	unsigned char *vertexbase;
	int numverts;
	PHY_ScalarType type;
	int stride;
	unsigned char *indexbase;
	int indexstride;
	int numfaces;
	PHY_ScalarType indicestype;

	meshInterface->getLockedVertexIndexBase(
		&vertexbase,
		numverts,
		type,
		stride,
		&indexbase,
		indexstride,
		numfaces,
		indicestype,
		0);

	unsigned int * gfxbase = (unsigned int*)(indexbase+hitTriangleIndex*indexstride);
	const btVector3 & meshScaling = shape->getLocalScaling();
	btVector3 triangle_v[3];

	for (int j=2;j>=0;j--)
	{
		int graphicsindex = indicestype==PHY_SHORT?((unsigned short*)gfxbase)[j]:gfxbase[j];

		btScalar * graphicsbase = (btScalar*)(vertexbase+graphicsindex*stride);

		if (shape->getShapeType() == GIMPACT_SHAPE_PROXYTYPE)
		{
			graphicsbase[0] *= btScalar(0.85);
			graphicsbase[1] *= btScalar(0.85);
			graphicsbase[2] *= btScalar(0.85);
		}
	}

	meshInterface->unLockVertexBase(0);

	if (shape->getShapeType() == GIMPACT_SHAPE_PROXYTYPE)
	{
		btGImpactMeshShape * gimp_shape = static_cast<btGImpactMeshShape*>(shape);
		gimp_shape->postUpdate();
	}
	return true;
}


void DPS::deleteOgreEntities(void)
{
	mSceneMgr->destroyAllManualObjects();
	mSceneMgr->destroyAllEntities();
	mSceneMgr->destroyAllLights();

	dpsHelper->createWorld();
}


bool DPS::mouseMoved(const OIS::MouseEvent &arg)
{
	//mouse move on screen
	bool mouseOnWidget = MyGUI::InputManager::getInstance().injectMouseMove(arg.state.X.abs, arg.state.Y.abs, arg.state.Z.abs);

	//MyGUI::IntPoint mousePos = MyGUI::InputManager::getInstance().getMousePosition();

	if (arg.state.buttonDown(OIS::MB_Right))
	{
		mCameraMan->injectMouseMove(arg);
	}

	return BaseApplication::mouseMoved(arg);
}


bool DPS::mousePressed(const OIS::MouseEvent &arg, OIS::MouseButtonID id)
{
	bool mouseOnWidget = MyGUI::InputManager::getInstance().injectMousePress(arg.state.X.abs, arg.state.Y.abs, MyGUI::MouseButton::Enum(id));

	if(!mouseOnWidget)
	{
		if (id == OIS::MB_Left)
		{
			MyGUI::IntPoint mousePos = MyGUI::InputManager::getInstance().getMousePosition();

			//do the raycast
			Ogre::Ray mouseRay = mCamera->getCameraToViewportRay(mousePos.left/float(arg.state.width),mousePos.top/float(arg.state.height));
			Ogre::RaySceneQuery* mRayScnQuery = mSceneMgr->createRayQuery(Ogre::Ray());
			
			mRayScnQuery->setRay(mouseRay);
			mRayScnQuery->setSortByDistance(true, 1);

			Ogre::RaySceneQueryResult &result = mRayScnQuery->execute();
			Ogre::RaySceneQueryResult::iterator iter = result.begin();

			if (iter != result.end() && !iter->worldFragment)
			{
				rayObject = iter->movable;
				rayNode = rayObject->getParentSceneNode();
				mGUI->miniCameraWindow->setVisible(true);
				mGUI->mGuiSystem->findWidget<MyGUI::Widget>("Command_Enable_Mini_Camera")->setEnabled(false);
				mGUI->mGuiSystem->findWidget<MyGUI::Widget>("Command_Disable_Mini_Camera")->setEnabled(true);
			}
		}
	}

	return BaseApplication::mousePressed(arg, id);
}


bool DPS::mouseReleased(const OIS::MouseEvent &arg, OIS::MouseButtonID id)
{
	MyGUI::InputManager::getInstance().injectMouseRelease(arg.state.X.abs, arg.state.Y.abs, MyGUI::MouseButton::Enum(id));

	return BaseApplication::mouseReleased(arg, id);
}


void DPS::resetCamera(void)
{
	mCamera->setPosition(0,5,20);
	mCamera->lookAt(0,5,-10);
}


void DPS::resetCamera(Ogre::Vector3 camPos)
{
	mCamera->setPosition(camPos);
	mCamera->lookAt(Ogre::Vector3(camPos.x,camPos.y,-camPos.z));
}


void DPS::setMiniCamPosition(Ogre::Vector3 camPos)
{
	miniCam->setPosition(10+camPos);
	miniCam->lookAt(camPos);
	miniCam->setNearClipDistance(5);
}


bool DPS::leapMotionInit(void)
{
	leapMotionController.addListener(leapMotionListener);

	fingerName_0 = "finger_" + convertInt(leapMotionCounter);
	fingerName_1 = "finger_" + convertInt(leapMotionCounter+1);

	dpsHelper->createLeapMotionSphere_0(fingerName_0,Ogre::Vector3(-12.f,100.0f,0.0f));
 	dpsHelper->createLeapMotionSphere_1(fingerName_1,Ogre::Vector3(-9.f,100.0f,0.0f));

	leapMotionRunning = true;
	leapMotionCounter += 2;

	return true;
}


std::string DPS::convertInt(int number)
{
	if (number == 0)
		return "0";
	std::string temp="";
	std::string returnValue="";
	while (number>0)
	{
		temp+=number%10+48;
		number/=10;
	}
	for (int i=0;i<temp.length();i++)
		returnValue+=temp[temp.length()-i-1];
	return returnValue;
}


void DPS::leapMotionUpdate(void)
{
	if(leapMotionController.isConnected())
	{
		Leap::Frame leapFrameData = leapMotionController.frame();
		Leap::Hand leapHand = leapFrameData.hands().rightmost();

		Leap::FingerList fingerListNow = leapFrameData.hands()[0].fingers();
		std::vector<Ogre::Vector3> fingerPositions;

		for(int i = 0; i < fingerListNow.count(); i++)
		{
			Leap::Vector fp = fingerListNow[i].tipPosition();
			fingerPositions.push_back(Ogre::Vector3(fp.x, fp.y, fp.z));
		}

		if(fingerPositions.size() == 1)
		{
			//finger 1
			Leap::Finger finger0 = leapHand.fingers()[0];
			Leap::Vector fp0 = finger0.tipPosition();

			if (fp0.y < 105)
			{
				fp0.y = 105;
			}

			mSceneMgr->getSceneNode(fingerName_0)->setPosition(Ogre::Vector3(fp0.x * 0.2,(fp0.y-100) * 0.2,fp0.z * 0.2));
			Ogre::Vector3 absPos0 = dpsHelper->sphereNode_0->_getDerivedPosition();
			Ogre::Quaternion absQuat0 = dpsHelper->sphereNode_0->_getDerivedOrientation();

			btTransform tr0 = dpsHelper->sphereBody_0->getWorldTransform();
			tr0.setOrigin(btVector3(absPos0.x,absPos0.y,absPos0.z));
			tr0.setRotation(btQuaternion(absQuat0.x, absQuat0.y, absQuat0.z, absQuat0.w));
			dpsHelper->sphereBody_0->applyCentralForce(btVector3(0,0,0));
			dpsHelper->sphereBody_0->setLinearVelocity(btVector3(0,0,0));
			dpsHelper->sphereBody_0->activate(true);
			dpsHelper->sphereBody_0->setWorldTransform(tr0);

			//finger 2
			mSceneMgr->getSceneNode(fingerName_1)->setPosition(Ogre::Vector3(-9.f,100.0f,0.0f));
			btTransform tr1 = dpsHelper->sphereBody_1->getWorldTransform();
			tr1.setOrigin(btVector3(-9.f,100.0f,0.0f));
			dpsHelper->sphereBody_1->applyCentralForce(btVector3(0,0,0));
			dpsHelper->sphereBody_1->setLinearVelocity(btVector3(0,0,0));
			dpsHelper->sphereBody_1->setWorldTransform(tr1);
		}

		if(fingerPositions.size() == 2)
		{
			//finger 1
			Leap::Finger finger0 = leapHand.fingers()[0];
			Leap::Vector fp0 = finger0.tipPosition();

			if (fp0.y < 105)
			{
				fp0.y = 105;
			}

			mSceneMgr->getSceneNode(fingerName_0)->setPosition(Ogre::Vector3(fp0.x * 0.2,(fp0.y-100) * 0.2,fp0.z * 0.2));
			Ogre::Vector3 absPos0 = dpsHelper->sphereNode_0->_getDerivedPosition();
			Ogre::Quaternion absQuat0 = dpsHelper->sphereNode_0->_getDerivedOrientation();

			btTransform tr0 = dpsHelper->sphereBody_0->getWorldTransform();
			tr0.setOrigin(btVector3(absPos0.x,absPos0.y,absPos0.z));
			tr0.setRotation(btQuaternion(absQuat0.x, absQuat0.y, absQuat0.z, absQuat0.w));
			dpsHelper->sphereBody_0->applyCentralForce(btVector3(0,0,0));
			dpsHelper->sphereBody_0->setLinearVelocity(btVector3(0,0,0));
			dpsHelper->sphereBody_0->activate(true);
			dpsHelper->sphereBody_0->setWorldTransform(tr0);

			//finger 2
			Leap::Finger finger1 = leapHand.fingers()[1];
			Leap::Vector fp1 = finger1.tipPosition();

			if (fp1.y < 105)
			{
				fp1.y = 105;
			}

			mSceneMgr->getSceneNode(fingerName_1)->setPosition(Ogre::Vector3(fp1.x * 0.2,(fp1.y-100) * 0.2,fp1.z * 0.2));
			Ogre::Vector3 absPos1 = dpsHelper->sphereNode_1->_getDerivedPosition();
			Ogre::Quaternion absQuat1 = dpsHelper->sphereNode_1->_getDerivedOrientation();

			btTransform tr1 = dpsHelper->sphereBody_1->getWorldTransform();
			tr1.setOrigin(btVector3(absPos1.x,absPos1.y,absPos1.z));
			tr1.setRotation(btQuaternion(absQuat1.x, absQuat1.y, absQuat1.z, absQuat1.w));
			dpsHelper->sphereBody_1->applyCentralForce(btVector3(0,0,0));
			dpsHelper->sphereBody_1->setLinearVelocity(btVector3(0,0,0));
			dpsHelper->sphereBody_1->activate(true);
			dpsHelper->sphereBody_1->setWorldTransform(tr1);
		}

		if(leapFrameData.hands().isEmpty() || fingerPositions.size() > 2)
		{
			mSceneMgr->getSceneNode(fingerName_0)->setPosition(Ogre::Vector3(-12.f,100.0f,0.0f));
			mSceneMgr->getSceneNode(fingerName_1)->setPosition(Ogre::Vector3(-9.f,100.0f,0.0f));

			btTransform tr0 = dpsHelper->sphereBody_0->getWorldTransform();
			tr0.setOrigin(btVector3(-12.f,100.0f,0.0f));
			dpsHelper->sphereBody_0->applyCentralForce(btVector3(0,0,0));
			dpsHelper->sphereBody_0->setLinearVelocity(btVector3(0,0,0));
			dpsHelper->sphereBody_0->setWorldTransform(tr0);

			btTransform tr1 = dpsHelper->sphereBody_1->getWorldTransform();
			tr1.setOrigin(btVector3(-9.f,100.0f,0.0f));
			dpsHelper->sphereBody_1->applyCentralForce(btVector3(0,0,0));
			dpsHelper->sphereBody_1->setLinearVelocity(btVector3(0,0,0));
			dpsHelper->sphereBody_1->setWorldTransform(tr1);
		}
	}
	else
	{
		mSceneMgr->getSceneNode(fingerName_0)->setPosition(Ogre::Vector3(-12.f,100.0f,0.0f));
		mSceneMgr->getSceneNode(fingerName_1)->setPosition(Ogre::Vector3(-9.f,100.0f,0.0f));

		btTransform tr0 = dpsHelper->sphereBody_0->getWorldTransform();
		tr0.setOrigin(btVector3(-12.f,100.0f,0.0f));
		dpsHelper->sphereBody_0->applyCentralForce(btVector3(0,0,0));
		dpsHelper->sphereBody_0->setLinearVelocity(btVector3(0,0,0));
		dpsHelper->sphereBody_0->setWorldTransform(tr0);

		btTransform tr1 = dpsHelper->sphereBody_1->getWorldTransform();
		tr1.setOrigin(btVector3(-9.f,100.0f,0.0f));
		dpsHelper->sphereBody_1->applyCentralForce(btVector3(0,0,0));
		dpsHelper->sphereBody_1->setLinearVelocity(btVector3(0,0,0));
		dpsHelper->sphereBody_1->setWorldTransform(tr1);
	}
}


void DPS::leapMotionCleanup(void)
{
	leapMotionController.removeListener(leapMotionListener);
	leapMotionRunning = false;
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