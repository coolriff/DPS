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

	OGRE_DELETE mTerrainGroup;
	OGRE_DELETE mTerrainGlobals;

	ControllerManager::getSingleton().destroyController(mLightController);
	MeshManager::getSingleton().remove("ground");
	MeshManager::getSingleton().remove("grass");
}




// void DPS::createCamera(void)
// {
//     // create the camera
//     mCamera = mSceneMgr->createCamera("PlayerCam");
//     // set its position, direction  
//     mCamera->setPosition(Ogre::Vector3(0,10,500));
//     mCamera->lookAt(Ogre::Vector3(0,0,0));
//     // set the near clip distance
//     mCamera->setNearClipDistance(5);
//  
//     mCameraMan = new OgreBites::SdkCameraMan(mCamera);   // create a default camera controller
// }
// 
// 
// void DPS::createViewports(void)
// {
//     // Create one viewport, entire window
//     Ogre::Viewport* vp = mWindow->addViewport(mCamera);
//     vp->setBackgroundColour(Ogre::ColourValue(0,0,0));
//     // Alter the camera aspect ratio to match the viewport
//     mCamera->setAspectRatio(Ogre::Real(vp->getActualWidth()) / Ogre::Real(vp->getActualHeight()));    
// }


void DPS::createScene(void)
{
	ParticleSystem::setDefaultNonVisibleUpdateTimeout(5);
	// Basic Ogre stuff.
	//mSceneMgr->setAmbientLight(ColourValue(0.3, 0.3, 0.3));
	//mSceneMgr->createLight()->setPosition(20, 80, 50);
// 	mCamera->setPosition(Vector3(0,20,20));
// 	mCamera->lookAt(Vector3(0,20,0));
// 	mCamera->setNearClipDistance(0.05f);
// 	//LogManager::getSingleton().setLogDetail(LL_BOREME);

	mCamera->setPosition(Ogre::Vector3(1683, 50, 2116));
	mCamera->lookAt(Ogre::Vector3(1963, 50, 1660));
	mCamera->setNearClipDistance(0.1);
	mCamera->setFarClipDistance(50000);

	if (mRoot->getRenderSystem()->getCapabilities()->hasCapability(Ogre::RSC_INFINITE_FAR_PLANE))
	{
		mCamera->setFarClipDistance(0);   // enable infinite far clip distance if we can
	}

	dpsHelper = std::make_shared<DPSHelper>(Globals::phyWorld, mCamera, mSceneMgr);
	dpsSoftbodyHelper = std::make_shared<DPSSoftBodyHelper>(Globals::phyWorld, mCamera, mSceneMgr);

	//dpsHelper->createGround();

	// Main light in scene
	//dpsHelper->createDirectionLight("mainLight",Ogre::Vector3(60,180,100),Ogre::Vector3(-60,-80,-100));
	//dpsHelper->createDirectionLight("mainLight1",Ogre::Vector3(0,200,0),Ogre::Vector3(0,0,0));

	// Debug drawing
	Globals::dbgdraw = new BtOgre::DebugDrawer(mSceneMgr->getRootSceneNode(), Globals::phyWorld);
	Globals::phyWorld->setDebugDrawer(Globals::dbgdraw);

	//mSceneMgr->setSkyBox(true, "Examples/SpaceSkyBox");

// 	Ogre::Plane plane;
// 	plane.d = 100;
// 	plane.normal = Ogre::Vector3::NEGATIVE_UNIT_Y;
// 
// 	//mSceneMgr->setSkyDome(true, "Examples/CloudySky", 5, 8, 500);
// 	mSceneMgr->setSkyPlane(true, plane, "Examples/SpaceSkyPlane", 500, 20, true, 0.5, 150, 150);
	//mSceneMgr->setSkyPlane(true, Plane(0, -1, 0, 5000), "Examples/SpaceSkyPlane", 10000, 3);
	mSceneMgr->setSkyBox(true, "Examples/SpaceSkyBox", 5000);


	//mSceneMgr->getRootSceneNode()->attachObject(mSceneMgr->createEntity("Dragon", "dragon.mesh"));

	Ogre::Vector3 pos = Ogre::Vector3(1500,200,2000);
	Ogre::Quaternion rot = Ogre::Quaternion::IDENTITY;
	Ogre::Entity *entDrangon = mSceneMgr->createEntity("Dragon","dragon.mesh");
	entDrangon->setCastShadows(true);
	Ogre::SceneNode* dragon = mSceneMgr->getRootSceneNode()->createChildSceneNode(pos,rot);
	dragon->attachObject(entDrangon);
	//dragon->attachObject(mSceneMgr->createParticleSystem("Smoke", "Examples/Smoke"));

	Ogre::Entity *entFire1 = mSceneMgr->createEntity("defCube.mesh");
	Ogre::SceneNode* fire1 = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	//dpsHelper->setColor(entFire1, Ogre::Vector3(0.3021f,0.3308f,0.3671f));
	fire1->attachObject(entFire1);
	fire1->setPosition(Ogre::Vector3(1500,30,1920));
	fireOnCube_1 = mSceneMgr->createParticleSystem("Smoke", "Examples/Smoke");
	fire1->attachObject(fireOnCube_1);

// 	Ogre::Entity *entFire2 = mSceneMgr->createEntity("defCube.mesh");
// 	Ogre::SceneNode* fire2 = mSceneMgr->getRootSceneNode()->createChildSceneNode();
// 	//dpsHelper->setColor(entFire2, Ogre::Vector3(0.3021f,0.3308f,0.3671f));
// 	fire2->attachObject(entFire2);
// 	fire2->setPosition(Ogre::Vector3(0,0,-20));
// 	fireOnCube_2 = mSceneMgr->createParticleSystem("Smoke", "Examples/Smoke");
// 	fire2->attachObject(fireOnCube_2);

	Ogre::Vector3 mTerrainPos = Ogre::Vector3(0,0,0);

	Ogre::Entity* e1= mSceneMgr->createEntity("tudorhouse.mesh");
	Ogre::Vector3 entPos1 = Ogre::Vector3(mTerrainPos.x + 2043, 65, mTerrainPos.z + 1715);
	Ogre::SceneNode* sn1 = mSceneMgr->getRootSceneNode()->createChildSceneNode(entPos1, rot);
	sn1->setScale(Vector3(0.12, 0.12, 0.12));
	sn1->attachObject(e1);

	Ogre::Entity* e2= mSceneMgr->createEntity("tudorhouse.mesh");
	Ogre::Vector3 entPos2 = Ogre::Vector3(mTerrainPos.x + 1850, 65, mTerrainPos.z + 1478);
	Ogre::SceneNode* sn2 = mSceneMgr->getRootSceneNode()->createChildSceneNode(entPos2, rot);
	sn2->setScale(Vector3(0.12, 0.12, 0.12));
	sn2->attachObject(e2);

	Ogre::Entity* e3= mSceneMgr->createEntity("tudorhouse.mesh");
	Ogre::Vector3 entPos3 = Ogre::Vector3(mTerrainPos.x + 1970, 85, mTerrainPos.z + 2180);
	Ogre::SceneNode* sn3 = mSceneMgr->getRootSceneNode()->createChildSceneNode(entPos3, rot);
	sn3->setScale(Vector3(0.12, 0.12, 0.12));
	sn3->attachObject(e3);




	Ogre::Entity* razor1 = mSceneMgr->createEntity("Razor", "razor.mesh");
	Ogre::SceneNode* entRazor1 = mSceneMgr->getRootSceneNode()->createChildSceneNode(Ogre::Vector3(1500,200,1420), rot);
	entRazor1->attachObject(razor1);

	Ogre::Entity* RZR_001 = mSceneMgr->createEntity("RZR-001", "RZR-002.mesh");
	Ogre::SceneNode* entRZR_001 = mSceneMgr->getRootSceneNode()->createChildSceneNode(Ogre::Vector3(1600,200,1420), rot);
	entRZR_001->attachObject(RZR_001);

	Ogre::Entity* RZR_002 = mSceneMgr->createEntity("RZR-002", "RZR-002.mesh");
	Ogre::SceneNode* entRZR_002 = mSceneMgr->getRootSceneNode()->createChildSceneNode(Ogre::Vector3(2000,200,1420), rot);
	entRZR_002->attachObject(RZR_002);
	entRZR_002->scale(Ogre::Vector3(5.0f,5.0f,5.0f));

	// create a particle system with 200 quota, then set its material and dimensions
	ParticleSystem* thrusters = mSceneMgr->createParticleSystem(25);
	thrusters->setMaterialName("Examples/Flare");
	thrusters->setDefaultDimensions(25, 25);

	// create two emitters for our thruster particle system
	for (unsigned int i = 0; i < 2; i++)
	{
		ParticleEmitter* emitter = thrusters->addEmitter("Point");  // add a point emitter

		// set the emitter properties
		emitter->setAngle(Degree(3));
		emitter->setTimeToLive(0.5);
		emitter->setEmissionRate(25);
		emitter->setParticleVelocity(25);
		emitter->setDirection(Vector3::NEGATIVE_UNIT_Z);
		emitter->setColour(ColourValue::White, ColourValue::Red);
		emitter->setPosition(Vector3(i == 0 ? 5.7 : -18, 0, 0));
	}

	// attach our thruster particles to the rear of the ship
	Ogre::SceneNode* thrusterParticles = entRazor1->createChildSceneNode(Vector3(0, 6.5, -67));
	thrusterParticles->attachObject(thrusters);
	//mSceneMgr->getRootSceneNode()->createChildSceneNode(Vector3(0, 6.5, -67))->attachObject(thrusters);


	// Play with startup Texture Filtering options
	// Note: Pressing T on runtime will discarde those settings
	//  Ogre::MaterialManager::getSingleton().setDefaultTextureFiltering(Ogre::TFO_ANISOTROPIC);
	//  Ogre::MaterialManager::getSingleton().setDefaultAnisotropy(7);

	Ogre::Vector3 lightdir(0.55, -0.3, 0.75);
	lightdir.normalise();

	Ogre::Light* light = mSceneMgr->createLight("tstLight");
	light->setType(Ogre::Light::LT_DIRECTIONAL);
	light->setDirection(lightdir);
	light->setDiffuseColour(Ogre::ColourValue::White);
	//light->setSpecularColour(Ogre::ColourValue(0.4, 0.4, 0.4));

	mSceneMgr->setAmbientLight(Ogre::ColourValue(0.2, 0.2, 0.2));

	mTerrainGlobals = OGRE_NEW Ogre::TerrainGlobalOptions();

	mTerrainGroup = OGRE_NEW Ogre::TerrainGroup(mSceneMgr, Ogre::Terrain::ALIGN_X_Z, 513, 12000.0f);
	mTerrainGroup->setFilenameConvention(Ogre::String("BasicTutorial3Terrain"), Ogre::String("dat"));
	mTerrainGroup->setOrigin(Ogre::Vector3::ZERO);

	configureTerrainDefaults(light);

	for (long x = 0; x <= 0; ++x)
		for (long y = 0; y <= 0; ++y)
			defineTerrain(x, y);

	// sync load since we want everything in place when we start
	mTerrainGroup->loadAllTerrains(true);

	if (mTerrainsImported)
	{
		Ogre::TerrainGroup::TerrainIterator ti = mTerrainGroup->getTerrainIterator();
		while(ti.hasMoreElements())
		{
			Ogre::Terrain* t = ti.getNext()->instance;
			initBlendMaps(t);
		}
	}

	mTerrainGroup->freeTemporaryResources();

	//Ogre::ColourValue fadeColour(0.9, 0.9, 0.9);
	//mSceneMgr->setFog(Ogre::FOG_LINEAR, fadeColour, 0.0, 10, 1200);
	//mWindow->getViewport(0)->setBackgroundColour(fadeColour);

	createGrassMesh();
	Entity* grass = mSceneMgr->createEntity("Grass", "grass");

	// create a static geometry field, which we will populate with grass
	mField = mSceneMgr->createStaticGeometry("Field");
	mField->setRegionDimensions(Vector3(140, 140, 140));
	//mField->setOrigin(Vector3(70, 70, 70));
	mField->setOrigin(Vector3(-70, -70, -70) + Vector3(2043,65,1715));

	// add grass uniformly throughout the field, with some random variations
	for (int x = -280; x < 280; x += 20)
	{
		for (int z = -280; z < 280; z += 20)
		{
			Vector3 pos(x + Math::RangeRandom(-7, 7), 0, z + Math::RangeRandom(-7, 7));
			Quaternion ori(Degree(Math::RangeRandom(0, 359)), Vector3::UNIT_Y);
			Vector3 scale(1, Math::RangeRandom(0.85, 1.15), 1);

			mField->addEntity(grass, pos, ori, scale);
		}
	}

	mField->build();  // build our static geometry (bake the grass into it)

	ps = mSceneMgr->createParticleSystem("Rain", "Examples/Rain");  // create a rainstorm
	ps->fastForward(5);   // fast-forward the rain so it looks more natural
	mSceneMgr->getRootSceneNode()->createChildSceneNode(Vector3(2000, 1000, 1500))->attachObject(ps);

	ps2 = mSceneMgr->createParticleSystem("Aureola", "Examples/Aureola");
	mSceneMgr->getRootSceneNode()->createChildSceneNode(Vector3(1600,200,2200))->attachObject(ps2);

	ps3 = mSceneMgr->createParticleSystem("Nimbus", "Examples/GreenyNimbus");
	//mSceneMgr->getRootSceneNode()->createChildSceneNode(Vector3(2000,200,1420))->attachObject(ps3);
// 
// 	ParticleEmitter* emitterNodePs3 = ps3->addEmitter("rear");  // add a point emitter
// 	// set the emitter properties
// 	emitterNodePs3->setAngle(Degree(3));
// 	emitterNodePs3->setDirection(Vector3::NEGATIVE_UNIT_Z);

	Ogre::SceneNode* nodePs3 = mSceneMgr->getRootSceneNode()->createChildSceneNode(Ogre::Vector3(2000,200,1420), rot);
	nodePs3->attachObject(ps3);


	Ogre::Entity* char_1 = mSceneMgr->createEntity("char_1", "jaiqua.mesh");
	Ogre::SceneNode* entChar_1 = mSceneMgr->getRootSceneNode()->createChildSceneNode(Ogre::Vector3(2000,2,1420), rot);
	entChar_1->attachObject(char_1);

	Ogre::Entity* char_2 = mSceneMgr->createEntity("char_2", "jaiqua.mesh");
	Ogre::SceneNode* entChar_2 = mSceneMgr->getRootSceneNode()->createChildSceneNode(Ogre::Vector3(2020,2,1420), rot);
	entChar_2->attachObject(char_2);

	Ogre::Entity* char_3 = mSceneMgr->createEntity("char_3", "Sinbad.mesh");
	Ogre::SceneNode* entChar_3 = mSceneMgr->getRootSceneNode()->createChildSceneNode(Ogre::Vector3(2040,5,1420), rot);
	entChar_3->attachObject(char_3);

	Ogre::Entity* char_4 = mSceneMgr->createEntity("char_4", "Sinbad.mesh");
	Ogre::SceneNode* entChar_4 = mSceneMgr->getRootSceneNode()->createChildSceneNode(Ogre::Vector3(2060,5,1420), rot);
	entChar_4->attachObject(char_4);

}

void DPS::configureTerrainDefaults(Ogre::Light* light)
{
	// Configure global
	mTerrainGlobals->setMaxPixelError(8);
	// testing composite map
	mTerrainGlobals->setCompositeMapDistance(3000);

	// Important to set these so that the terrain knows what to use for derived (non-realtime) data
	mTerrainGlobals->setLightMapDirection(light->getDerivedDirection());
	mTerrainGlobals->setCompositeMapAmbient(mSceneMgr->getAmbientLight());
	mTerrainGlobals->setCompositeMapDiffuse(light->getDiffuseColour());

	// Configure default import settings for if we use imported image
	Ogre::Terrain::ImportData& defaultimp = mTerrainGroup->getDefaultImportSettings();
	defaultimp.terrainSize = 513;
	defaultimp.worldSize = 12000.0f;
	defaultimp.inputScale = 600;
	defaultimp.minBatchSize = 33;
	defaultimp.maxBatchSize = 65;
	// textures
	defaultimp.layerList.resize(3);
	defaultimp.layerList[0].worldSize = 100;
	defaultimp.layerList[0].textureNames.push_back("dirt_grayrocky_diffusespecular.dds");
	defaultimp.layerList[0].textureNames.push_back("dirt_grayrocky_normalheight.dds");
	defaultimp.layerList[1].worldSize = 30;
	defaultimp.layerList[1].textureNames.push_back("grass_green-01_diffusespecular.dds");
	defaultimp.layerList[1].textureNames.push_back("grass_green-01_normalheight.dds");
	defaultimp.layerList[2].worldSize = 200;
	defaultimp.layerList[2].textureNames.push_back("growth_weirdfungus-03_diffusespecular.dds");
	defaultimp.layerList[2].textureNames.push_back("growth_weirdfungus-03_normalheight.dds");

	initSoftBody(dpsSoftbodyHelper->createCloth());
}


bool DPS::frameRenderingQueued(const Ogre::FrameEvent& evt)
{
	//Update Bullet world
	Globals::phyWorld->stepSimulation(evt.timeSinceLastFrame, 10); 
	Globals::phyWorld->debugDrawWorld();

	//Shows debug if F3 key down.
	Globals::dbgdraw->setDebugMode(mKeyboard->isKeyDown(OIS::KC_F3));
	Globals::dbgdraw->step();

	Globals::app->updateSoftBody(dpsSoftbodyHelper->m_cloth);
	//Globals::app->updateSoftBody(dpsSoftbodyHelper->m_SoftBody);
	//Globals::app->updateSoftBody(dpsSoftbodyHelper->m_deformableModel);
	//Globals::app->updateSoftBody(dpsSoftbodyHelper->m_bunny);


	if (mTerrainGroup->isDerivedDataUpdateInProgress())
	{
		mTrayMgr->moveWidgetToTray(mInfoLabel, OgreBites::TL_TOP, 0);
		mInfoLabel->show();
		if (mTerrainsImported)
		{
			mInfoLabel->setCaption("Building terrain, please wait...");
		}
		else
		{
			mInfoLabel->setCaption("Updating textures, patience...");
		}
	}
	else
	{
		mTrayMgr->removeWidgetFromTray(mInfoLabel);
		mInfoLabel->hide();
		if (mTerrainsImported)
		{
			mTerrainGroup->saveAllTerrains(true);
			mTerrainsImported = false;
		}
	}

	waveGrass(evt.timeSinceLastFrame); 

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

	//m_ManualObject->begin("CharacterMaterials/LiquidBody", Ogre::RenderOperation::OT_TRIANGLE_LIST);
	m_ManualObject->begin("ClothMaterial", Ogre::RenderOperation::OT_TRIANGLE_LIST);

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

void DPS::createFrameListener(void)
{
	BaseApplication::createFrameListener();

	mInfoLabel = mTrayMgr->createLabel(OgreBites::TL_TOP, "TInfo", "", 350);
}

void DPS::destroyScene(void)
{

}

void DPS::defineTerrain(long x, long y)
{
	Ogre::String filename = mTerrainGroup->generateFilename(x, y);
	if (Ogre::ResourceGroupManager::getSingleton().resourceExists(mTerrainGroup->getResourceGroup(), filename))
	{
		mTerrainGroup->defineTerrain(x, y);
	}
	else
	{
		Ogre::Image img;
		getTerrainImage(x % 2 != 0, y % 2 != 0, img);
		mTerrainGroup->defineTerrain(x, y, &img);
		mTerrainsImported = true;
	}
}

void DPS::initBlendMaps(Ogre::Terrain* terrain)
{
	Ogre::TerrainLayerBlendMap* blendMap0 = terrain->getLayerBlendMap(1);
	Ogre::TerrainLayerBlendMap* blendMap1 = terrain->getLayerBlendMap(2);
	Ogre::Real minHeight0 = 70;
	Ogre::Real fadeDist0 = 40;
	Ogre::Real minHeight1 = 70;
	Ogre::Real fadeDist1 = 15;
	float* pBlend0 = blendMap0->getBlendPointer();
	float* pBlend1 = blendMap1->getBlendPointer();
	for (Ogre::uint16 y = 0; y < terrain->getLayerBlendMapSize(); ++y)
	{
		for (Ogre::uint16 x = 0; x < terrain->getLayerBlendMapSize(); ++x)
		{
			Ogre::Real tx, ty;

			blendMap0->convertImageToTerrainSpace(x, y, &tx, &ty);
			Ogre::Real height = terrain->getHeightAtTerrainPosition(tx, ty);
			Ogre::Real val = (height - minHeight0) / fadeDist0;
			val = Ogre::Math::Clamp(val, (Ogre::Real)0, (Ogre::Real)1);
			*pBlend0++ = val;

			val = (height - minHeight1) / fadeDist1;
			val = Ogre::Math::Clamp(val, (Ogre::Real)0, (Ogre::Real)1);
			*pBlend1++ = val;
		}
	}
	blendMap0->dirty();
	blendMap1->dirty();
	blendMap0->update();
	blendMap1->update();
}

void DPS::getTerrainImage(bool flipX, bool flipY, Ogre::Image& img)
{
	img.load("terrain.png", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
	if (flipX)
		img.flipAroundY();
	if (flipY)
		img.flipAroundX();
}


void DPS::waveGrass(Real timeElapsed)
{
	static Real xinc = Math::PI * 0.3;
	static Real zinc = Math::PI * 0.44;
	static Real xpos = Math::RangeRandom(-Math::PI, Math::PI);
	static Real zpos = Math::RangeRandom(-Math::PI, Math::PI);
	static Vector4 offset(0, 0, 0, 0);

	xpos += xinc * timeElapsed;
	zpos += zinc * timeElapsed;

	// update vertex program parameters by binding a value to each renderable
	StaticGeometry::RegionIterator regs =  mField->getRegionIterator();
	while (regs.hasMoreElements())
	{
		StaticGeometry::Region* reg = regs.getNext();

		// a little randomness
		xpos += reg->getCentre().x * 0.001;
		zpos += reg->getCentre().z * 0.001;
		offset.x = Math::Sin(xpos) * 4;
		offset.z = Math::Sin(zpos) * 4;

		StaticGeometry::Region::LODIterator lods = reg->getLODIterator();
		while (lods.hasMoreElements())
		{
			StaticGeometry::LODBucket::MaterialIterator mats = lods.getNext()->getMaterialIterator();
			while (mats.hasMoreElements())
			{
				StaticGeometry::MaterialBucket::GeometryIterator geoms = mats.getNext()->getGeometryIterator();
				while (geoms.hasMoreElements()) geoms.getNext()->setCustomParameter(999, offset);
			}
		}
	}
}

void DPS::createGrassMesh()
{
	MeshPtr mesh = MeshManager::getSingleton().createManual("grass", ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

	// create a submesh with the grass material
	SubMesh* sm = mesh->createSubMesh();
	sm->setMaterialName("Examples/GrassBlades");
	sm->useSharedVertices = false;
	sm->vertexData = OGRE_NEW VertexData();
	sm->vertexData->vertexStart = 0;
	sm->vertexData->vertexCount = 12;
	sm->indexData->indexCount = 18;

#if defined(INCLUDE_RTSHADER_SYSTEM)
	MaterialPtr grassMat = MaterialManager::getSingleton().getByName("Examples/GrassBlades");
	grassMat->getTechnique(0)->setSchemeName(Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME);
#endif

	// specify a vertex format declaration for our mesh: 3 floats for position, 3 floats for normal, 2 floats for UV
	VertexDeclaration* decl = sm->vertexData->vertexDeclaration;
	decl->addElement(0, 0, VET_FLOAT3, VES_POSITION);
	decl->addElement(0, sizeof(float) * 3, VET_FLOAT3, VES_NORMAL);
	decl->addElement(0, sizeof(float) * 6, VET_FLOAT2, VES_TEXTURE_COORDINATES, 0);

	// create a vertex buffer
	HardwareVertexBufferSharedPtr vb = HardwareBufferManager::getSingleton().createVertexBuffer
		(decl->getVertexSize(0), sm->vertexData->vertexCount, HardwareBuffer::HBU_STATIC_WRITE_ONLY);

	GrassVertex* verts = (GrassVertex*)vb->lock(HardwareBuffer::HBL_DISCARD);  // start filling in vertex data

	for (unsigned int i = 0; i < 3; i++)  // each grass mesh consists of 3 planes
	{
		// planes intersect along the Y axis with 60 degrees between them
		Real x = Math::Cos(Degree(i * 60)) * GRASS_WIDTH / 2;
		Real z = Math::Sin(Degree(i * 60)) * GRASS_WIDTH / 2;

		for (unsigned int j = 0; j < 4; j++)  // each plane has 4 vertices
		{
			GrassVertex& vert = verts[i * 4 + j];

			vert.x = j < 2 ? -x : x;
			vert.y = j % 2 ? 0 : GRASS_HEIGHT;
			vert.z = j < 2 ? -z : z;

			// all normals point straight up
			vert.nx = 0;
			vert.ny = 1;
			vert.nz = 0;

			vert.u = j < 2 ? 0 : 1;
			vert.v = j % 2;
		}
	}

	vb->unlock();  // commit vertex changes

	sm->vertexData->vertexBufferBinding->setBinding(0, vb);  // bind vertex buffer to our submesh

	// create an index buffer
	sm->indexData->indexBuffer = HardwareBufferManager::getSingleton().createIndexBuffer
		(HardwareIndexBuffer::IT_16BIT, sm->indexData->indexCount, HardwareBuffer::HBU_STATIC_WRITE_ONLY);

	// start filling in index data
	Ogre::uint16* indices = (Ogre::uint16*)sm->indexData->indexBuffer->lock(HardwareBuffer::HBL_DISCARD);

	for (unsigned int i = 0; i < 3; i++)  // each grass mesh consists of 3 planes
	{
		unsigned int off = i * 4;  // each plane consists of 2 triangles

		*indices++ = 0 + off;
		*indices++ = 3 + off;
		*indices++ = 1 + off;

		*indices++ = 0 + off;
		*indices++ = 2 + off;
		*indices++ = 3 + off;
	}

	sm->indexData->indexBuffer->unlock();  // commit index changes
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