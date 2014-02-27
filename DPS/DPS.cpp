#include "DPS.h"
#include <vector>
#include <iostream>
#include <fstream> 
#include <string> 
//#include <memory>
//using namespace std;


DPS::DPS(void)
{
	//Bullet initialisation.
	collisionConfig = new btSoftBodyRigidBodyCollisionConfiguration();
	dispatcher = new btCollisionDispatcher(collisionConfig);
	broadphase = new btDbvtBroadphase();
	solver = new btSequentialImpulseConstraintSolver();
	softbodySolver = new btDefaultSoftBodySolver();
	Globals::phyWorld = new btSoftRigidDynamicsWorld(dispatcher,broadphase,solver,collisionConfig,softbodySolver);
	Globals::phyWorld->setGravity(btVector3(0,-10,0));
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
	mCamera->setPosition(Vector3(0,20,20));
	mCamera->lookAt(Vector3(0,20,0));
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

	mSceneMgr->setSkyBox(true, "Examples/SpaceSkyBox");

	//initSoftBody(createSoftBody(btVector3(0,20,0)));
	//initSoftBody(createCloth());
	initSoftBody(createDeformableModel());
}


bool DPS::frameRenderingQueued(const Ogre::FrameEvent& evt)
{
	//Update Bullet world
	Globals::phyWorld->stepSimulation(evt.timeSinceLastFrame, 10); 
	Globals::phyWorld->debugDrawWorld();

	//Shows debug if F3 key down.
	Globals::dbgdraw->setDebugMode(mKeyboard->isKeyDown(OIS::KC_F3));
	Globals::dbgdraw->step();

	//Globals::app->updateSoftBody(m_cloth);
	//Globals::app->updateSoftBody(m_SoftBody);
	Globals::app->updateSoftBody(m_deformableModel);

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

btSoftBody* DPS::createDeformableModel(void)
{
	std::vector<float> triangles;
	std::vector<int> indicies;
	Objloader* obj = new Objloader;
	obj->LoadModel("monkey",&triangles,&indicies);
	//load("monkey.obj",&triangles,&indicies);

	m_deformableModel = btSoftBodyHelpers::CreateFromTriMesh(Globals::phyWorld->getWorldInfo(),&(triangles[0]),&(indicies[0]),indicies.size()/3,true);
	m_deformableModel->setTotalMass(20.0,true);
	//m_deformableModel->generateClusters(1000);
	m_deformableModel->m_cfg.kSRHR_CL=1.0;	
	//m_deformableModel->m_cfg.collisions =	btSoftBody::fCollision::CL_RS;
	m_deformableModel->m_cfg.viterations=500;
	m_deformableModel->m_cfg.piterations=500;
	m_deformableModel->m_cfg.citerations=500;
	m_deformableModel->m_cfg.diterations=500;
	m_deformableModel->m_cfg.kPR=500;
	m_deformableModel->translate(btVector3(0,5,0));
	//softMonkey->setMass(0,0);
	Globals::phyWorld->addSoftBody(m_deformableModel);

	return m_deformableModel;
}


btSoftBody* DPS::createSoftBody(const btVector3& startPos)
{
	m_SoftBody = btSoftBodyHelpers::CreateEllipsoid(Globals::phyWorld->getWorldInfo(), startPos, btVector3(2,2,2), 200);
	//m_SoftBody->m_cfg.viterations=50;
	//m_SoftBody->m_cfg.piterations=50;
	//set the liquid body properties
	m_SoftBody->m_cfg.kPR = 3500.f;
	m_SoftBody->m_cfg.kDP = 0.001f;
	m_SoftBody->m_cfg.kDF = 0.1f;
	m_SoftBody->m_cfg.kKHR = 1.f; //we hardcode this parameter, since any value below 1.0 means the soft body does less than full correction on penetration
	m_SoftBody->m_cfg.kCHR  = 1.f;
	m_SoftBody->setTotalMass(50.0);
	m_SoftBody->setMass(0,0);
	//m_LiquidBody->generateClusters(100);
	m_SoftBody->m_materials[0]->m_kLST = 0.1f;
	Globals::phyWorld->addSoftBody(m_SoftBody);

	return m_SoftBody;
}


btSoftBody* DPS::createCloth(void)
{
	float s=4;
	float h=20;
	m_cloth = btSoftBodyHelpers::CreatePatch(Globals::phyWorld->getWorldInfo(),btVector3(-s,h,-s),btVector3(s,h,-s),btVector3(-s,h,s),btVector3(s,h,s),50,50,4+8,true);
	m_cloth->m_cfg.viterations=50;
	m_cloth->m_cfg.piterations=50;
	m_cloth->setTotalMass(3.0);
	//m_cloth->setMass(100,100);
	Globals::phyWorld->addSoftBody(m_cloth);

	return m_cloth;
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

	//http://www.ogre3d.org/tikiwiki/ManualObject
	m_ManualObject->begin("ClothMaterial", Ogre::RenderOperation::OT_TRIANGLE_LIST);
	for (int i = 0; i < faces.size(); ++i)
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


	//Ogre::Vector3 pos = Ogre::Vector3(0,50,0);
	//Ogre::Quaternion rot = Ogre::Quaternion::IDENTITY;
	//Ogre::SceneNode* mLiquidBodyNode = mSceneMgr->getRootSceneNode()->createChildSceneNode(pos,rot);

	Ogre::SceneNode* mLiquidBodyNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	mLiquidBodyNode->attachObject(m_ManualObject);
}


void DPS::updateSoftBody(btSoftBody* body)
{
	//grab the calculated mesh data from the physics body
	btSoftBody::tNodeArray& nodes(body->m_nodes);
	btSoftBody::tFaceArray& faces(body->m_faces);

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

void DPS::load(std::string filenames,std::vector<float>* triangles,std::vector<int>* indicies)
{
	std::vector<std::string*> coord;
	string filename = "C:\\DPS\\DPS\\DPS\\" + filenames;
	std::ifstream in(filename.c_str());

	if(!in.is_open())
	{
		std::cout << "Not oepened" << std::endl;
	}
	std::string path=filename.substr(0,(filename.find_last_of('/')!=std::string::npos ? filename.find_last_of('/')+1 : 0));
	char buf[256] = {0};
	int curmat=0;
	bool coll=false;
	while(!in.eof())
	{
		in.getline(buf,255);
		coord.push_back(new std::string(buf));
	}
	for(int i=0;i<coord.size();i++)
	{
		if((*coord[i])[0]=='#')
			continue;
		else if((*coord[i])[0]=='v' && (*coord[i])[1]==' ')
		{
			float tmpx,tmpy,tmpz;
			sscanf(coord[i]->c_str(),"v %f %f %f",&tmpx,&tmpy,&tmpz);
			//vertex.push_back(new vector3d(tmpx,tmpy,tmpz));
			if(triangles)
			{
				triangles->push_back(tmpx);
				triangles->push_back(tmpy);
				triangles->push_back(tmpz);
			}
		}
		else if((*coord[i])[0]=='v' && (*coord[i])[1]=='n')
		{
			float tmpx,tmpy,tmpz;
			sscanf(coord[i]->c_str(),"vn %f %f %f",&tmpx,&tmpy,&tmpz);
			//normals.push_back(new vector3d(tmpx,tmpy,tmpz));	
		}
		else if((*coord[i])[0]=='f')
		{
			int a,b,c,d,e;
			if(coll)
			{
				sscanf(coord[i]->c_str(),"f %d//%d %d//%d %d//%d %d//%d",&a,&b,&c,&b,&d,&b,&e,&b);
			}
			else
			{
				if(count(coord[i]->begin(),coord[i]->end(),' ')==4)
				{
					if(coord[i]->find("//")!=std::string::npos)
					{
						sscanf(coord[i]->c_str(),"f %d//%d %d//%d %d//%d %d//%d",&a,&b,&c,&b,&d,&b,&e,&b);
					}
					else if(coord[i]->find("/")!=std::string::npos)
					{
						int t[4];
						sscanf(coord[i]->c_str(),"f %d/%d/%d %d/%d/%d %d/%d/%d %d/%d/%d",&a,&t[0],&b,&c,&t[1],&b,&d,&t[2],&b,&e,&t[3],&b);
					}
					else
					{
						sscanf(coord[i]->c_str(),"f %d %d %d %d",&a,&b,&c,&d);				
					}
				}
				else
				{
					if(coord[i]->find("//")!=std::string::npos)
					{
						sscanf(coord[i]->c_str(),"f %d//%d %d//%d %d//%d",&a,&b,&c,&b,&d,&b);
						if(indicies)
						{
							indicies->push_back(a-1);
							indicies->push_back(c-1);
							indicies->push_back(d-1);
						}
					}
					else if(coord[i]->find("/")!=std::string::npos)
					{
						int t[3];
						sscanf(coord[i]->c_str(),"f %d/%d/%d %d/%d/%d %d/%d/%d",&a,&t[0],&b,&c,&t[1],&b,&d,&t[2],&b);
					}
					else
					{
						sscanf(coord[i]->c_str(),"f %d %d %d",&a,&b,&c);				
					}
				}
			}
		}
	}
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