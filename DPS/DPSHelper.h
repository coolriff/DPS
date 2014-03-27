#ifndef __DPSHELPER_h_
#define __DPSHELPER_h_

#include "BtOgrePG.h"
#include "BtOgreGP.h"
#include "BtOgreExtras.h"

class DPSHelper
{
	public:
		DPSHelper(btDynamicsWorld* phyWorld, Ogre::Camera* mCamera, Ogre::SceneManager* mSceneMgr);
		~DPSHelper(void);

		void setColor(Ogre::Entity* ent ,Ogre::Vector3 v);
		void createPointLight(std::string LightName, Ogre::Vector3 position ,Ogre::Vector3 direction);
		void createDirectionLight(std::string LightName, Ogre::Vector3 position ,Ogre::Vector3 direction);
		void createSpotLight(std::string LightName, Ogre::Vector3 position ,Ogre::Vector3 direction);
		void createSoft(void);
		void createWorld(void);
		btRigidBody* throwSphere(void);
		void throwCube(void);
		void createOgreHead(void);
		void createLeapMotionSphere_0(std::string fingerName, Ogre::Vector3 position);
		void createLeapMotionSphere_1(std::string fingerName, Ogre::Vector3 position);
		void createCube(Ogre::Vector3 position, btScalar mass);
		void createSphere(Ogre::Vector3 position, btScalar mass);

		Ogre::SceneNode* sphereNode_0;
		btRigidBody* sphereBody_0;

		Ogre::SceneNode* sphereNode_1;
		btRigidBody* sphereBody_1;

		btDynamicsWorld* phyWorld;
		Ogre::Camera* mCamera;
		Ogre::SceneManager* mSceneMgr;
};

#endif