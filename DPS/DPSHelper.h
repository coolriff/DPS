#ifndef __DPSHELPER_h_
#define __DPSHELPER_h_

#include "BtOgrePG.h"
#include "BtOgreGP.h"
#include "BtOgreExtras.h"

class DPSHelper
{
	public:
		DPSHelper(btDynamicsWorld* phyWorld,Ogre::Camera* mCamera);
		~DPSHelper(void);

		void setColor(Ogre::Entity* ent ,Ogre::Vector3 v);
		void createPointLight(std::string LightName, Ogre::Vector3 position ,Ogre::Vector3 direction, Ogre::SceneManager* mSceneMgr);
		void createDirectionLight(std::string LightName, Ogre::Vector3 position ,Ogre::Vector3 direction, Ogre::SceneManager* mSceneMgr);
		void createSpotLight(std::string LightName, Ogre::Vector3 position ,Ogre::Vector3 direction, Ogre::SceneManager* mSceneMgr);
		void createGround(Ogre::SceneManager* mSceneMgr);
		void throwSphere(Ogre::SceneManager* mSceneMgr,Ogre::Camera* mCamera);

		btDynamicsWorld* phyWorld;
		Ogre::Camera* mCamera;
};

#endif