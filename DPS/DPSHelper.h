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
		void createGround(void);
		void throwSphere(void);
		void throwCube(void);
		void createOgreHead(void);

		btDynamicsWorld* phyWorld;
		Ogre::Camera* mCamera;
		Ogre::SceneManager* mSceneMgr;
};

#endif