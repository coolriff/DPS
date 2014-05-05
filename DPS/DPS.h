#ifndef __DPS_h_
#define __DPS_h_

#define NUM_ANIMS 13           // number of animations the character has
#define CHAR_HEIGHT 5          // height of character's center of mass above ground
#define CAM_HEIGHT 2           // height of camera above character's center of mass
#define RUN_SPEED 17           // character running speed in units per second
#define TURN_SPEED 500.0f      // character turning in degrees per second
#define ANIM_FADE_SPEED 7.5f   // animation crossfade speed in % of full weight per second
#define JUMP_ACCEL 30.0f       // character jump acceleration in upward units per squared second
#define GRAVITY 90.0f          // gravity in downward units per squared second

#include <Terrain/OgreTerrain.h>
#include <Terrain/OgreTerrainGroup.h>
#include "BaseApplication.h"
#include "DPSHelper.h"
#include "DPSSoftBodyHelper.h"
#include "BtOgrePG.h"
#include "BtOgreGP.h"
#include "BtOgreExtras.h"
#include "ExampleApplication.h"
#include <btBulletDynamicsCommon.h>
#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>
#include <BulletSoftBody/btDefaultSoftBodySolver.h>
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h>
#include "Leap.h"
#include "LeapListener.h"

#include <OgreHardwarePixelBuffer.h>

class DPS;
class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;

///collisions between two btSoftBody's
class btSoftSoftCollisionAlgorithm;

///collisions between a btSoftBody and a btRigidBody
class btSoftRididCollisionAlgorithm;
class btSoftRigidDynamicsWorld;



namespace Globals
{
	DPS* app;
	btSoftRigidDynamicsWorld* phyWorld;
    BtOgre::DebugDrawer* dbgdraw;
}

class DPS : public BaseApplication
{
	public:

		DPS(void);
		~DPS(void);

		enum AnimID
		{
			ANIM_IDLE_BASE,
			ANIM_IDLE_TOP,
			ANIM_RUN_BASE,
			ANIM_RUN_TOP,
			ANIM_HANDS_CLOSED,
			ANIM_HANDS_RELAXED,
			ANIM_DRAW_SWORDS,
			ANIM_SLICE_VERTICAL,
			ANIM_SLICE_HORIZONTAL,
			ANIM_DANCE,
			ANIM_JUMP_START,
			ANIM_JUMP_LOOP,
			ANIM_JUMP_END,
			ANIM_NONE
		};

		void setBaseAnimation(AnimID id, bool reset = false)
		{
			if (mBaseAnimID >= 0 && mBaseAnimID < NUM_ANIMS)
			{
				// if we have an old animation, fade it out
				mFadingIn[mBaseAnimID] = false;
				mFadingOut[mBaseAnimID] = true;
			}

			mBaseAnimID = id;

			if (id != ANIM_NONE)
			{
				// if we have a new animation, enable it and fade it in
				mAnims[id]->setEnabled(true);
				mAnims[id]->setWeight(0);
				mFadingOut[id] = false;
				mFadingIn[id] = true;
				if (reset) mAnims[id]->setTimePosition(0);
			}
		}

		void setTopAnimation(AnimID id, bool reset = false)
		{
			if (mTopAnimID >= 0 && mTopAnimID < NUM_ANIMS)
			{
				// if we have an old animation, fade it out
				mFadingIn[mTopAnimID] = false;
				mFadingOut[mTopAnimID] = true;
			}

			mTopAnimID = id;
			if (id != ANIM_NONE)
			{
				// if we have a new animation, enable it and fade it in
				mAnims[id]->setEnabled(true);
				mAnims[id]->setWeight(0);
				mFadingOut[id] = false;
				mFadingIn[id] = true;
				if (reset) mAnims[id]->setTimePosition(0);
			}
		}

		btAlignedObjectArray<btSoftSoftCollisionAlgorithm*> m_SoftSoftCollisionAlgorithms;
		btAlignedObjectArray<btSoftRididCollisionAlgorithm*> m_SoftRigidCollisionAlgorithms;
		btSoftBodyWorldInfo	m_softBodyWorldInfo;

		//keep the collision shapes, for deletion/cleanup
		btAlignedObjectArray<btCollisionShape*> m_collisionShapes;
		btBroadphaseInterface*	m_broadphase;
		btCollisionDispatcher*	m_dispatcher;
		btConstraintSolver*	m_solver;
		btCollisionAlgorithmCreateFunc*	m_boxBoxCF;
		btDefaultCollisionConfiguration* m_collisionConfiguration;

		void initPhysics(void);
		void exitPhysics(void);
		void resetCamera(void);
		void resetCamera(Ogre::Vector3 camPos);
		void deleteOgreEntities(void);
		void deletePhysicsShapes(void);
		void GUIeventHandler(void);
		void setMiniCamPosition(Ogre::Vector3 camPos);
		std::string convertInt(int number);

		void bulletDebugScreen(void);
		void solidScreen(void);
		void demoController(void);
		void clearScreen(void);

		Ogre::FrameEvent evt;
		double dt;

		int maxProxies;
		btRigidBody *hit_body;
		btVector3 hit_rel_pos;
		btVector3 shot_imp;

		Ogre::MovableObject* rayObject;
		Ogre::SceneNode* rayNode;

		bool runClothDome_1;
		bool runClothDome_2;
		bool runClothDome_3;
		bool runClothDome_4;
		bool runClothDome_5;
		bool runClothDome_6;
		bool runClothDome_7;

		bool runDeformDome_1;
		bool runDeformDome_2;
		bool runDeformDome_3;
		bool runDeformDome_4;
		bool runDeformDome_5;
		bool runDeformDome_6;
		bool runDeformDome_7;

		bool runPlaygroud_1;
		bool runPlaygroud_2;
		bool runPlaygroud_3;
		bool runPlaygroud_4;
		bool runPlaygroud_5;

		int playgroundCount;

		SceneNode* camNode;
		Ogre::SceneNode* mSinbadNode;
		Ogre::SceneNode* mCameraPivot;
		Ogre::SceneNode* mCameraGoal;
		Ogre::SceneNode* mCameraNode;
		Ogre::Real mPivotPitch;
		Ogre::Entity* mBodyEnt;
		Ogre::Entity* mSword1;
		Ogre::Entity* mSword2;
		Ogre::RibbonTrail* mSwordTrail;
		Ogre::AnimationState* mAnims[NUM_ANIMS];    // master animation list
		AnimID mBaseAnimID;                   // current base (full- or lower-body) animation
		AnimID mTopAnimID;                    // current top (upper-body) animation
		bool mFadingIn[NUM_ANIMS];            // which animations are fading in
		bool mFadingOut[NUM_ANIMS];           // which animations are fading out
		bool mSwordsDrawn;
		Ogre::Vector3 mKeyDirection;      // player's local intended direction based on WASD keys
		Ogre::Vector3 mGoalDirection;     // actual intended direction in world-space
		Ogre::Real mVerticalVelocity;     // for jumping
		Ogre::Real mTimer;                // general timer to see how long animations have been playing

		ParticleSystem* fire_1;
		ParticleSystem* fire_2;
		ParticleSystem* ps1;
		ParticleSystem* ps2;
		ParticleSystem* ps3;
		int fireBallCount;
		std::string fireBallName;

		Ogre::SceneNode* pNode1;
		Ogre::SceneNode* pNode2;
		Ogre::SceneNode* pNode3;
		Ogre::SceneNode* pNode4;

		Ogre::SceneNode* dragon;

		Ogre::Rectangle2D* rect1;
		Ogre::Rectangle2D* rect2;
		Ogre::Rectangle2D* rect3;
		Ogre::Rectangle2D* rect4;
		Ogre::SceneNode* photoNode1;
		Ogre::SceneNode* photoNode2;
		Ogre::SceneNode* photoNode3;
		Ogre::SceneNode* photoNode4;

		Ogre::Entity* RZR_001;
		Ogre::Entity* razor1;
		Ogre::Entity* razor2;
		Ogre::Entity* razor3;

		Ogre::SceneNode* entRZR_001;
		Ogre::SceneNode* entRazor1;
		Ogre::SceneNode* entRazor2;
		Ogre::SceneNode* entRazor3;

		Ogre::SceneNode* p1;
		Ogre::SceneNode* p2;
		Ogre::SceneNode* p3;



		int timeLine;
		void GameCA(int timeLine, Ogre::Real Time);

		void GameCALoadModel(void);
		virtual bool nextLocation(void);
		virtual void createFrameListener(void);
		void defineTerrain(long x, long y);
		void initBlendMaps(Ogre::Terrain* terrain);
		void configureTerrainDefaults(Ogre::Light* light);
		void getTerrainImage(bool flipX, bool flipY, Ogre::Image& img);
		Ogre::Vector3 ogreLerp (Ogre::Vector3 srcLocation, Ogre::Vector3 destLocation, Ogre::Real Time);
		//void addPhoto(void);

		Ogre::Vector3 camLerpPos;
		Ogre::Vector3 camLerpPos1;
		Ogre::Vector3 camLerpPos2;
		Ogre::Vector3 camLerpPos3;
		Ogre::Vector3 camPos;
		Ogre::Vector3 startPos1;
		Ogre::Vector3 startPos2;
		Ogre::Vector3 startPos3;
		Ogre::Vector3 endPos1;
		Ogre::Vector3 endPos2;
		Ogre::Vector3 endPos3;

		Ogre::Vector3 targetPos;
		Ogre::Real times;
		Ogre::TerrainGlobalOptions* mTerrainGlobals;
		Ogre::TerrainGroup* mTerrainGroup;
		bool mTerrainsImported;
		OgreBites::Label* mInfoLabel;


		Ogre::Real mDistance;                  // The distance the object has left to travel
		Ogre::Vector3 mDirection;              // The direction the object is moving
		Ogre::Vector3 mDestination;            // The destination the object is moving towards

		Ogre::AnimationState *mAnimationState; // The current animation state of the object
		Ogre::AnimationState *mAnimationState1; // The current animation state of the object

		Ogre::Entity *mEntity;                 // The Entity we are animating
		Ogre::SceneNode *mNode;                // The SceneNode that the Entity is attached to
		std::deque<Ogre::Vector3> mWalkList;   // The list of points we are walking to

		Ogre::Real mWalkSpeed;                 // The speed at which the object is moving 

	protected:

		std::shared_ptr<DPSHelper> dpsHelper;
		std::shared_ptr<DPSSoftBodyHelper> dpsSoftbodyHelper;

		virtual void createScene(void);
		virtual bool frameRenderingQueued(const Ogre::FrameEvent& evt);

		// OIS::MouseListener
		virtual bool mouseMoved(const OIS::MouseEvent &arg);
		virtual bool mousePressed(const OIS::MouseEvent &arg, OIS::MouseButtonID id);
		virtual bool mouseReleased(const OIS::MouseEvent &arg, OIS::MouseButtonID id);

		bool keyPressed(const OIS::KeyEvent &arg);
};



#endif