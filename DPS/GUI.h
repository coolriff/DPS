#pragma once
#include "MyGUI.h"
#include "MyGUI_OgrePlatform.h"

class GUI
{
public:
	GUI(Ogre::Viewport* vp, Ogre::SceneManager* mSceneMgr, Ogre::RenderWindow* mWindow);
	~GUI(void);

	MyGUI::Gui* mGuiSystem;
	MyGUI::OgrePlatform* mGUIPlatform;

	void createGUI(void);
	void destroyGUI(void);
};

