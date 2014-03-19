#pragma once
#include "MyGUI.h"
#include "MyGUI_OgrePlatform.h"
#include "RenderBox.h"
#include "RenderBoxScene.h"

class GUI
{
public:
	GUI(Ogre::Viewport* vp, Ogre::SceneManager* mSceneMgr, Ogre::RenderWindow* mWindow);
	~GUI(void);

	//Ogre
	Ogre::SceneManager* mSceneMgr;
	Ogre::RenderWindow* mWindow;
	Ogre::Viewport* vp;

	//menu item
	bool Command_Play;
	bool Command_Pause;
	int Command_Slow_Motion;
	bool Command_Clear_Screen;
	bool Command_Cloth_Demo_1;
	bool Command_Cloth_Demo_2;
	bool Command_Cloth_Demo_3;
	bool Command_Softbody_Demo_1;
	bool Command_Softbody_Demo_2;
	bool Command_Softbody_Demo_3;
	bool Command_Deformable_Demo_1;
	bool Command_Deformable_Demo_2;
	bool Command_Deformable_Demo_3;
	bool Command_Deformable_Demo_4;
	bool Command_ResetCamera;
	size_t Theme_Choice;
	bool Command_SwitchTheme;
	bool Command_ScreenShot;
	bool Command_Enable_Mini_Camera;
	bool Command_Disable_Mini_Camera;
	bool Command_Disable_FPS;
	bool Command_Enable_FPS;
	bool Command_Solid;
	bool Command_Wireframe;
	bool Command_Points;
	bool Command_Quit;

	MyGUI::Gui* mGuiSystem;
	MyGUI::OgrePlatform* mGUIPlatform;
	MyGUI::VectorWidgetPtr menuPtr;
	MyGUI::Window* miniCameraWindow;
/*	MyGUI::Window* FPSWindow;*/

	void createGUI(void);
	void destroyGUI(void);
	void menuListener(void);
	void selectedMenuItem(MyGUI::Widget* sender);
	void createMiniCamera(Ogre::Camera* miniCam);
/*	void createFPSWindow(void);*/

};
