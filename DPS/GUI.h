#pragma once
#include "MyGUI.h"
#include "MyGUI_OgrePlatform.h"


class GUI
{
public:
	GUI(Ogre::Viewport* vp, Ogre::SceneManager* mSceneMgr, Ogre::RenderWindow* mWindow);
	~GUI(void);

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
	bool Command_SwitchScreen;

	MyGUI::Gui* mGuiSystem;
	MyGUI::OgrePlatform* mGUIPlatform;
	MyGUI::VectorWidgetPtr menuPtr;

	void createGUI(void);
	void destroyGUI(void);
	void menuListener(void);
	void selectedMenuItem(MyGUI::Widget* sender);
};

