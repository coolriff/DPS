#include "GUI.h"


GUI::GUI(Ogre::Viewport* vp, Ogre::SceneManager* mSceneMgr, Ogre::RenderWindow* mWindow)
{
	Command_Play = false;
	Command_Pause = false;
	Command_Slow_Motion = 0;
	Command_Clear_Screen = false;
	Command_Cloth_Demo_1 = false;
	Command_Cloth_Demo_2 = false;
	Command_Cloth_Demo_3 = false;
	Command_Softbody_Demo_1 = false;
	Command_Softbody_Demo_2 = false;
	Command_Softbody_Demo_3 = false;
	Command_Deformable_Demo_1 = false;
	Command_Deformable_Demo_2 = false;
	Command_Deformable_Demo_3 = false;
	Command_Deformable_Demo_4 = false;
	Command_ResetCamera = false;
	Theme_Choice = 0;
	Command_SwitchTheme = false;
	Command_SwitchScreen = false;

	mGUIPlatform = new MyGUI::OgrePlatform();
	mGUIPlatform->initialise(mWindow, mSceneMgr);
	mGuiSystem = new MyGUI::Gui();
	mGuiSystem->initialise();
}


GUI::~GUI(void)
{
}

void GUI::createGUI(void)
{
	menuPtr = MyGUI::LayoutManager::getInstance().loadLayout("DPS.layout");
	menuListener();
// 	MyGUI::ButtonPtr button= mGuiSystem->createWidget<MyGUI::Button>("Button",10,10,300,26, MyGUI::Align::Default,"Main");
// 	button->setCaption("Hao");
}

void GUI::destroyGUI(void)
{
	mGuiSystem->shutdown();
	delete mGuiSystem;

	mGUIPlatform->shutdown();
	delete mGUIPlatform;
}

void GUI::menuListener(void)
{
	MyGUI::Widget* widget;

	if(widget = mGuiSystem->findWidget<MyGUI::Widget>("Command_Clear_Screen"))
	{
		widget-> eventMouseButtonClick += MyGUI::newDelegate(this, &GUI::selectedMenuItem);
	}

}

void GUI::selectedMenuItem(MyGUI::Widget* sender)
{
	std::string name = sender->getName();
	if(name == "Command_Clear_Screen")
	{
		Command_Clear_Screen = true;
	}
}


