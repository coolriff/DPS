#include "GUI.h"


GUI::GUI(Ogre::Viewport* vp, Ogre::SceneManager* mSceneMgr, Ogre::RenderWindow* mWindow)
{
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
	MyGUI::ButtonPtr button= mGuiSystem->createWidget<MyGUI::Button>("Button",10,10,300,26, MyGUI::Align::Default,"Main");
	button->setCaption("Hao");

	//menuPtr = MyGUI::LayoutManager::getInstance().loadLayout("MyPhysicsLab_Menu.layout");
}

void GUI::destroyGUI(void)
{

}


