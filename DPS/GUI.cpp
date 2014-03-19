#include "GUI.h"

GUI::GUI(Ogre::Viewport* vp, Ogre::SceneManager* mSceneMgr, Ogre::RenderWindow* mWindow)
{
	this->vp = vp;
	this->mSceneMgr = mSceneMgr;
	this->mWindow = mWindow;

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
	Command_ScreenShot = false;
	Command_Enable_Mini_Camera = false;
	Command_Disable_Mini_Camera = false;
	Command_Disable_FPS = false;
	Command_Enable_FPS = false;
	Command_Solid = false;
	Command_Wireframe = false;
	Command_Points = false;
	Command_Quit = false;

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
	createMiniCamera(mSceneMgr->getCamera("miniCam"));
/*	createFPSWindow();*/
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

	if(widget = mGuiSystem->findWidget<MyGUI::Widget>("Command_Quit"))
	{
		widget-> eventMouseButtonClick += MyGUI::newDelegate(this, &GUI::selectedMenuItem);
	}
	if(widget = mGuiSystem->findWidget<MyGUI::Widget>("Command_Clear_Screen"))
	{
		widget-> eventMouseButtonClick += MyGUI::newDelegate(this, &GUI::selectedMenuItem);
	}
	if(widget = mGuiSystem->findWidget<MyGUI::Widget>("Command_Enable_Mini_Camera"))
	{
		widget-> eventMouseButtonClick += MyGUI::newDelegate(this, &GUI::selectedMenuItem);
	}
	if(widget = mGuiSystem->findWidget<MyGUI::Widget>("Command_Disable_Mini_Camera"))
	{
		widget-> eventMouseButtonClick += MyGUI::newDelegate(this, &GUI::selectedMenuItem);
	}
	if(widget = mGuiSystem->findWidget<MyGUI::Widget>("Command_Enable_FPS"))
	{
		widget-> eventMouseButtonClick += MyGUI::newDelegate(this, &GUI::selectedMenuItem);
	}
	if(widget = mGuiSystem->findWidget<MyGUI::Widget>("Command_Disable_FPS"))
	{
		widget-> eventMouseButtonClick += MyGUI::newDelegate(this, &GUI::selectedMenuItem);
	}
	if(widget = mGuiSystem->findWidget<MyGUI::Widget>("Command_Solid"))
	{
		widget-> eventMouseButtonClick += MyGUI::newDelegate(this, &GUI::selectedMenuItem);
	}
	if(widget = mGuiSystem->findWidget<MyGUI::Widget>("Command_Wireframe"))
	{
		widget-> eventMouseButtonClick += MyGUI::newDelegate(this, &GUI::selectedMenuItem);
	}
	if(widget = mGuiSystem->findWidget<MyGUI::Widget>("Command_Points"))
	{
		widget-> eventMouseButtonClick += MyGUI::newDelegate(this, &GUI::selectedMenuItem);
	}
	if(widget = mGuiSystem->findWidget<MyGUI::Widget>("Command_ScreenShot"))
	{
		widget-> eventMouseButtonClick += MyGUI::newDelegate(this, &GUI::selectedMenuItem);
	}
	
}

void GUI::selectedMenuItem(MyGUI::Widget* sender)
{
	std::string name = sender->getName();
	if (name == "Command_Quit")
	{
		Command_Quit = true;
	} 
	if(name == "Command_Clear_Screen")
	{
		Command_Clear_Screen = true;
	}
	if(name == "Command_Enable_Mini_Camera")
	{
		Command_Enable_Mini_Camera = true;
		Command_Disable_Mini_Camera = false;
		mGuiSystem->findWidget<MyGUI::Widget>("Command_Enable_Mini_Camera")->setEnabled(false);
		mGuiSystem->findWidget<MyGUI::Widget>("Command_Disable_Mini_Camera")->setEnabled(true);
		miniCameraWindow->setVisible(true);
	}
	if(name == "Command_Disable_Mini_Camera")
	{
		Command_Disable_Mini_Camera = true;
		Command_Enable_Mini_Camera = false;
		mGuiSystem->findWidget<MyGUI::Widget>("Command_Enable_Mini_Camera")->setEnabled(true);
		mGuiSystem->findWidget<MyGUI::Widget>("Command_Disable_Mini_Camera")->setEnabled(false);
		miniCameraWindow->setVisible(false);
	}
	if(name == "Command_Enable_FPS")
	{
		Command_Enable_FPS = true;
		mGuiSystem->findWidget<MyGUI::Widget>("Command_Enable_FPS")->setEnabled(false);
		mGuiSystem->findWidget<MyGUI::Widget>("Command_Disable_FPS")->setEnabled(true);
/*		FPSWindow->setVisible(true);*/
	}
	if(name == "Command_Disable_FPS")
	{
		Command_Disable_FPS = true;
		mGuiSystem->findWidget<MyGUI::Widget>("Command_Enable_FPS")->setEnabled(true);
		mGuiSystem->findWidget<MyGUI::Widget>("Command_Disable_FPS")->setEnabled(false);
/*		FPSWindow->setVisible(false);*/
	}
	if(name == "Command_Solid")
	{
		vp->getCamera()->setPolygonMode(Ogre::PM_SOLID);

		mGuiSystem->findWidget<MyGUI::Widget>("Command_Solid")->setEnabled(false);
		mGuiSystem->findWidget<MyGUI::Widget>("Command_Wireframe")->setEnabled(true);
		mGuiSystem->findWidget<MyGUI::Widget>("Command_Points")->setEnabled(true);
	}
	if(name == "Command_Wireframe")
	{
		vp->getCamera()->setPolygonMode(Ogre::PM_WIREFRAME);

		mGuiSystem->findWidget<MyGUI::Widget>("Command_Solid")->setEnabled(true);
		mGuiSystem->findWidget<MyGUI::Widget>("Command_Wireframe")->setEnabled(false);
		mGuiSystem->findWidget<MyGUI::Widget>("Command_Points")->setEnabled(true);
	}
	if(name == "Command_Points")
	{
		vp->getCamera()->setPolygonMode(Ogre::PM_POINTS);

		mGuiSystem->findWidget<MyGUI::Widget>("Command_Solid")->setEnabled(true);
		mGuiSystem->findWidget<MyGUI::Widget>("Command_Wireframe")->setEnabled(true);
		mGuiSystem->findWidget<MyGUI::Widget>("Command_Points")->setEnabled(false);
	}
	if(name == "Command_ScreenShot")
	{
		Command_ScreenShot = true;
	}
}

void GUI::createMiniCamera(Ogre::Camera* miniCam)
{
	const MyGUI::IntSize& size = MyGUI::RenderManager::getInstance().getViewSize();

	miniCameraWindow = MyGUI::Gui::getInstance().createWidget<MyGUI::Window>("WindowCS", MyGUI::IntCoord(size.width - 360, size.height - 270, 360, 270), MyGUI::Align::Right|MyGUI::Align::Bottom, "Overlapped");
	miniCameraWindow->setCaption("Camera View");
	miniCameraWindow->setMinSize(MyGUI::IntSize(100, 100));
	miniCameraWindow->setVisible(false);
	MyGUI::Canvas* canvas = miniCameraWindow->createWidget<MyGUI::Canvas>("Canvas", MyGUI::IntCoord(0, 0, miniCameraWindow->getClientCoord().width, miniCameraWindow->getClientCoord().height), MyGUI::Align::Stretch);
	canvas->setPointer("hand");

	wraps::RenderBox* gRenderBox = new wraps::RenderBox();
	gRenderBox->setCanvas(canvas);
	gRenderBox->setViewport(miniCam);
	gRenderBox->setBackgroundColour(MyGUI::Colour::Black);
}

// void GUI::createFPSWindow(void)
// {
// 	const MyGUI::IntSize& size = MyGUI::RenderManager::getInstance().getViewSize();
// 
// 	FPSWindow = mGuiSystem->findWidget<MyGUI::Window>("FPSWindow");
// 	FPSWindow->setPosition(10, size.height-FPSWindow->getHeight()-10);
// 	FPSWindow->setVisible(true);
// }


