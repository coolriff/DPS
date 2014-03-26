#include "GUI.h"

GUI::GUI(Ogre::Viewport* vp, Ogre::SceneManager* mSceneMgr, Ogre::RenderWindow* mWindow)
{
	this->vp = vp;
	this->mSceneMgr = mSceneMgr;
	this->mWindow = mWindow;

	Command_Enable_Slow = false;
	Command_Disable_Slow = false;
	Command_Slow_Motion = false;
	Command_Clear_Screen = false;
	Command_Cloth_Demo_1 = false;
	Command_Cloth_Demo_2 = false;
	Command_Cloth_Demo_3 = false;
	Command_Cloth_Demo_4 = false;
	Command_Cloth_Demo_5 = false;
	Command_Softbody_Demo_1 = false;
	Command_Softbody_Demo_2 = false;
	Command_Softbody_Demo_3 = false;
	Command_Softbody_Demo_4 = false;
	Command_Softbody_Demo_5 = false;
	Command_Deformable_Demo_1 = false;
	Command_Deformable_Demo_2 = false;
	Command_Deformable_Demo_3 = false;
	Command_Deformable_Demo_4 = false;
	Command_Deformable_Demo_5 = false;
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
	Command_Bullet_Debug_Mode  = false;
	Command_Quit = false;
	Command_Reset_Camera = false;
	Simulation_Default = true;
	Simulation_Stop = false;
	demoDt = 0;
	lastNum = 0;

	mGUIPlatform = new MyGUI::OgrePlatform();
	mGUIPlatform->initialise(mWindow, mSceneMgr);
	mGuiSystem = new MyGUI::Gui();
	mGuiSystem->initialise();
}


GUI::~GUI(void)
{
}

void GUI::createGUI(size_t _index)
{
	if (_index == 0)
	{
		MyGUI::ResourceManager::getInstance().load("MyGUI_BlueWhiteTheme.xml");
	}
	else if (_index == 1)
	{
		MyGUI::ResourceManager::getInstance().load("MyGUI_BlackBlueTheme.xml");
	}
	else if (_index == 2)
	{
		MyGUI::ResourceManager::getInstance().load("MyGUI_BlackOrangeTheme.xml");
	}

	menuPtr = MyGUI::LayoutManager::getInstance().loadLayout("DPS.layout");
	menuListener();
	createMiniCamera(mSceneMgr->getCamera("miniCam"));
	createSimulationSpeedWindow();
/*	createFPSWindow();*/

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
	if(widget = mGuiSystem->findWidget<MyGUI::Widget>("Command_Slow_Motion"))
	{
		widget-> eventMouseButtonClick += MyGUI::newDelegate(this, &GUI::selectedMenuItem);
	}
	if(widget = mGuiSystem->findWidget<MyGUI::Widget>("Command_Enable_Slow"))
	{
		widget-> eventMouseButtonClick += MyGUI::newDelegate(this, &GUI::selectedMenuItem);
	}
	if(widget = mGuiSystem->findWidget<MyGUI::Widget>("Command_Disable_Slow"))
	{
		widget-> eventMouseButtonClick += MyGUI::newDelegate(this, &GUI::selectedMenuItem);
	}
	if(widget = mGuiSystem->findWidget<MyGUI::Widget>("Command_Bullet_Debug_Mode"))
	{
		widget-> eventMouseButtonClick += MyGUI::newDelegate(this, &GUI::selectedMenuItem);
	}
	if(widget = mGuiSystem->findWidget<MyGUI::Widget>("Command_Reset_Camera"))
	{
		widget-> eventMouseButtonClick += MyGUI::newDelegate(this, &GUI::selectedMenuItem);
	}

	//softbody demos
	if(widget = mGuiSystem->findWidget<MyGUI::Widget>("Command_Softbody_Demo_1"))
	{
		widget-> eventMouseButtonClick += MyGUI::newDelegate(this, &GUI::selectedMenuItem);
	}
	if(widget = mGuiSystem->findWidget<MyGUI::Widget>("Command_Softbody_Demo_2"))
	{
		widget-> eventMouseButtonClick += MyGUI::newDelegate(this, &GUI::selectedMenuItem);
	}
	if(widget = mGuiSystem->findWidget<MyGUI::Widget>("Command_Softbody_Demo_3"))
	{
		widget-> eventMouseButtonClick += MyGUI::newDelegate(this, &GUI::selectedMenuItem);
	}
	if(widget = mGuiSystem->findWidget<MyGUI::Widget>("Command_Softbody_Demo_4"))
	{
		widget-> eventMouseButtonClick += MyGUI::newDelegate(this, &GUI::selectedMenuItem);
	}
	if(widget = mGuiSystem->findWidget<MyGUI::Widget>("Command_Softbody_Demo_5"))
	{
		widget-> eventMouseButtonClick += MyGUI::newDelegate(this, &GUI::selectedMenuItem);
	}

	//cloth demos
	if(widget = mGuiSystem->findWidget<MyGUI::Widget>("Command_Cloth_Demo_1"))
	{
		widget-> eventMouseButtonClick += MyGUI::newDelegate(this, &GUI::selectedMenuItem);
	}
	if(widget = mGuiSystem->findWidget<MyGUI::Widget>("Command_Cloth_Demo_2"))
	{
		widget-> eventMouseButtonClick += MyGUI::newDelegate(this, &GUI::selectedMenuItem);
	}
	if(widget = mGuiSystem->findWidget<MyGUI::Widget>("Command_Cloth_Demo_3"))
	{
		widget-> eventMouseButtonClick += MyGUI::newDelegate(this, &GUI::selectedMenuItem);
	}
	if(widget = mGuiSystem->findWidget<MyGUI::Widget>("Command_Cloth_Demo_4"))
	{
		widget-> eventMouseButtonClick += MyGUI::newDelegate(this, &GUI::selectedMenuItem);
	}
	if(widget = mGuiSystem->findWidget<MyGUI::Widget>("Command_Cloth_Demo_5"))
	{
		widget-> eventMouseButtonClick += MyGUI::newDelegate(this, &GUI::selectedMenuItem);
	}

	//deformable demos
	if(widget = mGuiSystem->findWidget<MyGUI::Widget>("Command_Deformable_Demo_1"))
	{
		widget-> eventMouseButtonClick += MyGUI::newDelegate(this, &GUI::selectedMenuItem);
	}
	if(widget = mGuiSystem->findWidget<MyGUI::Widget>("Command_Deformable_Demo_2"))
	{
		widget-> eventMouseButtonClick += MyGUI::newDelegate(this, &GUI::selectedMenuItem);
	}
	if(widget = mGuiSystem->findWidget<MyGUI::Widget>("Command_Deformable_Demo_3"))
	{
		widget-> eventMouseButtonClick += MyGUI::newDelegate(this, &GUI::selectedMenuItem);
	}
	if(widget = mGuiSystem->findWidget<MyGUI::Widget>("Command_Deformable_Demo_4"))
	{
		widget-> eventMouseButtonClick += MyGUI::newDelegate(this, &GUI::selectedMenuItem);
	}
	if(widget = mGuiSystem->findWidget<MyGUI::Widget>("Command_Deformable_Demo_5"))
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
		Command_Bullet_Debug_Mode = false;
		vp->getCamera()->setPolygonMode(Ogre::PM_SOLID);

		mGuiSystem->findWidget<MyGUI::Widget>("Command_Solid")->setEnabled(false);
		mGuiSystem->findWidget<MyGUI::Widget>("Command_Wireframe")->setEnabled(true);
		mGuiSystem->findWidget<MyGUI::Widget>("Command_Points")->setEnabled(true);
		mGuiSystem->findWidget<MyGUI::Widget>("Command_Bullet_Debug_Mode")->setEnabled(true);
	}
	if(name == "Command_Wireframe")
	{
		Command_Bullet_Debug_Mode = false;
		vp->getCamera()->setPolygonMode(Ogre::PM_WIREFRAME);

		mGuiSystem->findWidget<MyGUI::Widget>("Command_Solid")->setEnabled(true);
		mGuiSystem->findWidget<MyGUI::Widget>("Command_Wireframe")->setEnabled(false);
		mGuiSystem->findWidget<MyGUI::Widget>("Command_Points")->setEnabled(true);
		mGuiSystem->findWidget<MyGUI::Widget>("Command_Bullet_Debug_Mode")->setEnabled(true);
	}
	if(name == "Command_Points")
	{
		Command_Bullet_Debug_Mode = false;
		vp->getCamera()->setPolygonMode(Ogre::PM_POINTS);

		mGuiSystem->findWidget<MyGUI::Widget>("Command_Solid")->setEnabled(true);
		mGuiSystem->findWidget<MyGUI::Widget>("Command_Wireframe")->setEnabled(true);
		mGuiSystem->findWidget<MyGUI::Widget>("Command_Points")->setEnabled(false);
		mGuiSystem->findWidget<MyGUI::Widget>("Command_Bullet_Debug_Mode")->setEnabled(true);
	}
	if(name == "Command_Bullet_Debug_Mode")
	{
		Command_Bullet_Debug_Mode = true;
		mGuiSystem->findWidget<MyGUI::Widget>("Command_Solid")->setEnabled(true);
		mGuiSystem->findWidget<MyGUI::Widget>("Command_Wireframe")->setEnabled(true);
		mGuiSystem->findWidget<MyGUI::Widget>("Command_Points")->setEnabled(true);
		mGuiSystem->findWidget<MyGUI::Widget>("Command_Bullet_Debug_Mode")->setEnabled(false);
	}
	if(name == "Command_ScreenShot")
	{
		Command_ScreenShot = true;
	}
	if(name == "Command_Slow_Motion")
	{
		if(simulationWindow->isVisible())
		{
			simulationWindow->setVisible(false);
		}
		else
		{
			simulationWindow->setVisible(true);
		}

	}
	if(name == "Simulation_Default")
	{
		Simulation_Stop = false;
		mGuiSystem->findWidget<MyGUI::TextBox>("Status_Speed")->setCaption("Simulation Speed: " + MyGUI::utility::toString(0)+"%");
		mGuiSystem->findWidget<MyGUI::ScrollBar>("Status_SpeedBar")->setScrollPosition(50);
		mGuiSystem->findWidget<MyGUI::ScrollBar>("Status_SpeedBar")->setEnabled(true);
		demoDt = 0;

		mGuiSystem->findWidget<MyGUI::Widget>("Command_Enable_Slow")->setEnabled(true);
		mGuiSystem->findWidget<MyGUI::Widget>("Command_Disable_Slow")->setEnabled(false);
		Command_Enable_Slow = false;
		Command_Disable_Slow = false;
	}
	if(name == "Simulation_Stop")
	{
		if(Simulation_Stop)
		{
			Simulation_Stop = false;
			Simulation_Default = true;
			demoDt = lastNum;
			if (Command_Enable_Slow)
			{
				mGuiSystem->findWidget<MyGUI::ScrollBar>("Status_SpeedBar")->setEnabled(false);
			}
			else
			{
				mGuiSystem->findWidget<MyGUI::ScrollBar>("Status_SpeedBar")->setEnabled(true);
			}
		}
		else
		{
			Simulation_Stop = true;
			Simulation_Default = false;
			lastNum = demoDt;
			demoDt = 0;
			mGuiSystem->findWidget<MyGUI::ScrollBar>("Status_SpeedBar")->setEnabled(false);
		}
	}
	if(name == "Command_Enable_Slow")
	{
		Command_Enable_Slow = true;
		mGuiSystem->findWidget<MyGUI::Widget>("Command_Enable_Slow")->setEnabled(false);
		mGuiSystem->findWidget<MyGUI::Widget>("Command_Disable_Slow")->setEnabled(true);
		mGuiSystem->findWidget<MyGUI::ScrollBar>("Status_SpeedBar")->setEnabled(false);
	}
	if(name == "Command_Disable_Slow")
	{
		Command_Disable_Slow = true;
		mGuiSystem->findWidget<MyGUI::Widget>("Command_Enable_Slow")->setEnabled(true);
		mGuiSystem->findWidget<MyGUI::Widget>("Command_Disable_Slow")->setEnabled(false);
		mGuiSystem->findWidget<MyGUI::ScrollBar>("Status_SpeedBar")->setEnabled(true);
	}
	if(name == "Command_Reset_Camera")
	{
		Command_Reset_Camera = true;
	}

	if(name == "Command_Cloth_Demo_1")
	{
		Command_Cloth_Demo_1 = true;
		mGuiSystem->findWidget<MyGUI::Widget>("Command_Cloth_Demo_1")->setEnabled(false);
		mGuiSystem->findWidget<MyGUI::Widget>("Command_Cloth_Demo_2")->setEnabled(true);
		mGuiSystem->findWidget<MyGUI::Widget>("Command_Cloth_Demo_3")->setEnabled(true);
		mGuiSystem->findWidget<MyGUI::Widget>("Command_Cloth_Demo_4")->setEnabled(true);
		mGuiSystem->findWidget<MyGUI::Widget>("Command_Cloth_Demo_5")->setEnabled(true);
	}
	if(name == "Command_Cloth_Demo_2")
	{
		Command_Cloth_Demo_2 = true;
		mGuiSystem->findWidget<MyGUI::Widget>("Command_Cloth_Demo_1")->setEnabled(true);
		mGuiSystem->findWidget<MyGUI::Widget>("Command_Cloth_Demo_2")->setEnabled(false);
		mGuiSystem->findWidget<MyGUI::Widget>("Command_Cloth_Demo_3")->setEnabled(true);
		mGuiSystem->findWidget<MyGUI::Widget>("Command_Cloth_Demo_4")->setEnabled(true);
		mGuiSystem->findWidget<MyGUI::Widget>("Command_Cloth_Demo_5")->setEnabled(true);
	}
	if(name == "Command_Cloth_Demo_3")
	{
		Command_Cloth_Demo_3 = true;
		mGuiSystem->findWidget<MyGUI::Widget>("Command_Cloth_Demo_1")->setEnabled(true);
		mGuiSystem->findWidget<MyGUI::Widget>("Command_Cloth_Demo_2")->setEnabled(true);
		mGuiSystem->findWidget<MyGUI::Widget>("Command_Cloth_Demo_3")->setEnabled(false);
		mGuiSystem->findWidget<MyGUI::Widget>("Command_Cloth_Demo_4")->setEnabled(true);
		mGuiSystem->findWidget<MyGUI::Widget>("Command_Cloth_Demo_5")->setEnabled(true);
	}
	if(name == "Command_Cloth_Demo_4")
	{
		Command_Cloth_Demo_4 = true;
		mGuiSystem->findWidget<MyGUI::Widget>("Command_Cloth_Demo_1")->setEnabled(true);
		mGuiSystem->findWidget<MyGUI::Widget>("Command_Cloth_Demo_2")->setEnabled(true);
		mGuiSystem->findWidget<MyGUI::Widget>("Command_Cloth_Demo_3")->setEnabled(true);
		mGuiSystem->findWidget<MyGUI::Widget>("Command_Cloth_Demo_4")->setEnabled(false);
		mGuiSystem->findWidget<MyGUI::Widget>("Command_Cloth_Demo_5")->setEnabled(true);
	}
	if(name == "Command_Cloth_Demo_5")
	{
		Command_Cloth_Demo_5 = true;
		mGuiSystem->findWidget<MyGUI::Widget>("Command_Cloth_Demo_1")->setEnabled(true);
		mGuiSystem->findWidget<MyGUI::Widget>("Command_Cloth_Demo_2")->setEnabled(true);
		mGuiSystem->findWidget<MyGUI::Widget>("Command_Cloth_Demo_3")->setEnabled(true);
		mGuiSystem->findWidget<MyGUI::Widget>("Command_Cloth_Demo_4")->setEnabled(true);
		mGuiSystem->findWidget<MyGUI::Widget>("Command_Cloth_Demo_5")->setEnabled(false);
	}


	if(name == "Command_Softbody_Demo_1")
	{
		Command_Softbody_Demo_1 = true;
	}
	if(name == "Command_Softbody_Demo_2")
	{
		Command_Softbody_Demo_2 = true;
	}
	if(name == "Command_Softbody_Demo_3")
	{
		Command_Softbody_Demo_3 = true;
	}
	if(name == "Command_Softbody_Demo_4")
	{
		Command_Softbody_Demo_4 = true;
	}
	if(name == "Command_Softbody_Demo_5")
	{
		Command_Softbody_Demo_5 = true;
	}


	if(name == "Command_Deformable_Demo_1")
	{
		Command_Deformable_Demo_1 = true;
	}
	if(name == "Command_Deformable_Demo_2")
	{
		Command_Deformable_Demo_2 = true;
	}
	if(name == "Command_Deformable_Demo_3")
	{
		Command_Deformable_Demo_3 = true;
	}
	if(name == "Command_Deformable_Demo_4")
	{
		Command_Deformable_Demo_4 = true;
	}
	if(name == "Command_Deformable_Demo_5")
	{
		Command_Deformable_Demo_5 = true;
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

void GUI::createSimulationSpeedWindow(void)
{
	const MyGUI::IntSize& size = MyGUI::RenderManager::getInstance().getViewSize();

	simulationPtr = MyGUI::LayoutManager::getInstance().loadLayout("Simulation.layout");

	simulationWindow = mGuiSystem->findWidget<MyGUI::Window>("Simulation_Window");
	simulationWindow->eventWindowButtonPressed += MyGUI::newDelegate(this, &GUI::selectedWindowItem);
	simulationWindow->setVisible(false);
	simulationWindow->setPosition(size.width - 360, size.height - (size.height - 26));

	MyGUI::Button* button = mGuiSystem->findWidget<MyGUI::Button>("Simulation_Default");
	button->eventMouseButtonClick += MyGUI::newDelegate(this, &GUI::selectedMenuItem);

	button = mGuiSystem->findWidget<MyGUI::Button>("Simulation_Stop");
	button->eventMouseButtonClick += MyGUI::newDelegate(this, &GUI::selectedMenuItem);

	mGuiSystem->findWidget<MyGUI::ScrollBar>("Status_SpeedBar")->eventScrollChangePosition += newDelegate(this, &GUI::modifySimulationSpeed);
}


void GUI::selectedWindowItem(MyGUI::Window* widget, const std::string& name)
{
	MyGUI::Window* window = widget->castType<MyGUI::Window>(); 
	if (name == "close")
	{
		simulationWindow->setVisible(false);
	}
	else if (name == "minimized") 
	{ 
		// hide window and show button in your taskbar 
	} 
	else if (name == "maximized") 
	{ 
		// maximized window 
	} 
}

void GUI::modifySimulationSpeed(MyGUI::ScrollBar* sender, size_t pos)
{
	if(pos > 50)
	{
		mGuiSystem->findWidget<MyGUI::TextBox>("Status_Speed")->setCaption("Simulation Speed: " + MyGUI::utility::toString(pos - 50)+"%");
		if(!Simulation_Stop)
		{
			demoDt = (double)(pos - 50)/1000;
		}
	}
	if(pos < 50)
	{
		mGuiSystem->findWidget<MyGUI::TextBox>("Status_Speed")->setCaption("Simulation Speed: -" + MyGUI::utility::toString(50 - pos)+"%");
		if(!Simulation_Stop)
		{
			demoDt = (double)(-((50 - pos)/100));
		}
	}
}

double GUI::demoSpeed(double dt)
{
	return dt += demoDt;
}

// void GUI::createFPSWindow(void)
// {
// 	const MyGUI::IntSize& size = MyGUI::RenderManager::getInstance().getViewSize();
// 
// 	FPSWindow = mGuiSystem->findWidget<MyGUI::Window>("FPSWindow");
// 	FPSWindow->setPosition(10, size.height-FPSWindow->getHeight()-10);
// 	FPSWindow->setVisible(true);
// }


