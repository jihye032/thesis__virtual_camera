


#include "GL/glew.h"
#include "GL/freeglut.h"
#include <iostream>
#include <memory>
#include "cml/cml.h"
#include "BaseLib/FLTKU/Fl_Glew_Window.h"
#include "BaseLib/FLTKU/AnimationApp.h"
#include "BaseLib/FLTKU/CharacterAnimation.h"
#include "BaseLib/FLTKU/Animation.h"
#include "BaseLib/GL4U/GL_VBOVAO.h"
#include "BaseLib/GL4U/GL_Material.h"
#include "BaseLib/GL4U/GL_Renderer.h"
#include "BaseLib/GL4U/GL_ResourceManager.h"
#include "BaseLib/Algorithm/Graph.h"
#include "BisectorSurfaceTestAni.h"
#include "BisectorSurfaceTestAni2.h"
#include "BisectorSurfaceTestAni3.h"
#include "BisectorSurfaceTestAni4.h"
#include "BisectorSurfaceTestAni5.h"
#include "Leap/Leap.h"


using namespace Leap;


int
main(int argc, char** argv)
{
	glutInit(&argc, argv);
	mg::InitFlGlew();

	
	mg::AnimationApp app;
	app.end();
	//app.ani_time_control_->SetPlayFPS_ms(10);
	app.ani_time_control_->play_mode(mg::AnimationTimeController::TO_INFINITE);


	



	//PmHuman *human = new PmHuman("Data/wd2.actor");

	BisectorSurfaceTestAni *ani = new BisectorSurfaceTestAni;
	ani->name("Bisector Test");
	app.ani_browser_->AddAnimation(ani);
	
	BisectorSurfaceTestAni2 *ani2 = new BisectorSurfaceTestAni2;
	ani2->name("Bisector Test2");
	app.ani_browser_->AddAnimation(ani2);
	
	BisectorSurfaceTestAni3 *ani3 = new BisectorSurfaceTestAni3;
	ani3->name("Bisector Test3");
	app.ani_browser_->AddAnimation(ani3);

	BisectorSurfaceTestAni4 *ani4 = new BisectorSurfaceTestAni4;
	ani4->name("Bisector Test4");
	app.ani_browser_->AddAnimation(ani4);

	BisectorSurfaceTestAni5 *ani5 = new BisectorSurfaceTestAni5;
	ani5->name("Bisector Test5");
	app.ani_browser_->AddAnimation(ani5);

	app.ani_browser_->SelectAni(4);

	app.show();
	
	//app.ani_frame_control_->play_mode(mg::AnimationFrameController::TO_INFINITE);
	

	
	/////////////////////////////////////////////////////////////////////////////
	// Leap motion ÃÊ±âÈ­ 
	Controller controller;
	{
		// Have the sample listener receive events from the controller
		controller.addListener(*ani);
		controller.addListener(*ani2);
		controller.addListener(*ani3);
		controller.addListener(*ani4);
		controller.addListener(*ani5);

		if (argc > 1 && strcmp(argv[1], "--bg") == 0)
			controller.setPolicy(Leap::Controller::POLICY_BACKGROUND_FRAMES);

		controller.setPolicy(Leap::Controller::POLICY_ALLOW_PAUSE_RESUME);

		// Keep this process running until Enter is pressed
		std::cout << "Press Enter to quit, or enter 'p' to pause or unpause the service..." << std::endl;
	}
	


	Fl::run();

	mg::DestroyFlGlew();

	return 0;
}

