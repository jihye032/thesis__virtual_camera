#include "BisectorSurfaceTestAni3.h"
#include "BaseLib/GL4U/GL_ResourceManager.h"
#include "BaseLib/FLTKU/AnimationViewer.h"
#include "BaseLib/CmlExt/CmlCamera.h"
#include "BaseLib/CmlExt/CmlExt.h"
#include "FL/fl.h"
#include <iostream>
#include <queue>



BisectorSurfaceTestAni3::BisectorSurfaceTestAni3()
{
	camera1_.lookAt({ 300., 300., 300. }, { 0., 0., 0. }, { 0., 1., 0. });
	final_ani_time_ms(2000);


	car_nor_.zero();
	dot_nor_.zero();


	/*/
	mg::Mesh mesh_a_, mesh_b_, mesh_c_;
	mesh_a_.CreateBox(100, 100, 100);
	mesh_b_.CreateBox(100, 100, 100);
	mesh_c_.CreateBox(100, 50, 100);

	mesh_a_.TranslateVertices(cml::vector3d(-100, 100, 0));
	mesh_b_.TranslateVertices(cml::vector3d(100, 100, 0));
	mesh_c_.TranslateVertices(cml::vector3d(0, 75, 0));

	mesh_B_.Merge(mesh_a_);
	mesh_B_.Merge(mesh_b_);
	mesh_B_.Merge(mesh_c_);
	mesh_B_.Triangulate();


	//mesh_joint_.CreateSphere(10);
	mesh_joint_.CreateBox(10, 10, 10);

	finger_joint[0].zero();
	finger_joint[1].zero();
	finger_joint[2] = cml::vector3d(0, 125, 0);
	

	scene_root_ = std::make_unique<mg::GL_SceneRoot>();
	mg::GL_ResourceManager::singleton()->ImportByAssimp(scene_root_.get(), "../Data/pointer.fbx", "playground");
	//mg::GL_ResourceManager::singleton()->ImportByAssimp(scene_root_.get(), "../Data/obj/pointer.obj", "pointer");
	scene_root_->BuildSinglePolygonMesh(&mesh_joint_obj_);
	mesh_joint_obj_.ScaleUniformlyVertices(10);
	mesh_joint_obj_.Triangulate();
	*/

	public_cmr_pos = cml::vector3d(0, 120, 60);
	cml::vector3d joint_pos = cml::vector3d(0, 100, 20);
	finger_joint[0].zero();
	finger_joint[1].zero();
	finger_joint[2] = joint_pos;


	mesh_cmr_.CreateSphere(6);
	mesh_cmr_.TranslateVertices(public_cmr_pos);
	mesh_cmr_.Triangulate();

	//mesh_joint_.CreateSphere(10);
	mesh_joint_.CreateBox(10, 10, 10);
	mesh_joint_.TranslateVertices(joint_pos);


	mg::Mesh a, b, c, d, e, f, g;
	a.CreateCylinder(20, 50);
	b.CreateCylinder(20, 50);
	c.CreateCylinder(20, 50);
	d.CreateCylinder(20, 50);
	e.CreateCylinder(20, 50);
	f.CreateCylinder(20, 50);
	g.CreateCylinder(20, 50);

	a.TranslateVertices(cml::vector3d(0, 60, 0));
	b.TranslateVertices(cml::vector3d(0, 80, 0));
	c.TranslateVertices(cml::vector3d(0, 100, 0));
	d.TranslateVertices(cml::vector3d(0, 120, 0));
	e.TranslateVertices(cml::vector3d(0, 140, 0));
	f.TranslateVertices(cml::vector3d(0, 160, 0));
	g.TranslateVertices(cml::vector3d(0, 180, 0));
	mesh_joint_obj_.Merge(a);
	mesh_joint_obj_.Merge(b);
	mesh_joint_obj_.Merge(c);
	mesh_joint_obj_.Merge(d);
	mesh_joint_obj_.Merge(e);
	mesh_joint_obj_.Merge(f);
	mesh_joint_obj_.Merge(g);


	mesh_joint_obj_.Triangulate();


	scene_root_ = std::make_unique<mg::GL_SceneRoot>();
	mg::GL_ResourceManager::singleton()->ImportByAssimp(scene_root_.get(), "../Data/obj/torus2.obj", "torus");
	scene_root_->BuildSinglePolygonMesh(&mesh_B_);
	mesh_B_.ScaleUniformlyVertices(25);
	mesh_B_.TranslateVertices(cml::vector3d(0, 100, 0));
	mesh_B_.Triangulate();







	num_leap_hands_ = 1;

}

void
BisectorSurfaceTestAni3::SetCurrentTime(mg::AniTime_ms t)
{
	if (t<0) cur_time_ms_ = 0;
	//else if (t >= final_ani_time_ms_) cur_time_ms_ = final_ani_time_ms_;
	else cur_time_ms_ = t;

	//mg::AniTime_ms pre_time_ms = cur_time_ms_;


	static int frmae_id = -1;
	if (frmae_id == GetCurFrameId()) return;

	frmae_id = GetCurFrameId();


	if (num_leap_hands_ >= 1)
	{
		///////////////////////////////////////////////////////
		// set user camera
		//cml::vector3d cmr_pos = FindSubCameraPosition();
		cml::vector3d cmr_pos = cml::vector3d(public_cmr_pos[0] , public_cmr_pos[1]*1.5, public_cmr_pos[2] * 2);
		cml::vector3d eye(cmr_pos);
		cml::vector3d center(finger_joint[2]);
		cml::vector3d up(cml::vector3d(0, 1, 0));
		camera1_.lookAt(eye, center, up);

		//mesh_joint_obj_.TranslateVertices(finger_joint[2] - finger_joint_p[2]);
		//mesh_joint_obj_.Triangulate();

		//mesh_joint_.TranslateVertices(finger_joint[2] - finger_joint_p[2]);
		//mesh_joint_.Triangulate();


		b_surface_.SetVertices(mesh_B_.num_vertices(), mesh_B_.vertices(),
			mesh_joint_obj_.num_vertices(), mesh_joint_obj_.vertices());
		b_surface_.Update();
	}
	else
		find_camera_ = false;


	NotifyAnimationUpdated();
}


void
BisectorSurfaceTestAni3::Draw(mg::AnimationViewer *w)
{
	int win_w = w->w();
	int win_h = w->h();


	//renderer_->SetModelIdentity();
	renderer_->UseDefaultCamera();
	DrawInSubViewport(0, 0, win_w, win_h, 0);


	// Right-Top PIP
	renderer_->UseCamera(&camera1_);
	DrawInSubViewport2(win_w - win_w / 3, win_h - win_h / 3, win_w / 3, win_h / 3, 3);

	renderer_->UseDefaultCamera();

}

void
BisectorSurfaceTestAni3::DrawInSubViewport(int vp_x, int vp_y, int vp_w, int vp_h, int border_width = 0)
{
	// Define the subviewport
	glViewport(vp_x, vp_y, vp_w, vp_h);
	glEnable(GL_SCISSOR_TEST);

	// Border
	if (border_width > 0)
	{
		glScissor(vp_x, vp_y, vp_w, vp_h);
		glClearColor(0.0, 0.0, 0.0, 1.0);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	}

	// Clear
	glScissor(vp_x + border_width, vp_y + border_width, vp_w - 2 * border_width, vp_h - 2 * border_width);
	glClearColor(1.0, 1.0, 1.0, 1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


	renderer_->SetColor(0);
	renderer_->DrawMesh(&mesh_B_);


	if (num_leap_hands_ >= 1)
	{
		renderer_->SetColor(2);
		renderer_->DrawMesh(&mesh_joint_obj_);

		renderer_->SetColor(10);
		renderer_->DrawMesh(&mesh_cmr_);


		renderer_->EnableTransparency(true);
		renderer_->SetColor(3);
		DrawBisector();
		renderer_->EnableTransparency(false);
	}

	glDisable(GL_SCISSOR_TEST);
}


void
BisectorSurfaceTestAni3::DrawInSubViewport2(int vp_x, int vp_y, int vp_w, int vp_h, int border_width = 0)
{
	// Define the subviewport
	glViewport(vp_x, vp_y, vp_w, vp_h);
	glEnable(GL_SCISSOR_TEST);

	// Border
	if (border_width > 0)
	{
		glScissor(vp_x, vp_y, vp_w, vp_h);
		glClearColor(0.0, 0.0, 0.0, 1.0);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	}

	// Clear
	glScissor(vp_x + border_width, vp_y + border_width, vp_w - 2 * border_width, vp_h - 2 * border_width);
	glClearColor(1.0, 1.0, 1.0, 1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	renderer_->SetColor(0);
	renderer_->DrawMesh(&mesh_B_);


	if (num_leap_hands_ >= 1)
	{
		renderer_->SetColor(2);
		renderer_->DrawMesh(&mesh_joint_obj_);
	}

	glDisable(GL_SCISSOR_TEST);
}


static bool CheckBoundOut(cml::vector3d v)
{
	double max_bound = 1000.;
	double min_bound = -1000.;

	for (int i = 0; i < 3; i++)
		if (v[i] > max_bound) return true;

	for (int i = 0; i < 3; i++)
		if (v[i] < min_bound) return true;

	return false;
}

void
BisectorSurfaceTestAni3::DrawBisector()
{

	///////////////////////////////////////////////////////////////////////////////
	// sites
	if (false)
	{
		renderer_->SetColor(0, 0, 0);
		renderer_->glBegin(GL_LINES);
		for (unsigned int i = 0; i < b_sites.size() && i < b_faces.size(); i++)
		{
			// Boundary Check
			bool out_bound = false;
			for (int fv_i = 0; fv_i < (int)b_faces[i].size(); fv_i++)
			{
				if (CheckBoundOut(b_vertices[b_faces[i][fv_i]]))
				{
					out_bound = true;
				}
			}
			if (out_bound) continue;

			renderer_->glVertex(b_surface_.GetVertexOfA(b_sites[i].first));
			renderer_->glVertex(b_surface_.GetVertexOfB(b_sites[i].second));
		}
		renderer_->glEnd();
	}


	renderer_->SetColor(1, 0.1);
	//glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	for (int i = 0; i<(int)b_faces.size(); i++)
	{
		// Boundary Check
		bool out_bound = false;
		for (int fv_i = 0; fv_i<(int)b_faces[i].size(); fv_i++)
		{
			if (CheckBoundOut(b_vertices[b_faces[i][fv_i]]))
			{
				out_bound = true;
			}
		}
		if (out_bound) continue;

		renderer_->glBegin(GL_TRIANGLE_FAN);
		renderer_->glNormal(b_normals[i]);
		for (int fv_i = 0; fv_i<(int)b_faces[i].size(); fv_i++)
		{
			renderer_->glVertex(b_vertices[b_faces[i][fv_i]]);
		}
		renderer_->glEnd();


		renderer_->glBegin(GL_TRIANGLE_FAN);
		renderer_->glNormal(-1 * b_normals[i]);
		for (int fv_i = (int)b_faces[i].size() - 1; fv_i >= 0; fv_i--)
		{
			renderer_->glVertex(b_vertices[b_faces[i][fv_i]]);
		}
		renderer_->glEnd();
	}





	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	//renderer_->DisableAllLight();
	glLineWidth(2);
	renderer_->SetColor(0, 0, 0, 0.5);
	for (int i = 0; i<(int)b_faces.size(); i++)
	{
		/*if ( std::find(b_faces[i].begin(), b_faces[i].end(), 0) != b_faces[i].end() )
		{
		continue;
		}*/

		// Boundary Check
		bool out_bound = false;
		for (int fv_i = 0; fv_i<(int)b_faces[i].size(); fv_i++)
		{
			if (CheckBoundOut(b_vertices[b_faces[i][fv_i]]))
			{
				out_bound = true;
			}
		}
		if (out_bound) continue;

		renderer_->glBegin(GL_LINE_STRIP);
		renderer_->glNormal(b_normals[i]);
		for (int fv_i = 0; fv_i<(int)b_faces[i].size(); fv_i++)
		{
			renderer_->glVertex(b_vertices[b_faces[i][fv_i]]);
		}
		renderer_->glVertex(b_vertices[b_faces[i][0]]);
		renderer_->glEnd();


	}
	renderer_->EnableLight(0);




	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);


}

static bool CheckVertexOut(cml::vector3d v, cml::vector3d env, cml::vector3d ptr)
{
	double bound = 300.;


	for (int i = 0; i < 3; i++)
	{
		if (env[i] > ptr[i])
			if (v[i] > ptr[i] - bound && v[i] < env[i] + bound) return true;
			else if (env[i] < ptr[i])
				if (v[i] > env[i] - bound && v[i] < ptr[i] + bound) return true;
	}

	return false;
}

cml::vector3d
BisectorSurfaceTestAni3::FindSubCameraPosition()
{

	// 중심점
	/*cml::vector3d cnt_point;
	cnt_point.zero();
	for (int i = 0; i < b_vertices.size(); i++)
	{
	cnt_point += b_vertices[i];
	}
	cnt_point /= b_vertices.size();

	int num = 0;
	double dis = 10000;
	for (int i = 0; i < b_vertices.size(); i++)
	{
	double tmp = pow(cnt_point[0] - b_vertices[i][0], 2) + pow(cnt_point[2] - b_vertices[i][2], 2);

	if (tmp <= dis)		num = i;
	}
	cnt_point = b_vertices[num];

	dot_nor_ = cnt_point;*/



	// 거리
	double min_dis = 99999999999999999;
	cml::vector3d env_vtx_, ptr_vtx_;
	env_vtx_.zero();
	ptr_vtx_.zero();
	std::vector<double> b_dis_joint2envi;
	for (int i = 0; i < b_sites.size(); i++)
	{
		cml::vector3d a = mesh_B_.vertex(b_sites[0].first);
		cml::vector3d b = mesh_joint_.vertex(b_sites[0].second);
		double dis = sqrt(pow(a[0] - b[0], 2) + pow(a[1] - b[1], 2) + pow(a[2] - b[2], 2));

		b_dis_joint2envi.push_back(dis);


		if (min_dis > dis)
		{
			min_dis = dis;
			env_vtx_ = a;
			ptr_vtx_ = b;

			//std::cout << "min dis : " << min_dis << std::endl;
		}
	}



	for (int id = 0; id < b_vertices.size(); id++)
	{
		if (CheckVertexOut(b_vertices[id], env_vtx_, ptr_vtx_))
		{
			cmr_pos_id_ = id;
		}
	}

	std::cout << " cmr_ pos : " << cmr_pos_id_ << std::endl;
	//dot_nor_ = b_vertices[cmr_pos_id_];


	/*
	// normal vectors
	for (int i = 0; i < b_normals.size(); i++)
	{
	cml::vector3d nor, tmp;

	if (i == 0)
	{
	dot_nor_ = b_normals[i];
	}
	else
	{
	nor = b_normals[i];
	tmp = b_normals[i + 1];

	double n_x = nor[1] * tmp[2] - nor[2] * tmp[1];
	double n_y = nor[2] * tmp[0] - nor[0] * tmp[2];
	double n_z = nor[0] * tmp[1] - nor[1] * tmp[0];

	dot_nor_ = cml::vector3d(n_x, n_y, n_z);
	}
	}

	int x=finger_joint[0][0], y=0, z=finger_joint[2][2];
	{
	int tmp = dot_nor_[0] * x + dot_nor_[1] * y + dot_nor_[2] * z;
	for(int i= -50; i<=100 && tmp!=0; i++)
	{
	tmp = dot_nor_[0] * x + dot_nor_[1] * y + dot_nor_[2] * z;
	}


	dot_nor_ = cml::vector3d(x, y, z);
	}
	*/




	////////////////////////////////////////////////////////////
	// normal vector의 값이 일정 크기 이상 변해야지만 카메라의 위치가 바뀌도록 함
	double tmp = pow(car_nor_[0] - dot_nor_[0], 2)
		+ pow(car_nor_[1] - dot_nor_[1], 2)
		+ pow(car_nor_[2] - dot_nor_[2], 2);

	if (tmp < 10) 	dot_nor_ = car_nor_;
	else			dot_nor_ = dot_nor_;

	car_nor_ = dot_nor_;

	return dot_nor_;
}



/*
/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
//
// Leap motion function

const std::string fingerNames[] = { "Thumb", "Index", "Middle", "Ring", "Pinky" };
const std::string boneNames[] = { "Metacarpal", "Proximal", "Middle", "Distal" };

void BisectorSurfaceTestAni3::onInit(const Leap::Controller& controller) {
std::cout << "Initialized" << std::endl;

finger_joint[0].zero();
finger_joint[1].zero();
finger_joint[2].zero();

finger_joint_p[0].zero();
finger_joint_p[1].zero();
finger_joint_p[2].zero();
}

void BisectorSurfaceTestAni3::onConnect(const Leap::Controller& controller) {
std::cout << "Connected" << std::endl;
}

void BisectorSurfaceTestAni3::onDisconnect(const Leap::Controller& controller) {
// Note: not dispatched when running in a debugger.
std::cout << "Disconnected" << std::endl;
}

void BisectorSurfaceTestAni3::onExit(const Leap::Controller& controller) {
std::cout << "Exited" << std::endl;
}

void BisectorSurfaceTestAni3::onFrame(const Leap::Controller& controller) {


// Get the most recent frame and report some basic information
const Leap::Frame frame = controller.frame();

Leap::HandList hands = frame.hands();
num_leap_hands_ = hands.count();

for (Leap::HandList::const_iterator hl = hands.begin(); hl != hands.end(); ++hl) {
// Get the first hand
const Leap::Hand hand = *hl;

// Get the hand's normal vector and direction
const Leap::Vector normal = hand.palmNormal();		// 손바닥의 방향
const Leap::Vector direction = hand.direction();

// Get fingers
const Leap::FingerList fingers = hand.fingers();
for (Leap::FingerList::const_iterator fl = fingers.begin(); fl != fingers.end(); ++fl) {
const Leap::Finger finger = *fl;
//			std::cout << std::string(4, ' ') << fingerNames[finger.type()]
//				<< " finger, id: " << finger.id()
//				<< ", length: " << finger.length()
//				<< "mm, width: " << finger.width() << std::endl;
// finger type : TYPE_THUMB, TYPE_INDEX, TYPE_MIDDLE, TYPE_RING, TYPE_PINKY

// Get finger bones
for (int b = 0; b < 4; ++b) {
Leap::Bone::Type boneType = static_cast<Leap::Bone::Type>(b);
Leap::Bone bone = finger.bone(boneType);
//					std::cout << std::string(6, ' ') << boneNames[boneType]
//						<< " bone, start: " << bone.prevJoint()
//						<< ", end: " << bone.nextJoint()
//						<< ", direction: " << bone.direction() << std::endl;


// 검지 손가락만 골라내기
if (fingerNames[finger.type()] == "Index" && b != 3)
{
finger_joint[b] = cml::vector3d(bone.nextJoint().x, bone.nextJoint().y, bone.nextJoint().z);
//std::cout << "finger_joint : " << finger_joint[b] << std::endl;
}

}

}
}

}

void BisectorSurfaceTestAni3::onFocusGained(const Leap::Controller& controller) {
std::cout << "Focus Gained" << std::endl;
}

void BisectorSurfaceTestAni3::onFocusLost(const Leap::Controller& controller) {
std::cout << "Focus Lost" << std::endl;
}

void BisectorSurfaceTestAni3::onDeviceChange(const Leap::Controller& controller) {
std::cout << "Device Changed" << std::endl;
const Leap::DeviceList devices = controller.devices();

for (int i = 0; i < devices.count(); ++i) {
std::cout << "id: " << devices[i].toString() << std::endl;
std::cout << "  isStreaming: " << (devices[i].isStreaming() ? "true" : "false") << std::endl;
std::cout << "  isSmudged:" << (devices[i].isSmudged() ? "true" : "false") << std::endl;
std::cout << "  isLightingBad:" << (devices[i].isLightingBad() ? "true" : "false") << std::endl;
}
}

void BisectorSurfaceTestAni3::onServiceConnect(const Leap::Controller& controller) {
std::cout << "Service Connected" << std::endl;
}

void BisectorSurfaceTestAni3::onServiceDisconnect(const Leap::Controller& controller) {
std::cout << "Service Disconnected" << std::endl;
}

void BisectorSurfaceTestAni3::onServiceChange(const Leap::Controller& controller) {
std::cout << "Service Changed" << std::endl;
}

void BisectorSurfaceTestAni3::onDeviceFailure(const Leap::Controller& controller) {
std::cout << "Device Error" << std::endl;
const Leap::FailedDeviceList devices = controller.failedDevices();

for (Leap::FailedDeviceList::const_iterator dl = devices.begin(); dl != devices.end(); ++dl) {
const Leap::FailedDevice device = *dl;
std::cout << "  PNP ID:" << device.pnpId();
std::cout << "    Failure type:" << device.failure();
}
}

void BisectorSurfaceTestAni3::onLogMessage(const Leap::Controller&, Leap::MessageSeverity s, int64_t t, const char* msg) {
switch (s) {
case Leap::MESSAGE_CRITICAL:
std::cout << "[Critical]";
break;
case Leap::MESSAGE_WARNING:
std::cout << "[Warning]";
break;
case Leap::MESSAGE_INFORMATION:
std::cout << "[Info]";
break;
case Leap::MESSAGE_UNKNOWN:
std::cout << "[Unknown]";
}
std::cout << "[" << t << "] ";
std::cout << msg << std::endl;
}

*/